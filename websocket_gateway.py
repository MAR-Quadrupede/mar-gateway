#!/usr/bin/env python3
"""
M.A.R. — WebSocket & Telemetry API Gateway
Bridges internal MQTT topics to web browsers and operator dashboards via WebSocket.
"""

import os
import sys
import json
import time
import queue
import asyncio
import signal
import threading
from typing import Optional, Set, Dict, Any, Callable

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

try:
    import websockets
except ImportError:
    websockets = None

from lora_telemetry import LoRaRadio, LoRaBridge


def env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in ("true", "1", "yes", "on")


MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
WS_PORT = int(os.getenv("WS_PORT", "8765"))

# LoRa long-range telemetry link towards the ground station
LORA_ENABLED = env_bool("LORA_ENABLED", False)
LORA_BACKEND = os.getenv("LORA_BACKEND", "auto")
LORA_PORT = os.getenv("LORA_PORT", "/dev/ttyUSB0")
LORA_BAUD = int(os.getenv("LORA_BAUD", "9600"))
LORA_SPI_BUS = int(os.getenv("LORA_SPI_BUS", "0"))
LORA_SPI_DEV = int(os.getenv("LORA_SPI_DEV", "0"))
LORA_CS_PIN = int(os.getenv("LORA_CS_PIN", "5"))
LORA_RST_PIN = int(os.getenv("LORA_RST_PIN", "25"))
LORA_FREQ_MHZ = float(os.getenv("LORA_FREQ_MHZ", "868.0"))
LORA_TX_INTERVAL = float(os.getenv("LORA_TX_INTERVAL", "1.0"))

# LiDAR scan → browser: abbastanza denso per radar, senza saturare Tailscale
LIDAR_SCAN_MIN_INTERVAL_S = float(os.getenv("LIDAR_SCAN_MIN_INTERVAL_S", "0.15"))
LIDAR_SCAN_MAX_POINTS = int(os.getenv("LIDAR_SCAN_MAX_POINTS", "180"))
FSM_MIN_INTERVAL_S = float(os.getenv("FSM_MIN_INTERVAL_S", "0.1"))

connected_clients: Set = set()
mqtt_client = None
lora_bridge: Optional[LoRaBridge] = None
outbound_q: queue.Queue = queue.Queue(maxsize=500)
latest_by_topic: Dict[str, Any] = {}
_last_forward: Dict[str, float] = {}
_mqtt_msg_count = 0


def _should_forward(topic: str) -> bool:
    """Drop or throttle noisy topics before they hit the browser."""
    now = time.time()
    if topic == "mar/lidar/scan":
        last = _last_forward.get(topic, 0.0)
        if now - last < LIDAR_SCAN_MIN_INTERVAL_S:
            return False
        _last_forward[topic] = now
        return True
    if topic == "mar/fsm/state":
        last = _last_forward.get(topic, 0.0)
        if now - last < FSM_MIN_INTERVAL_S:
            return False
        _last_forward[topic] = now
        return True
    return True


def _lighten_payload(topic: str, data: Any) -> Any:
    """Keep a dense-enough LiDAR cloud for the radar widget (≤ LIDAR_SCAN_MAX_POINTS)."""
    if topic != "mar/lidar/scan" or not isinstance(data, dict):
        return data
    points = data.get("points") or []
    n = len(points)
    max_pts = max(36, LIDAR_SCAN_MAX_POINTS)
    if n > max_pts:
        step = max(1, n // max_pts)
        points = points[::step][:max_pts]
    return {
        "timestamp": data.get("timestamp", time.time()),
        "point_count": int(data.get("point_count") or n),
        "cloud_count": len(points),
        "sector_mins": data.get("sector_mins", {}),
        "front_distance_m": data.get("front_distance_m"),
        "points": points,
    }


class GatewayCommandHub:
    """
    Centro Unificato di Ingestione Comandi del Gateway M.A.R.
    Riceve le direttive da entrambi i vettori di comunicazione:
      1. WebSocket (IP / Tailscale / LAN / Internet)
      2. Radio LoRa (868 MHz / porta seriale locale / SPI)

    Centralizza validazione, normalizzazione, deduplicazione e dispatch
    verso il bus MQTT (mar/safety/*, mar/control/*).
    """

    def __init__(self, get_mqtt_client: Callable[[], Any]):
        self.get_mqtt_client = get_mqtt_client
        self._lock = threading.Lock()
        self.last_cmd: Optional[str] = None
        self.last_param: Optional[str] = None
        self.last_kind: Optional[str] = None
        self.last_source: Optional[str] = None
        self.last_ts: float = 0.0
        self.total_commands: int = 0

    def dispatch_from_websocket(self, raw_data: Dict[str, Any]) -> bool:
        """Entry point unificato per comandi ricevuti via WebSocket."""
        return self._handle_command(source="WEBSOCKET", data=raw_data)

    def dispatch_from_lora(self, raw_line: str) -> bool:
        """Entry point unificato per comandi ricevuti via Radio LoRa 868 MHz."""
        return self._handle_command(source="LORA_RADIO", data=raw_line)

    def _handle_command(self, source: str, data: Any) -> bool:
        mqtt = self.get_mqtt_client()
        if not mqtt or not mqtt.is_connected():
            print(f"⚠️ [GATEWAY HUB] Ricevuto comando da {source} ma MQTT non è connesso!")
            return False

        now = time.time()
        kind = ""
        cmd = ""
        param = ""
        reason = ""

        # --- A. Ingestione da Radio LoRa (stringa ASCII grezza) ---
        if isinstance(data, str):
            clean = data.strip()
            if not clean or clean.startswith("#"):
                return False

            # Log e tracciamento frame raw LoRa su MQTT
            mqtt.publish("mar/lora/rx", json.dumps({"timestamp": now, "payload": clean}))

            # Se fosse una stringa JSON
            if clean.startswith("{") and clean.endswith("}"):
                try:
                    data = json.loads(clean)
                except Exception:
                    pass

        # --- B. Estrazione semantica del comando ---
        if isinstance(data, dict):
            msg_type = str(data.get("type", "")).upper()
            if msg_type in ("ESTOP", "E-STOP", "E-CUT"):
                kind = "ESTOP"
                reason = data.get("reason", f"DASHBOARD_{source}")
            elif msg_type in ("RESUME", "RESET"):
                kind = "RESUME"
                reason = data.get("reason", f"DASHBOARD_RESUME_{source}")
            elif msg_type == "MOTION_CMD":
                kind = "MOTION"
                cmd_data = data.get("data", {})
                cmd = str(cmd_data.get("cmd", "")).upper()
                param = str(cmd_data.get("param", ""))
                reason = cmd_data.get("reason", f"MANUAL_{source}")
            else:
                return False
        else:
            # Stringa di testo (LoRa o terminale)
            clean = str(data).strip()
            upper = clean.upper()

            if any(k in upper for k in ("ESTOP", "E-STOP", "E-CUT", "STOP_EMERGENCY")):
                kind = "ESTOP"
                reason = f"LORA_RADIO_ESTOP"
            elif upper in ("RESUME", "CMD:RESUME", "RESET", "CMD:RESET"):
                kind = "RESUME"
                reason = f"LORA_RADIO_RESUME"
            else:
                kind = "MOTION"
                raw_cmd = clean
                if raw_cmd.upper().startswith("CMD:"):
                    raw_cmd = raw_cmd[4:].strip()
                parts = [p.strip() for p in raw_cmd.split(",", 1)]
                verb = parts[0].upper()
                param = parts[1] if len(parts) > 1 else ""

                verb_map = {
                    "AVANTI": "FORWARD",
                    "INDIETRO": "BACKWARD",
                    "RETRO": "BACKWARD",
                    "SINISTRA": "TURN_L",
                    "GIRA_SX": "TURN_L",
                    "DESTRA": "TURN_R",
                    "GIRA_DX": "TURN_R",
                    "STOP": "STOP",
                    "FERMA": "STOP",
                    "STOP_TERRA": "STOP",
                    "PASSO": "WALK",
                    "TROTTO": "TROT",
                    "STAND": "STAND",
                    "SIT": "SIT"
                }
                cmd = verb_map.get(verb, verb)
                reason = f"LORA_GROUND_LINK"

        # --- C. Deduplicazione & Arbitraggio tra canali paralleli ---
        # Se la dashboard trasmette sia via LoRa che via WebSocket contemporaneamente (dual-mode)
        # deduplichiamo entro una finestra di 200ms per evitare doppio azionamento meccanico.
        with self._lock:
            if kind == "MOTION" and cmd == self.last_cmd and param == self.last_param:
                if (now - self.last_ts) < 0.20:
                    delta_ms = (now - self.last_ts) * 1000.0
                    print(f"ℹ️ [GATEWAY HUB] Comando {cmd}({param}) da {source} duplicato ({delta_ms:.0f}ms da {self.last_source}) - Filtrato")
                    return True

            self.last_kind = kind
            self.last_cmd = cmd
            self.last_param = param
            self.last_source = source
            self.last_ts = now
            self.total_commands += 1

        # --- D. Transizione Unificata al Bus MQTT locale ---
        if kind == "ESTOP":
            print(f"🚨 [GATEWAY HUB] E-STOP RICEVUTO DA {source}! Motivo: {reason}")
            mqtt.publish(
                "mar/safety/estop",
                json.dumps({"source": source, "reason": reason, "timestamp": now})
            )
            mqtt.publish(
                "mar/control/motion",
                json.dumps({
                    "cmd": "STOP",
                    "param": "",
                    "source": source,
                    "reason": reason,
                    "state": "EMERGENCY_STOP",
                    "timestamp": now
                })
            )
            return True

        elif kind == "RESUME":
            print(f"✅ [GATEWAY HUB] RESUME RICEVUTO DA {source}! Motivo: {reason}")
            mqtt.publish(
                "mar/safety/resume",
                json.dumps({"source": source, "reason": reason, "timestamp": now})
            )
            return True

        elif kind == "MOTION":
            print(f"🎯 [GATEWAY HUB] MOTION DA {source}: {cmd} (param='{param}') -> MQTT mar/control/motion")
            # Un movimento manuale intenzionale dell'operatore sblocca l'E-STOP latched
            mqtt.publish(
                "mar/safety/resume",
                json.dumps({"source": source, "reason": f"MOTION_AFTER_ESTOP_{source}", "timestamp": now})
            )
            mqtt.publish(
                "mar/control/motion",
                json.dumps({
                    "cmd": cmd,
                    "param": param,
                    "source": source,
                    "reason": reason,
                    "state": "MANUAL",
                    "timestamp": now
                })
            )
            return True

        return False


command_hub = GatewayCommandHub(lambda: mqtt_client)


def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        client.subscribe("mar/#")
        print(f"  ✓ MQTT connesso e sottoscritto a 'mar/#' (rc={rc})")
    else:
        print(f"  ✗ MQTT on_connect fallito rc={rc}")


def on_mqtt_message(client, userdata, msg):
    """Queue MQTT payloads for the asyncio broadcaster (thread-safe)."""
    global _mqtt_msg_count
    topic = msg.topic
    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except Exception:
        data = msg.payload.decode("utf-8", errors="replace")

    if lora_bridge is not None:
        try:
            lora_bridge.handle_message(topic, data)
        except Exception as e:
            print(f"[GATEWAY] LoRa handle error: {e}")

    if not _should_forward(topic):
        return

    data = _lighten_payload(topic, data)
    packet = {"topic": topic, "payload": data, "ts": time.time()}
    latest_by_topic[topic] = packet

    _mqtt_msg_count += 1
    if _mqtt_msg_count <= 3 or _mqtt_msg_count % 200 == 0:
        print(f"  📡 MQTT→WS #{_mqtt_msg_count} topic={topic} clients={len(connected_clients)}")

    try:
        outbound_q.put_nowait(packet)
    except queue.Full:
        try:
            outbound_q.get_nowait()
        except queue.Empty:
            pass
        try:
            outbound_q.put_nowait(packet)
        except queue.Full:
            pass


async def broadcast_to_clients(message: str):
    """Sends JSON string to all active WebSockets."""
    if not connected_clients:
        return
    dead_clients = set()
    for ws in list(connected_clients):
        try:
            await ws.send(message)
        except Exception:
            dead_clients.add(ws)
    for dead in dead_clients:
        connected_clients.discard(dead)


async def outbound_pump():
    """Drain the MQTT→WS queue on the asyncio event loop."""
    while True:
        try:
            while True:
                try:
                    packet = outbound_q.get_nowait()
                except queue.Empty:
                    break
                await broadcast_to_clients(json.dumps(packet, separators=(",", ":")))
        except Exception as e:
            print(f"[GATEWAY] outbound_pump error: {e}")
        await asyncio.sleep(0.02)


async def ws_handler(websocket):
    """Handles new client connections and incoming operator commands."""
    connected_clients.add(websocket)
    client_ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
    print(f"🔌 [GATEWAY] Nuovo client WebSocket connesso: {client_ip} (Totali: {len(connected_clients)})")

    # Push cached snapshot so the UI is live immediately
    try:
        await websocket.send(json.dumps({
            "topic": "mar/gateway/hello",
            "payload": {
                "ok": True,
                "clients": len(connected_clients),
                "cached_topics": list(latest_by_topic.keys()),
                "mqtt_messages": _mqtt_msg_count,
            },
            "ts": time.time(),
        }))
        priority = (
            "mar/fsm/state",
            "mar/lidar/alert",
            "mar/lidar/telemetry",
            "mar/vision/detections",
            "mar/vision/status",
            "mar/vision/command",
            "mar/imu/telemetry",
            "mar/hardware/telemetry",
            "mar/control/motion",
        )
        for topic in priority:
            cached = latest_by_topic.get(topic)
            if cached:
                await websocket.send(json.dumps(cached, separators=(",", ":")))
    except Exception as e:
        print(f"[GATEWAY] snapshot push failed: {e}")

    try:
        async for raw_msg in websocket:
            try:
                data = json.loads(raw_msg)
                msg_type = data.get("type", "")

                if msg_type in ("ESTOP", "RESUME", "MOTION_CMD"):
                    command_hub.dispatch_from_websocket(data)

                elif msg_type == "PING":
                    await websocket.send(json.dumps({
                        "topic": "mar/gateway/pong",
                        "payload": {"t": time.time(), "mqtt_messages": _mqtt_msg_count},
                        "ts": time.time(),
                    }))

            except Exception as e:
                print(f"[GATEWAY] Errore parsing messaggio client: {e}")

    except Exception:
        pass
    finally:
        connected_clients.discard(websocket)
        print(f"❌ [GATEWAY] Client disconnesso: {client_ip} (Rimanenti: {len(connected_clients)})")


async def main_async():
    global mqtt_client, lora_bridge

    print("=" * 60)
    print(f" 🐾 M.A.R. — WEBSOCKET TELEMETRY GATEWAY (Porta {WS_PORT})")
    print("=" * 60)

    if mqtt:
        # Callback API v2 when available; fall back to v1 signature
        try:
            mqtt_client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id="mar-gateway-server",
            )
        except Exception:
            mqtt_client = mqtt.Client(client_id="mar-gateway-server")

        mqtt_client.on_connect = on_mqtt_connect
        mqtt_client.on_message = on_mqtt_message
        connected = False
        for attempt in range(1, 16):
            try:
                mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
                mqtt_client.loop_start()
                print(f"  ✓ MQTT loop avviato verso {MQTT_BROKER}:{MQTT_PORT}")
                connected = True
                break
            except Exception as e:
                print(f"  ⚠️ MQTT tentativo {attempt}/15 fallito ({e}). Retry in 2s...")
                await asyncio.sleep(2)
        if not connected:
            print("  ⚠️ Connessione MQTT non riuscita dopo i retry.")

    if LORA_ENABLED:
        def on_lora_rx(line: str):
            command_hub.dispatch_from_lora(line)

        radio = LoRaRadio(
            backend=LORA_BACKEND,
            port=LORA_PORT,
            baudrate=LORA_BAUD,
            spi_bus=LORA_SPI_BUS,
            spi_device=LORA_SPI_DEV,
            cs_pin=LORA_CS_PIN,
            rst_pin=LORA_RST_PIN,
            freq_mhz=LORA_FREQ_MHZ,
            on_receive=on_lora_rx
        )
        radio.open()
        lora_bridge = LoRaBridge(radio, mqtt_client=mqtt_client, interval_s=LORA_TX_INTERVAL)
        lora_bridge.start()
    else:
        print("  ℹ️ [LORA] Link Long-Range disabilitato (LORA_ENABLED=false).")

    if websockets is None:
        print("✗ websockets non è installato. Esegui: pip install websockets")
        return

    asyncio.create_task(outbound_pump())

    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT, max_size=2_000_000):
        print(f"  🚀 Server WebSocket in ascolto su ws://0.0.0.0:{WS_PORT}")
        await asyncio.Future()


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[GATEWAY] Chiusura gateway...")
    finally:
        if lora_bridge is not None:
            lora_bridge.stop()
        if mqtt_client is not None:
            try:
                mqtt_client.loop_stop()
                mqtt_client.disconnect()
            except Exception:
                pass


if __name__ == "__main__":
    main()
