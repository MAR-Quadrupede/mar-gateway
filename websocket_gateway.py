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
from typing import Optional, Set, Dict, Any

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

# Heavy lidar scans freeze browsers over Tailscale — forward a light summary only
LIDAR_SCAN_MIN_INTERVAL_S = float(os.getenv("LIDAR_SCAN_MIN_INTERVAL_S", "0.5"))
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
    """Replace huge lidar point clouds with a dashboard-friendly summary."""
    if topic != "mar/lidar/scan" or not isinstance(data, dict):
        return data
    points = data.get("points") or []
    return {
        "timestamp": data.get("timestamp", time.time()),
        "point_count": len(points),
        "sector_mins": data.get("sector_mins", {}),
        "front_distance_m": data.get("front_distance_m"),
        # keep a coarse subset for optional radar widgets
        "points": points[:: max(1, len(points) // 60)][:60] if points else [],
    }


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

                if msg_type == "ESTOP":
                    print("🚨 [GATEWAY] Ricevuto E-STOP da WebSocket!")
                    if mqtt_client:
                        mqtt_client.publish(
                            "mar/safety/estop",
                            json.dumps({"reason": "DASHBOARD_BUTTON", "timestamp": time.time()}),
                        )

                elif msg_type == "RESUME":
                    print("✅ [GATEWAY] Resume da WebSocket")
                    if mqtt_client:
                        mqtt_client.publish(
                            "mar/safety/resume",
                            json.dumps({"reason": "DASHBOARD_RESUME", "timestamp": time.time()}),
                        )

                elif msg_type == "MOTION_CMD":
                    cmd_data = data.get("data", {})
                    if mqtt_client:
                        if "timestamp" not in cmd_data:
                            cmd_data = {**cmd_data, "timestamp": time.time()}
                        # Manual motion implies operator wants to leave latched E-STOP
                        mqtt_client.publish(
                            "mar/safety/resume",
                            json.dumps({"reason": "MOTION_AFTER_ESTOP", "timestamp": time.time()}),
                        )
                        mqtt_client.publish("mar/control/motion", json.dumps(cmd_data))

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
            if mqtt_client and mqtt_client.is_connected():
                mqtt_client.publish("mar/lora/rx", json.dumps({
                    "timestamp": time.time(),
                    "payload": line
                }))

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
