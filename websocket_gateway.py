#!/usr/bin/env python3
"""
M.A.R. — WebSocket & Telemetry API Gateway
Bridges internal MQTT topics to web browsers and operator dashboards via WebSocket.
"""

import os
import sys
import json
import asyncio
import signal
from typing import Optional, Set

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

connected_clients: Set = set()
loop: asyncio.AbstractEventLoop = None
mqtt_client = None
lora_bridge: Optional[LoRaBridge] = None


def on_mqtt_message(client, userdata, msg):
    """Broadcasts received MQTT payload to all connected WebSocket clients."""
    topic = msg.topic
    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except Exception:
        data = msg.payload.decode("utf-8")

    # Feed the long-range radio link with the aggregated robot state
    if lora_bridge is not None:
        lora_bridge.handle_message(topic, data)

    packet = json.dumps({
        "topic": topic,
        "payload": data
    })

    if connected_clients and loop and loop.is_running():
        # Dispatch to asyncio loop
        asyncio.run_coroutine_threadsafe(broadcast_to_clients(packet), loop)


async def broadcast_to_clients(message: str):
    """Sends JSON string to all active WebSockets."""
    dead_clients = set()
    for ws in list(connected_clients):
        try:
            await ws.send(message)
        except Exception:
            dead_clients.add(ws)

    for dead in dead_clients:
        connected_clients.discard(dead)


async def ws_handler(websocket):
    """Handles new client connections and incoming operator commands."""
    connected_clients.add(websocket)
    client_ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
    print(f"🔌 [GATEWAY] Nuovo client WebSocket connesso: {client_ip} (Totali: {len(connected_clients)})")

    try:
        async for raw_msg in websocket:
            try:
                data = json.loads(raw_msg)
                msg_type = data.get("type", "")

                # E-Stop da pulsante rosso dashboard
                if msg_type == "ESTOP":
                    print("🚨 [GATEWAY] Ricevuto E-STOP da WebSocket!")
                    if mqtt_client:
                        mqtt_client.publish("mar/safety/estop", json.dumps({"reason": "DASHBOARD_BUTTON"}))

                # Comando manuale joystick o andatura
                elif msg_type == "MOTION_CMD":
                    cmd_data = data.get("data", {})
                    if mqtt_client:
                        mqtt_client.publish("mar/control/motion", json.dumps(cmd_data))

            except Exception as e:
                print(f"[GATEWAY] Errore parsing messaggio client: {e}")

    except Exception:
        pass
    finally:
        connected_clients.discard(websocket)
        print(f"❌ [GATEWAY] Client disconnesso: {client_ip} (Rimanenti: {len(connected_clients)})")


async def main_async():
    global loop, mqtt_client, lora_bridge
    loop = asyncio.get_running_loop()

    print("=" * 60)
    print(f" 🐾 M.A.R. — WEBSOCKET TELEMETRY GATEWAY (Porta {WS_PORT})")
    print("=" * 60)

    if mqtt:
        try:
            mqtt_client = mqtt.Client(client_id="mar-gateway-server")
            mqtt_client.on_message = on_mqtt_message
            mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            mqtt_client.subscribe("mar/#")
            mqtt_client.loop_start()
            print(f"  ✓ Sottoscritto a MQTT Broker ({MQTT_BROKER}:{MQTT_PORT}) su 'mar/#'")
        except Exception as e:
            print(f"  ⚠️ Connessione MQTT fallita ({e}).")

    # Long-range LoRa telemetry link (RFM95W SPI or Waveshare SX1262 USB)
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

    async with websockets.serve(ws_handler, "0.0.0.0", WS_PORT):
        print(f"  🚀 Server WebSocket in ascolto su ws://0.0.0.0:{WS_PORT}")
        await asyncio.Future()  # run forever


def main():
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[GATEWAY] Chiusura gateway...")
    finally:
        if lora_bridge is not None:
            lora_bridge.stop()


if __name__ == "__main__":
    main()
