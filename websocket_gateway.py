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
from typing import Set

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None

try:
    import websockets
except ImportError:
    websockets = None

MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
WS_PORT = int(os.getenv("WS_PORT", "8765"))

connected_clients: Set = set()
loop: asyncio.AbstractEventLoop = None
mqtt_client = None


def on_mqtt_message(client, userdata, msg):
    """Broadcasts received MQTT payload to all connected WebSocket clients."""
    topic = msg.topic
    try:
        data = json.loads(msg.payload.decode("utf-8"))
    except Exception:
        data = msg.payload.decode("utf-8")

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
    global loop, mqtt_client
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


if __name__ == "__main__":
    main()
