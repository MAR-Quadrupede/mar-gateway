# Microservizio: `mar-gateway`

**Inter-Service Message Broker & WebSocket Telemetry Gateway**  
*Parte dell'architettura a microservizi M.A.R. (Multi-legged Autonomous Robot)*

---

## Ruolo nell'Architettura a Microservizi

`mar-gateway` costituisce il **centro nevralgico della comunicazione** di M.A.R.:
1. **Local Message Broker (MQTT Mosquitto)**:
   - Funziona da dorsale ad altissima efficienza e minimo impatto di memoria/CPU su Raspberry Pi 5.
   - Disaccoppia completamente i microservizi (`mar-lidar-service`, `mar-vision-service`, `mar-decision-engine`, `mar-serial-bridge`).
2. **WebSocket Telemetry Gateway**:
   - Mantiene connessioni WebSocket aperte con i browser connessi alla `mar-dashboard`.
   - Aggrega e trasmette in streaming real-time a 10 Hz la point cloud radar LiDAR, lo stato della FSM e la telemetria di batteria e motori.
3. **Dispatch Comandi Operatore**:
   - Riceve i messaggi inviati dalla dashboard (E-Stop, joystick virtuale, cambio andatura) e li pubblica con priorità sui topic MQTT corrispondenti.
4. **Link Telemetrico Long-Range LoRa (868 MHz)**:
   - Aggrega assetto (roll/pitch), distanza frontale LiDAR, batteria e stato FSM in un frame ASCII compatto e lo trasmette alla stazione di terra una volta al secondo.
   - Supporta due backend derivati dai test legacy: **RFM95W/SX1276 su SPI** (`spi`) e **Waveshare SX1262 su USB** (`usb`, trasparente a 9600 baud), più una modalità `simulated` per lo sviluppo senza radio.
   - Pubblica lo stato del link su `mar/lora/status` e inoltra eventuali frame ricevuti su `mar/lora/rx`.

---

## Interfacce e Topic MQTT

| Topic | Direzione | Formato Payload | Descrizione |
| :--- | :--- | :--- | :--- |
| `mar/#` | In ingresso (Sub) | JSON dei microservizi | Tutta la telemetria interna (inoltrata anche via WebSocket) |
| `mar/lora/status` | In uscita (Pub) | `{"backend": "spi", "tx_count": 42, "last_frame": "[IMU] ..."}` | Stato del link radio long-range |
| `mar/lora/rx` | In uscita (Pub) | `{"timestamp": ..., "payload": "CMD:STOP"}` | Frame ricevuti dalla stazione di terra |

---

## Porte di Rete e Protocolli

| Porta | Protocollo | Destinazione / Utilizzo |
| :--- | :--- | :--- |
| **1883** | TCP (MQTT) | Bus messaggi interno per tutti i microservizi su RPi 5 |
| **8765** | WebSocket (WS) | Canale dati bidirezionale real-time per la `mar-dashboard` |
| **9001** | WebSocket (MQTT-over-WS) | Opzionale per client MQTT nativi web |

---

## Struttura del Repository

```text
mar-gateway/
├── README.md                  # Questo documento
├── requirements.txt           # paho-mqtt, websockets, pyserial
├── requirements-raspberrypi.txt # RPi.GPIO + spidev (solo backend LoRa SPI)
├── mosquitto.conf             # Configurazione del broker Mosquitto locale
├── lora_telemetry.py          # Bridge LoRa (RFM95W SPI / Waveshare SX1262 USB) per la stazione di terra
└── websocket_gateway.py       # Gateway asincrono MQTT ↔ WebSocket
```

---

## Avvio del Microservizio

### 1. Avvio Broker Mosquitto
```bash
mosquitto -c mosquitto.conf
```

### 2. Avvio Gateway WebSocket
```bash
pip install -r requirements.txt
python websocket_gateway.py
```

### 3. Attivazione del link LoRa
```bash
# Su Raspberry Pi (RFM95W su SPI, 868 MHz)
pip install -r requirements-raspberrypi.txt
LORA_ENABLED=true LORA_BACKEND=spi LORA_FREQ_MHZ=868.0 python websocket_gateway.py

# Con modulo Waveshare SX1262 su USB (trasparente)
LORA_ENABLED=true LORA_BACKEND=usb LORA_PORT=/dev/ttyUSB0 LORA_BAUD=9600 python websocket_gateway.py

# Test senza radio (frame stampati a console)
LORA_ENABLED=true LORA_BACKEND=simulated python websocket_gateway.py
```
