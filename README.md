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
├── requirements.txt           # paho-mqtt, websockets
├── mosquitto.conf             # Configurazione del broker Mosquitto locale
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
