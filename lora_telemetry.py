#!/usr/bin/env python3
"""
M.A.R. — LoRa Telemetry Bridge (868 MHz long-range ground link)
Porting of the legacy `canelidarstereoimuloradef2.py` (RFM95W on SPI) and
`lorausb.py` (Waveshare SX1262 USB transparent bridge) into the telemetry
gateway, which is the architectural owner of the link towards the ground station.

The bridge aggregates the low-rate robot state from MQTT (attitude, front LiDAR
distance, battery, FSM state) and transmits a compact ASCII frame every
`LORA_TX_INTERVAL` seconds. Received bytes (when the radio supports RX) are
republished on `mar/lora/rx` so future ground-station commands can be handled
by the rest of the system.

Backends:
  * ``spi``       -> RFM95W / SX1276 on SPI0 + GPIO (Raspberry Pi)
  * ``usb``       -> Waveshare SX1262 USB module (transparent serial, 9600 baud)
  * ``simulated`` -> console only (no hardware)
  * ``auto``      -> spi if /dev/spidev0.0 exists, else usb if port exists, else simulated
"""

from typing import Callable, Dict, Optional
import json
import os
import threading
import time

try:
    import serial
except ImportError:
    serial = None


# RFM95W / SX1276 registers
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FRF_MSB = 0x06
REG_FRF_MID = 0x07
REG_FRF_LSB = 0x08
REG_IRQ_FLAGS = 0x12
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_PAYLOAD_LENGTH = 0x22

MODE_SLEEP_LORA = 0x80
MODE_STANDBY = 0x81
MODE_TX = 0x83

IRQ_TX_DONE = 0x08


class LoRaRadio:
    """Small hardware abstraction for the two supported radio modules."""

    def __init__(
        self,
        backend: str = "auto",
        port: str = "/dev/ttyUSB0",
        baudrate: int = 9600,
        spi_bus: int = 0,
        spi_device: int = 0,
        cs_pin: int = 5,
        rst_pin: int = 25,
        freq_mhz: float = 868.0,
        on_receive: Optional[Callable[[str], None]] = None
    ):
        self.requested_backend = (backend or "auto").lower()
        self.backend = self.requested_backend
        self.port = port
        self.baudrate = int(baudrate)
        self.spi_bus = int(spi_bus)
        self.spi_device = int(spi_device)
        self.cs_pin = int(cs_pin)
        self.rst_pin = int(rst_pin)
        self.freq_mhz = float(freq_mhz)
        self.on_receive = on_receive

        self.is_open = False
        self.last_error = ""
        self._serial = None
        self._spi = None
        self._gpio = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False

    # ------------------------------------------------------------------ open
    def _resolve_backend(self) -> str:
        if self.requested_backend != "auto":
            return self.requested_backend
        if os.path.exists("/dev/spidev0.0"):
            return "spi"

        arduino_port = os.environ.get("SERIAL_PORT", "/dev/ttyUSB0")
        if os.path.exists(self.port) and self.port != arduino_port:
            return "usb"

        for candidate in ("/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyACM0", "/dev/ttyACM1"):
            if os.path.exists(candidate) and candidate != arduino_port:
                self.port = candidate
                return "usb"

        return "simulated"

    def open(self) -> bool:
        self.backend = self._resolve_backend()
        try:
            if self.backend == "spi":
                return self._open_spi()
            if self.backend == "usb":
                return self._open_usb()
            if self.backend == "simulated":
                print("  🧪 [LORA] Backend SIMULATO (nessuna radio hardware rilevata).")
                self.is_open = True
                return True
            print(f"  ⚠️ [LORA] Backend sconosciuto '{self.backend}': uso simulazione.")
            self.backend = "simulated"
            self.is_open = True
            return True
        except Exception as e:
            self.last_error = str(e)
            print(f"  ⚠️ [LORA] Inizializzazione fallita ({e}).")
            # Only an explicit "auto" request is allowed to degrade to simulation:
            # a configured radio that fails must stay visible as an error.
            if self.requested_backend == "auto":
                print("  🧪 [LORA] Fallback su backend SIMULATO.")
                self.backend = "simulated"
                self.is_open = True
            return False

    def _open_usb(self) -> bool:
        if serial is None:
            self.last_error = "pySerial non installato"
            return False
        self._serial = serial.Serial(self.port, self.baudrate, timeout=0.5)
        time.sleep(0.5)
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass
        self.is_open = True
        self._running = True
        self._rx_thread = threading.Thread(target=self._usb_rx_loop, name="lora-rx", daemon=True)
        self._rx_thread.start()
        print(f"  ✓ [LORA] Waveshare SX1262 connessa su {self.port} ({self.baudrate} baud)")
        return True

    def _open_spi(self) -> bool:
        import spidev
        import RPi.GPIO as GPIO

        self._gpio = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.rst_pin, GPIO.OUT)
        GPIO.setup(self.cs_pin, GPIO.OUT)
        GPIO.output(self.cs_pin, GPIO.HIGH)

        # Hardware reset of the RFM95W
        GPIO.output(self.rst_pin, GPIO.LOW)
        time.sleep(0.01)
        GPIO.output(self.rst_pin, GPIO.HIGH)
        time.sleep(0.05)

        self._spi = spidev.SpiDev()
        self._spi.open(self.spi_bus, self.spi_device)
        self._spi.max_speed_hz = 5_000_000
        self._spi.mode = 0

        # 1. Sleep + LoRa mode
        self._write_reg(REG_OP_MODE, MODE_SLEEP_LORA)
        time.sleep(0.01)

        # 2. Carrier frequency (Frf = freq * 2^19 / 32 MHz)
        frf = int(round(self.freq_mhz * 1_000_000 * (1 << 19) / 32_000_000.0))
        self._write_reg(REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._write_reg(REG_FRF_MID, (frf >> 8) & 0xFF)
        self._write_reg(REG_FRF_LSB, frf & 0xFF)

        # 3. Modem: BW 125 kHz, CR 4/5, SF7, CRC on
        self._write_reg(REG_MODEM_CONFIG_1, 0x72)
        self._write_reg(REG_MODEM_CONFIG_2, 0x70)

        # 4. Standby
        self._write_reg(REG_OP_MODE, MODE_STANDBY)
        self.is_open = True
        self._running = True
        print(f"  ✓ [LORA] RFM95W configurata su SPI{self.spi_bus}.{self.spi_device} "
              f"@ {self.freq_mhz:.1f} MHz (CS={self.cs_pin}, RST={self.rst_pin})")
        return True

    # ------------------------------------------------------------------- SPI
    def _cs_low(self):
        if self._gpio:
            self._gpio.output(self.cs_pin, self._gpio.LOW)

    def _cs_high(self):
        if self._gpio:
            self._gpio.output(self.cs_pin, self._gpio.HIGH)

    def _write_reg(self, reg: int, value: int):
        self._cs_low()
        self._spi.xfer2([reg | 0x80, value & 0xFF])
        self._cs_high()

    def _read_reg(self, reg: int) -> int:
        self._cs_low()
        result = self._spi.xfer2([reg & 0x7F, 0x00])
        self._cs_high()
        return result[1]

    def _write_fifo(self, payload: bytes):
        self._cs_low()
        self._spi.xfer2([REG_FIFO | 0x80] + list(payload))
        self._cs_high()

    # ------------------------------------------------------------------- send
    def send(self, text: str) -> bool:
        """Sends one text frame (newline-terminated by the caller/bridge)."""
        if not self.is_open:
            return False
        try:
            if self.backend == "usb":
                self._serial.write(text.encode("utf-8", errors="ignore"))
                self._serial.flush()
                return True
            if self.backend == "spi":
                return self._send_spi(text.encode("utf-8", errors="ignore"))
            print(f"  📡 [LORA-SIM] {text.strip()}")
            return True
        except Exception as e:
            self.last_error = str(e)
            print(f"  ⚠️ [LORA] Errore trasmissione: {e}")
            return False

    def _send_spi(self, payload: bytes) -> bool:
        payload = payload[:255]
        self._write_reg(REG_OP_MODE, MODE_STANDBY)
        self._write_reg(REG_FIFO, 0x00)  # FIFO pointer
        self._write_fifo(payload)
        self._write_reg(REG_PAYLOAD_LENGTH, len(payload))
        self._write_reg(REG_IRQ_FLAGS, 0xFF)  # clear flags
        self._write_reg(REG_OP_MODE, MODE_TX)

        # Bounded wait for TxDone before the next frame can reuse the FIFO
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if self._read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE:
                break
            time.sleep(0.005)

        self._write_reg(REG_IRQ_FLAGS, 0xFF)
        self._write_reg(REG_OP_MODE, MODE_STANDBY)
        return True

    # -------------------------------------------------------------------- RX
    def _usb_rx_loop(self):
        while self._running and self._serial and self._serial.is_open:
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if line and self.on_receive:
                    self.on_receive(line)
            except Exception:
                time.sleep(0.2)

    # ------------------------------------------------------------------ close
    def close(self):
        self._running = False
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        if self._spi is not None:
            try:
                self._spi.close()
            except Exception:
                pass
            self._spi = None
        if self._gpio is not None:
            try:
                self._gpio.cleanup()
            except Exception:
                pass
            self._gpio = None
        self.is_open = False


class LoRaBridge:
    """Aggregates MQTT telemetry and streams a compact frame over LoRa."""

    def __init__(
        self,
        radio: LoRaRadio,
        mqtt_client=None,
        interval_s: float = 1.0
    ):
        self.radio = radio
        self.mqtt_client = mqtt_client
        self.interval_s = max(0.2, float(interval_s))
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.tx_count = 0
        self.tx_errors = 0
        self._last_open_attempt = 0.0
        self.reconnect_interval_s = 5.0

        self._roll: Optional[float] = None
        self._pitch: Optional[float] = None
        self._front_cm: Optional[float] = None
        self._ultrasonic_cm: Optional[float] = None
        self._battery_v: Optional[float] = None
        self._fsm_state: str = ""
        self._last_source_ts = 0.0

    # ------------------------------------------------------------- MQTT input
    def handle_message(self, topic: str, data):
        """Called from the gateway MQTT callback for every `mar/#` message."""
        if not isinstance(data, dict):
            return

        with self._lock:
            if topic == "mar/imu/telemetry":
                try:
                    self._roll = float(data.get("roll"))
                    self._pitch = float(data.get("pitch"))
                except (TypeError, ValueError):
                    pass

            elif topic in ("mar/lidar/alert", "mar/lidar/telemetry"):
                value = data.get("front_distance_m")
                try:
                    if value is not None and float(value) < 90.0:
                        self._front_cm = float(value) * 100.0
                except (TypeError, ValueError):
                    pass

            elif topic in ("mar/ultrasonic/range", "mar/ultrasonic/telemetry"):
                raw_us = data.get("us_cm") or data.get("ultrasonic_cm") or data.get("distance_cm") or data.get("us")
                try:
                    if raw_us is not None:
                        self._ultrasonic_cm = float(raw_us)
                except (TypeError, ValueError):
                    pass

            elif topic == "mar/fsm/state":
                state = str(data.get("state", "")).strip()
                reason = str(data.get("reason", "")).strip()
                self._fsm_state = f"{state} {reason}".strip() if reason else state

            elif topic == "mar/hardware/telemetry":
                raw = data.get("batt") or data.get("batt_v")
                try:
                    if raw is not None:
                        self._battery_v = float(raw)
                except (TypeError, ValueError):
                    pass

            else:
                return

            self._last_source_ts = time.time()

    # -------------------------------------------------------------- formatting
    def format_frame(self) -> str:
        with self._lock:
            roll, pitch = self._roll, self._pitch
            front_cm = self._front_cm
            us_cm = self._ultrasonic_cm
            battery = self._battery_v
            state = self._fsm_state or "IDLE"

        if roll is not None and pitch is not None:
            imu_part = f"R:{roll:5.1f}° P:{pitch:5.1f}°"
        else:
            imu_part = "R:  --  P:  -- "

        lidar_part = f"Fr:{front_cm:3.0f}cm" if front_cm is not None else "Fr: -- "
        us_part = f" | [US] {us_cm:3.0f}cm" if us_cm is not None else ""
        batt_part = f" | [BATT] {battery:.1f}V" if battery is not None else ""

        # Same frame layout as legacy with US addition, terminated with \n so that
        # the ground-station serial reader is unblocked (Windows/Linux/Web Serial).
        return f"[IMU] {imu_part} | [LiDAR] {lidar_part}{us_part}{batt_part} || {state}\n"

    # ------------------------------------------------------------------- loop
    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="lora-tx", daemon=True)
        self._thread.start()
        print(f"  🚀 [LORA] Telemetria Long-Range attiva (backend={self.radio.backend}, "
              f"intervallo={self.interval_s:.1f}s)")

    def _loop(self):
        while self._running:
            # Attempt reconnection if the radio dropped / was not present at boot
            if not self.radio.is_open:
                now = time.time()
                if now - self._last_open_attempt >= self.reconnect_interval_s:
                    self._last_open_attempt = now
                    self.radio.open()
                time.sleep(min(1.0, self.interval_s))
                continue

            frame = self.format_frame()
            ok = self.radio.send(frame)
            if ok:
                self.tx_count += 1
            else:
                self.tx_errors += 1

            if self.mqtt_client and self.mqtt_client.is_connected():
                self.mqtt_client.publish("mar/lora/status", json.dumps({
                    "timestamp": time.time(),
                    "backend": self.radio.backend,
                    "radio_ready": self.radio.is_open,
                    "tx_count": self.tx_count,
                    "tx_errors": self.tx_errors,
                    "last_frame": frame.strip()
                }))

            time.sleep(self.interval_s)

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.radio.close()
