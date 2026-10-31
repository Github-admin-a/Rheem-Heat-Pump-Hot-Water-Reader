# ESP32 Hot Water Serial Monitor

An ESP32-C3 sniffer that listens to the 300-baud serial line inside a heat pump hot water system, decodes the operating state and temperatures, and publishes them to **Home Assistant** over MQTT with automatic discovery. It also has a built-in web dashboard for live viewing and setup.

It only listens: the ESP32 never transmits to the hot water unit.

---

## Features

- **Passive serial capture** of the hot water controller's 300-baud data stream (hardware UART, no SoftwareSerial)
- **Decodes:**
  - Byte 1: operating state and fault code
  - Byte 2: tank temperature (°C)
  - Byte 3: evaporator temperature (°C)
  - Byte 4: compressor temperature (°C)
- **Home Assistant MQTT Discovery:** entities appear automatically under one device
- **Availability / Last Will & Testament:** HA shows the device as unavailable if it drops off
- **Stale-reading protection:** sends nulls on reconnect so HA doesn't show yesterday's temperature as a spike, and drops the first (possibly torn) frame after boot
- **Live web dashboard** that refreshes every 1.5 s, showing raw hex, decoded values, Wi-Fi signal and MQTT status
- **Web-based setup** for Wi-Fi, MQTT broker and baud rate. Nothing is hard-coded.
- **WiFiManager captive portal** for first-time Wi-Fi setup
- **OTA updates** via the Arduino IDE
- **ESP32-C3 Super Mini fixes:** reduced TX power for the board's weak antenna; the onboard LED shows Wi-Fi state

---

## Hardware

| Part | Notes |
|---|---|
| ESP32-C3 Super Mini | Any ESP32 should work with pin changes |
| 1.8 kΩ resistor | Top of voltage divider |
| 3.3 kΩ resistor | Bottom of voltage divider |
| Hookup wire | To the hot water controller's signal, 5 V and GND |

### Wiring

The hot water signal is 5 V logic, so a resistor divider brings it down to about 3.2 V for the ESP32.

```
Hot Water Signal ──[ 1.8kΩ ]──┬── GPIO 20 (RX)
                              │
                           [ 3.3kΩ ]
                              │
Hot Water GND ────────────────┴── GND

Hot Water 5V ──────────────────── 5V
```

| ESP32-C3 Pin | Connects to |
|---|---|
| GPIO 20 (RX) | Signal (through divider) |
| GPIO 21 (TX) | **Not connected** |
| 5V | Hot water 5 V rail |
| GND | Hot water GND |
| GPIO 8 | Onboard LED (used as Wi-Fi status) |

> ⚠️ **Safety:** Hot water systems contain mains voltage. Isolate power before opening the unit, only connect to the low-voltage control side, and be aware that modifying your unit may affect its warranty. You do this at your own risk.

---

## Software Setup

### Arduino IDE settings

- **Board:** `ESP32C3 Dev Module` (ESP32 Arduino core 3.x)
- **USB CDC On Boot:** `Enabled` (needed for USB serial debug output)

### Libraries

| Library | Source |
|---|---|
| WiFiManager | tzapu (Library Manager) |
| PubSubClient | Nick O'Leary (Library Manager) |
| ArduinoJson | Benoit Blanchon, **v7.x** (Library Manager) |
| WebServer, LittleFS, ArduinoOTA | Included with the ESP32 core |

### Flashing

1. Open the `.ino` file in the Arduino IDE.
2. Install the libraries above.
3. Select the board and settings, then upload over USB.
4. Later updates can be uploaded over Wi-Fi: the device appears as the network port **`ESP32-HotWaterMonitor`**.

---

## First-Time Configuration

1. **Wi-Fi:** On first boot (or after a Wi-Fi reset) the board starts an access point called **`ESP32-HotWater-AP`**. Connect to it and the captive portal opens (or browse to `192.168.4.1`). Choose your network and enter the password. The portal times out after 3 minutes and the board restarts.
2. **Find the device:** Check your router or the USB serial monitor for its IP address.
3. **MQTT:** Browse to `http://<device-ip>/mqtt` and enter your broker host, port, username, password and topic prefix. Settings are saved to the board's flash (`/config.json` on LittleFS).
4. **Home Assistant:** With the MQTT integration installed, the device appears automatically as **Hot Water Serial Monitor**.

### LED status

| LED | Meaning |
|---|---|
| On | Not connected / setup AP running |
| Off | Connected to Wi-Fi |

---

## Web Interface

| Page | Purpose |
|---|---|
| `/` | Live status dashboard |
| `/wifi` | Scan and join a network, or forget Wi-Fi and restart in AP mode |
| `/serial` | Change baud rate (default 300) |
| `/mqtt` | MQTT broker settings |
| `/data.json` | Live data as JSON (used by the dashboard) |

---

## Home Assistant Entities

| Entity | Type | Source |
|---|---|---|
| Hot Water Telemetry Frame | Sensor | Raw hex frame |
| Hot Water Operating State | Sensor | Byte 1 (decoded text) |
| Hot Water Operating Code (Byte 1) | Sensor | Byte 1 (numeric) |
| Hot Water Fault Alert | Binary sensor (`problem`) | Byte 1 |
| Hot Water Tank Temp | Sensor (°C) | Byte 2 |
| Hot Water Evaporator Temp | Sensor (°C) | Byte 3 |
| Hot Water Compressor Temp | Sensor (°C) | Byte 4 |
| Hot Water Monitor IP Address | Diagnostic sensor | Device IP |

### MQTT topics

With the default prefix `homeassistant/sensor/hotwater`:

| Topic | Content |
|---|---|
| `homeassistant/sensor/hotwater/state` | JSON telemetry |
| `homeassistant/sensor/hotwater/availability` | `online` / `offline` (retained, LWT) |

Example state payload:

```json
{
  "raw_hex": "AA 17 37 0E 2C ...",
  "byte_1": 23,
  "status": "Heating",
  "fault": false,
  "byte_2": 55,
  "byte_3": 14,
  "byte_4": 44,
  "count": 128,
  "ip": "192.168.x.x"
}
```

Temperature fields are only included when the frame contains a valid reading. On reconnect they're sent as `null`, so HA shows `unknown` instead of a stale value.

---

## Status Codes (Byte 1)

These were worked out by watching the unit, so they may differ on other models.

| Code | Meaning |
|---|---|
| `0x00` | Off / Standby |
| `0x03`, `0x20` | Initialising / Booting |
| `0x0A` | Idle (up to temperature) |
| `0x16` | Startup / Transition |
| `0x17` | Heating |
| `0xE1`–`0xEF` | Fault (E1–E15) |
| `0xE0`, `0xEE`, `0xFF` | Fault detected |

Temperatures are ignored while the unit is off or booting (`0x00`, `0x03`, `0x20`), or when Byte 2 is zero.

---

## Notes & Limitations

- The protocol was reverse-engineered from one unit; byte meanings may not match every model. The raw hex sensor is there so you can check against your own unit.
- Frames are detected by **250 ms of line silence**, which suits 300 baud. If you change the baud rate a lot, you may need to adjust `PACKET_TIMEOUT_MS`.
- The web interface has **no authentication**, and the MQTT password is visible in the `/mqtt` page source. Keep the device on a trusted network or IoT VLAN.
- OTA has no password set. Add `ArduinoOTA.setPassword("...")` if you want one.

---

## Contributing

If you have a different model and can confirm or add status codes or byte meanings, issues and pull requests are welcome. Please include raw hex captures.

## License

Add your chosen license here (e.g. MIT).
