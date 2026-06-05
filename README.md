# ESP32-CAM FPV Firmware — Build & Flash Guide
## For OV3660 · AI-Thinker ESP32-CAM · FS-i6X RC Car

---

## Files in this Project

| File | Purpose |
|------|---------|
| `esp32cam_fpv.ino` | Main firmware — flash this to the ESP32-CAM |
| `viewer.html` | Standalone FPV viewer — open in any browser on your PC or phone |

---

## Hardware Requirements

- AI-Thinker ESP32-CAM (with OV3660 sensor)
- FTDI USB-UART adapter (3.3V — **not** 5V)
- 4× jumper wires

### FTDI → ESP32-CAM Wiring

| FTDI | ESP32-CAM |
|------|-----------|
| GND  | GND       |
| VCC  | 3.3V      |
| TX   | UOR (GPIO3) |
| RX   | UOT (GPIO1) |
| GND  | **IO0** (flash mode) |

> ⚠️ Connect **IO0 → GND only while flashing**. Remove after upload.

---

## Arduino IDE Setup

### 1. Install Arduino IDE 2.x
Download from https://www.arduino.cc/en/software

### 2. Add ESP32 Board Package
Preferences → Additional boards manager URLs, add:
```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```
Then: Tools → Board → Boards Manager → search "esp32" → Install **esp32 by Espressif Systems**

### 3. Select Board & Settings
- Board: **AI Thinker ESP32-CAM**
- Upload Speed: **115200** (reliable) or **460800** (faster)
- Partition Scheme: **Huge APP (3MB No OTA/1MB SPIFFS)**
- CPU Freq: **240MHz**
- Flash Freq: **80MHz**
- Flash Mode: **QIO**

### 4. Configure WiFi Credentials
Open `esp32cam_fpv.ino` and edit:
```cpp
#define WIFI_SSID   "YOUR_WIFI_SSID"
#define WIFI_PASS   "YOUR_WIFI_PASSWORD"
```

### 5. Flash
1. Wire IO0 → GND
2. Plug in FTDI
3. Click Upload in Arduino IDE
4. Wait for "Hard resetting via RTS pin..." message
5. **Remove IO0→GND wire**
6. Press RESET button on ESP32-CAM
7. Open Serial Monitor @ 115200 baud
8. Note the IP address printed

---

## Viewing the Stream

### Option A — Built-in Web UI (served by ESP32)
Open in browser: `http://<camera-ip>/`

### Option B — Standalone viewer.html (recommended)
Open `viewer.html` in Chrome/Edge on your PC, enter the camera IP, click ▶ Stream.
This viewer runs locally so there is zero server round-trip.

### Option C — Raw MJPEG (VLC / OpenCV / any player)
```
http://<camera-ip>:81/stream
```
VLC: Media → Open Network Stream → paste URL

### Option D — Single Snapshot
```
http://<camera-ip>/capture
```

---

## API Reference

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web UI |
| `/stream` | GET | MJPEG stream (port 81) |
| `/capture` | GET | Single JPEG snapshot |
| `/status` | GET | JSON camera status |
| `/control?var=X&val=Y` | GET | Adjust a parameter |

### Control Variables

| var | values | description |
|-----|--------|-------------|
| `framesize` | 5-9 | QVGA(5) CIF(6) HVGA(7) VGA(8) SVGA(9) |
| `quality` | 4-40 | JPEG quality — lower = better |
| `brightness` | -2 to 2 | Image brightness |
| `contrast` | -2 to 2 | Image contrast |
| `saturation` | -2 to 2 | Colour saturation |
| `vflip` | 0 or 1 | Vertical flip |
| `hmirror` | 0 or 1 | Horizontal mirror |
| `agc_gain` | 0-30 | Manual gain (0=auto) |
| `aec` | 0 or 1 | Auto exposure on/off |
| `ae_level` | -2 to 2 | AE target offset |
| `flash` | 0 or 1 | Onboard LED flash |

---

## Performance Tuning

### For Absolute Minimum Latency
```
framesize = QVGA (320×240)   → fastest encode
quality   = 12-15            → balance size vs quality
```

### For Best FPV Quality
```
framesize = VGA (640×480)    → crisp image
quality   = 8-12             → high quality JPEG
```

### Router Tips
- Use **2.4 GHz** band (better range, lower latency for video)
- Place router between car operating area and your viewing position
- Enable **QoS** on router and prioritize the ESP32 MAC address
- Disable router **Airtime Fairness** if available (penalises slow clients)

### Typical Performance (home WiFi)
| Resolution | Quality | Typical FPS | Typical Latency |
|------------|---------|-------------|-----------------|
| QVGA | 15 | 25-30 fps | 30-80 ms |
| CIF  | 12 | 20-25 fps | 40-100 ms |
| VGA  | 10 | 15-20 fps | 60-150 ms |
| SVGA | 8  | 8-12 fps  | 100-250 ms |

---

## Troubleshooting

**Camera init failed (brownout)**
→ Ensure FTDI supplies enough current; use a powered USB hub

**Grainy / dark image outdoors**
→ Lower `gainceiling` to 1 or 2; increase `ae_level` to +1

**Stream freezes then recovers**
→ Normal WiFi behaviour; firmware auto-reconnects

**Image upside-down**
→ Toggle vflip in the viewer or set `vflip=0` in firmware defaults

**Can't reach camera IP**
→ Check Serial Monitor for actual IP; check router DHCP table

---

## Notes on FS-i6X Integration
This firmware is purely for video — the FS-i6X handles RC control independently via its receiver (iBUS/PPM). No conflict exists. Mount the ESP32-CAM on the car chassis pointing forward, power it from a regulated 5V BEC or USB power bank on the car.
