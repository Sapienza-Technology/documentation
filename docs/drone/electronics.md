# Electronics


## System Overview

| Component          | Model                    | Interface              | Connected to            |
|--------------------|--------------------------|------------------------|-------------------------|
| Flight Controller  | Pixhawk 6C               | —                      | Central hub             |
| ESC                | SpeedyBee BLS 55A 4in1   | Bidirectional DShot600 | Pixhawk PWM OUT         |
| Companion Computer | Raspberry Pi 5           | USB 2.0                | Pixhawk + RealSense     |
| Camera / IMU       | Intel RealSense D435i    | USB 2.0                | Raspberry Pi            |
| Telemetry Bridge   | ESP8266 (MavESP8266)     | UART (TELEM1)          | Pixhawk                 |
| RC Receiver        | ELRS receiver            | SBUS                   | Pixhawk RC port         |
| Power Module       | Holybro PM02 v3          | JST                    | Pixhawk + Battery       |
| UBEC               | 5V 8A UBEC               | Soldered to ESC pads   | Raspberry Pi            |
| GPS + Compass      | —                        | GPS + I2C              | Pixhawk                 |
| Range Sensor       | VL53L1X                  | I2C                    | Pixhawk                 |
| Buzzer             | —                        | GPIO                   | Pixhawk                 |
| LEDs               | —                        | GPIO                   | Pixhawk                 |

---

## Pixhawk 6C — Port mapping

| Port    | Connected to        | Protocol / Notes                      |
|---------|---------------------|---------------------------------------|
| TELEM1  | ESP8266             | MAVLink — telemetry bridge for QGC    |
| TELEM2  | Raspberry Pi (UART) | uXRCE-DDS @ 921600 baud               |
| PPM/SBUS RC   | ELRS receiver       | SBUS                                  |
| PWM OUT | SpeedyBee BLS 55A   | Bidirectional DShot600                |
| GPS     | GPS module          | GPS + compass (I2C)                   |
| I2C     | VL53L1X             | Range sensor — configured and working |

---

## Power distribution
``` mermaid
graph TD
    A[Battery 6S LiPo] --> B[PM02 v3\nvoltage / current sensing]
    B --> C[Pixhawk 6C\npower + telemetry]
    B --> D[SpeedyBee BLS 55A ESC]
    D --> E[Motors M1–M4\nDShot600]
    D --> F[UBEC 5V 8A\nsoldered on ESC VBAT pads]
    F --> G[Raspberry Pi 5\nvia USB-C]
```

---

## ESC — SpeedyBee BLS 55A 4in1

- **Protocol:** Bidirectional DShot600
- **Motors:** M1–M4 connected to ESC outputs
- **Signal:** from Pixhawk PWM OUT port
- **UBEC:** soldered directly on ESC VBAT pads — no Y cable

!!! danger "ESC overheating"
    The ESC is known to overheat. Do not power the drone for extended periods on the ground without active cooling or flying.
    Allow cooling time between runs.

??? note "Motor numbering"
    With props out configuration, wrong motor order = crash.

---

## RealSense D435i

- Connected to Raspberry Pi via **USB 2.0**
- Used for: RGB image + IMU data → OpenVINS

!!! warning "USB 2.0 bandwidth limitation"
    The RealSense D435i should ideally be connected via USB 3.0 for full performance.
    Currently using USB 2.0 — depth stream is disabled to reduce bandwidth usage.
    Upgrade to USB 3.0 when available.

---

## ESP8266 — Telemetry Bridge

- **Firmware:** MavESP8266 (MAVLink bridge)
- **Connection:** TELEM1 on Pixhawk, UART 7
- **WiFi SSID:** `TechDrone_Telem` — password: `TechDrone26`
- **Web interface:** `192.168.4.1` (connect to `TechDrone_Telem` first)

---

## VL53L1X — Range Sensor

- **Interface:** I2C
- **Status:** configured and working in PX4
- Used for: low-altitude distance measurement

---

## TODO

- [ ] Add full wiring diagram
- [ ] Find correct Pixhawk PWM OUT port for DShot600 (confirm FMU AUX vs MAIN OUT)
- [ ] Shield all cables — currently unshielded, potential source of EMI interference
- [ ] Add delayed Pixhawk power-on circuit (separate button to power Pixhawk after RPi boots — would fix MicroXRCE timing issue)
- [ ] Replace ELRS SBUS with telemetry-based RC link — requires a **custom PX4 firmware build** to enable the telemetry RC input, which is not supported out of the box. Non-trivial effort — coordinate with the software team before attempting.

---

*Maintainer: Alessandro Paolantonio — for questions or issues, contact Alessandro.*