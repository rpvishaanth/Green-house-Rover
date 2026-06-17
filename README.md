<!-- at the top of README.md, after the title -->
# Green-house-Rover
# Rover Firmware

<div align="center">

[![ESP32](https://img.shields.io/badge/ESP32-FreeRTOS-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](#)
[![UDP](https://img.shields.io/badge/Communication-UDP-2C7BE5?style=for-the-badge)](#)
[![PlatformIO](https://img.shields.io/badge/Build-PlatformIO-F5822A?style=for-the-badge&logo=platformio&logoColor=white)](#)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)](LICENSE)

</div>

## 📌 Project Overview

A mobile, WiFi-controlled greenhouse rover built on the **ESP32** microcontroller running **FreeRTOS**. The rover navigates greenhouse zones, collects real-time environmental data, detects sensor anomalies, and drives actuators — all managed by concurrent RTOS tasks.

**Key highlights:**
- Zone-wise environmental monitoring (temp, humidity, pressure, light)
- IR sensor-based line following for autonomous navigation
- UDP protocol for real-time wireless rover control
- FreeRTOS multi-tasking with mutex-protected shared data
- Statistical anomaly detection (rolling Z-score) for sensor fault isolation
- Dual cloud integration: **MQTT broker** + **Blynk v2** dashboard

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   ESP32 (FreeRTOS)                  │
│                                                     │
│  Core 1 (Sensing & Logic)   Core 0 (Motor & Comms) │
│  ┌──────────────────────┐   ┌─────────────────────┐ │
│  │ Task1: SensorRead    │   │ Task5: UDP Listener │ │
│  │ Task2: AnomalyDetect │   │ Task5: MotorFwd     │ │
│  │ Task3: ThreshCheck   │   │ Task5: MotorRev     │ │
│  │ Task4: Actuator      │   │ Task5: MotorLeft    │ │
│  │ Task6: MQTTPublish   │   │ Task5: MotorRight   │ │
│  │ Task7: BlynkSync     │   │ Task8: LineFollow   │ │
│  └──────────────────────┘   └─────────────────────┘ │
└─────────────────────────────────────────────────────┘
         │                            │
    WiFi (MQTT)                  WiFi (UDP :1234)
         │                            │
   ┌─────┴──────┐              ┌──────┴──────┐
   │ MQTT Broker│              │ Mobile App  │
   │ + Blynk v2 │              │ (UDP cmds)  │
   └────────────┘              └─────────────┘
```

---

## 🔩 Hardware Components

| Component | Purpose | Qty |
|---|---|---|
| ESP32 Dev Board | Main controller (WiFi + BT + dual-core) | 1 |
| DHT22 | Temperature & Humidity sensor | 1 |
| BMP280 | Barometric pressure sensor (I2C) | 1 |
| LDR Module | Ambient light sensing | 2 |
| IR Sensor Module | Line following / zone edge detection | 2 |
| L298N Motor Driver | DC motor direction & speed control | 1 |
| BO Motors + Wheels | 2WD drive | 2 |
| Relay Module (3-ch) | Fan, pump, grow-light switching | 1 |
| 9V Battery / 4xAA | Power supply | 1 |
| Breadboard + wires | Prototyping | — |

---

## 📌 Pin Map (ESP32)

| GPIO | Connected To |
|---|---|
| 4 | DHT22 Data |
| 21 / 22 | I2C SDA / SCL → BMP280 |
| 34 | LDR (analog) |
| 35 | IR Left sensor |
| 32 | IR Right sensor |
| 25 | Motor L – IN1 |
| 26 | Motor L – IN2 |
| 27 | Motor R – IN3 |
| 14 | Motor R – IN4 |
| 16 | Relay – FAN (active LOW) |
| 17 | Relay – PUMP (active LOW) |
| 18 | Relay – GROW LIGHT (active LOW) |

---

## ⚙️ FreeRTOS Task Scheduling

| Task | Function | Core | Priority | Scheduling |
|---|---|---|---|---|
| Task 1 – SensorRead | Read DHT22, BMP280, LDR, IR every 2 s | 1 | 3 (HIGH) | Fixed-Priority |
| Task 2 – AnomalyDetect | Rolling Z-score outlier detection | 1 | 3 (HIGH) | Fixed-Priority |
| Task 3 – ThresholdCheck | Compare readings vs zone thresholds | 1 | 3 (HIGH) | Fixed-Priority |
| Task 4 – Actuator | Drive fan / pump / light relays | 1 | 2 (MED) | **Round-Robin** |
| Task 5 – UDPListener | Parse F/B/L/R/S commands on port 1234 | 0 | 4 (HIGHEST) | Fixed-Priority |
| Task 5 – Motor sub-tasks | Execute movement via binary semaphore | 0 | 4 (HIGHEST) | Semaphore-gated |
| Task 6 – MQTTPublish | Push JSON telemetry every 5 s | 1 | 1 (LOW) | Fixed-Priority |
| Task 7 – BlynkSync | Write to Blynk virtual pins every 3 s | 1 | 1 (LOW) | Fixed-Priority |
| Task 8 – LineFollow | IR-based corridor following (mode 'A') | 0 | 4 (HIGHEST) | Fixed-Priority |

**Synchronisation primitives used:**
- `xMutexSensor` — protects the shared `SensorData` struct (all tasks)
- `xSemFwd / xSemRev / xSemStop / xSemLeft / xSemRight` — binary semaphores gate each motor direction task; ensures only one motor action runs at a time

---

## 🛰️ UDP Command Protocol

Send single-character ASCII commands to the rover's IP on **port 1234**:

| Command | Action |
|---|---|
| `F` | Move Forward |
| `B` | Move in Reverse |
| `L` | Turn Left |
| `R` | Turn Right |
| `S` | Stop |
| `A` | Enable autonomous line-following mode |

**Python sender snippet (PC / Raspberry Pi):**
```python
import socket
UDP_IP   = "192.168.x.x"   # rover IP shown on Serial Monitor
UDP_PORT = 1234
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b'F', (UDP_IP, UDP_PORT))  # send Forward
```

---

## 🔍 Anomaly Detection Algorithm

Each sensor value is checked in two ways:

1. **Hard-range sanity** — temperature outside –10 °C to 80 °C, or humidity outside 0–100% → immediate fault flag.
2. **Rolling Z-score** — a window of the last 10 samples is kept. If the current reading deviates more than **3σ** from the rolling mean, it is flagged as a statistical outlier (sensor spike / malfunction).

When an anomaly is flagged:
- All actuators are forced to **safe-off** state.
- An alert JSON `{"alert":"SENSOR_ANOMALY"}` is published to the MQTT status topic.
- Blynk virtual pin V7 is set to 1 (shown as LED widget in app).

---

## ☁️ MQTT Telemetry

**Broker:** `broker.hivemq.com:1883` (swap for local Mosquitto if needed)

| Topic | Direction | Payload |
|---|---|---|
| `greenhouse/rover/sensors` | Publish (every 5 s) | JSON (see below) |
| `greenhouse/rover/status` | Publish (on anomaly) | `{"alert":"SENSOR_ANOMALY"}` |
| `greenhouse/rover/cmd` | Subscribe | Single char F/B/L/R/S/A |

**Example publish payload:**
```json
{
  "temp": 28.50,
  "humidity": 65.20,
  "pressure": 1008.30,
  "light": 1520,
  "fan": 0,
  "pump": 0,
  "light_relay": 1,
  "anomaly": 0,
  "ts": 45230
}
```

You can visualise live data at `http://www.hivemq.com/demos/websocket-client/` — subscribe to `greenhouse/rover/sensors`.

---

## 📱 Blynk v2 Virtual Pin Map

| Virtual Pin | Widget | Data |
|---|---|---|
| V0 | Gauge | Temperature (°C) |
| V1 | Gauge | Humidity (%) |
| V2 | Gauge | Pressure (hPa) |
| V3 | Level | Light intensity |
| V4 | LED | Fan status |
| V5 | LED | Pump status |
| V6 | LED | Grow-light status |
| V7 | LED (red) | Anomaly alert |
| V10 | Button | Motor Forward |
| V11 | Button | Motor Reverse |
| V12 | Switch | Line-follow mode |
| V13 | Switch | Manual fan override |

---

## 🚀 Getting Started

### 1. Install Arduino IDE Libraries

Go to **Sketch → Include Library → Manage Libraries** and install:

```
DHT sensor library       by Adafruit
Adafruit BMP280 library  by Adafruit
Adafruit Unified Sensor  by Adafruit
PubSubClient             by Nick O'Leary
Blynk                    by Volodymyr Shymanskyy  (v1.3+)
```

Board: **ESP32 Dev Module** via Boards Manager → `esp32 by Espressif Systems`

### 2. Edit Config in `greenhouse_rover.ino`

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* MQTT_BROKER   = "broker.hivemq.com";  // or your local IP

#define BLYNK_TEMPLATE_ID   "YOUR_TEMPLATE_ID"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_TOKEN"
```

### 3. Flash to ESP32

- Select board: `ESP32 Dev Module`
- Upload Speed: `921600`
- Open Serial Monitor at `115200 baud`

### 4. Verify startup output

```
=== Greenhouse Rover Booting ===
[WiFi] Connected! IP: 192.168.1.42
[Task1] SensorRead started
[Task2] AnomalyDetect started
...
[Setup] All tasks created. Rover ready.
```

### 5. Send a UDP command

```bash
echo -n "F" | nc -u 192.168.1.42 1234
```

---

## 📁 Repository Structure

```
greenhouse_rover/
├── src/
│   └── greenhouse_rover.ino     # Main Arduino/FreeRTOS source
├── docs/
│   ├── flowchart_sensor.png     # Fig 3.2.2 — sensor logic flowchart
│   └── flowchart_udp.png        # Fig 5.2.1 — UDP command flowchart
├── README.md
```

---

## 🔮 Future Scope

- Autonomous navigation with ultrasonic / LiDAR + SLAM
- Machine learning anomaly detection replacing Z-score
- Solar-powered charging for outdoor/remote deployment
- Multi-rover mesh coordination using ESP-NOW
- Persistent cloud data logging for compliance reports

---

## 📚 References

1. M. Sushmitha et al., *"Implementation of Greenhouse Control and Monitoring System Using ESP32"*, IEEE CONECCT 2022.
2. P. D. Rosero-Montalvo et al., *"Smart Farming Robot for Detecting Environmental Conditions in a Greenhouse"*, IEEE Access, 2023.
3. X. Geng et al., *"A Mobile Greenhouse Environment Monitoring System Based on IoT"*, IEEE Access, 2019.
4. V. P. Chandanshiv et al., *"Greenhouse Environment Controlling Robot"*, IRJMETS, 2021.
5. F. Cañadas-Aránega et al., *"Autonomous Collaborative Mobile Robot for Greenhouses"*, Smart Agricultural Technology, 2024.

