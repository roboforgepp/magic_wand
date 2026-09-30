# Magic Wand Data Collector (ESP-IDF v6.1)

Firmware for the **Seeed Studio XIAO ESP32S3** that collects 6-DOF inertial data from an **MPU6050** sensor, provides visual feedback using a **WS2812B 5mm RGB LED**, and streams CSV telemetry to the **Edge Impulse Data Forwarder** (or direct CSV recording).

---

## 1. Hardware Connections

| Peripheral | Sensor Pin | XIAO ESP32S3 Pin | ESP32-S3 GPIO | Description |
| :--- | :--- | :--- | :--- | :--- |
| **MPU6050** | `VCC` | `3V3` | 3.3V | Power rail |
| **MPU6050** | `GND` | `GND` | GND | Common ground |
| **MPU6050** | `SDA` | `D4` | `GPIO 5` | I2C Data line (internal pull-up enabled) |
| **MPU6050** | `SCL` | `D5` | `GPIO 6` | I2C Clock line (internal pull-up enabled) |
| **MPU6050** | `AD0` | `GND` | GND | Sets I2C address to `0x68` |
| **WS2812B** | `VDD` | `5V` | 5V | Recommended 5V for color fidelity |
| **WS2812B** | `GND` | `GND` | GND | Common ground |
| **WS2812B** | `DIN` | `D3` | `GPIO 4` | RMT-driven addressable LED signal |

---

## 2. Architecture & FreeRTOS Concurrency

- **Core 1 (`sensor_task`, Priority 10)**: Strict periodic acquisition at 50 Hz using `vTaskDelayUntil()`. Reads all 14 sensor bytes in a single atomic I2C burst transaction.
- **Core 0 (`telemetry_task`, Priority 4)**: Consumes from a 32-element FreeRTOS queue and formats serial output, ensuring USB/serial latency never induces sampling jitter.
- **Core 0 (`led_task`, Priority 2)**: Non-blocking LED animation engine driven by system state.

### LED Visual Indicators

| State | LED Color & Pattern | Meaning |
| :--- | :--- | :--- |
| **Init** | Off | System booting |
| **Calibrating** | Yellow Pulsing (2 Hz) | Keep wand stationary to zero out gyro drift |
| **Ready** | Solid Green | Calibration complete |
| **Streaming** | Cyan / Blue Breathing | Active telemetry streaming to serial |
| **Error** | Flashing Red (5 Hz) | I2C communication fault or queue overrun |

---

## 3. Telemetry Stream Format
 
By default, the serial output streams raw 6-axis CSV rows without timestamps, designed specifically for **Option A (Edge Impulse Data Forwarder)**:
 
```text
accX,accY,accZ,gyrX,gyrY,gyrZ
```
 
- **`accX, accY, accZ`**: Acceleration in $\text{m/s}^2$ ($\pm 8g$ full-scale range).
- **`gyrX, gyrY, gyrZ`**: Angular velocity in $\text{deg/s}$ ($\pm 2000^\circ/\text{s}$ full-scale range) with zero-rate bias subtracted.
 
> **Note**: For **Option B (Direct CSV file upload)**, Edge Impulse requires a timestamp column. You can enable timestamps (`timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ`) via `idf.py menuconfig` under **Magic Wand Configuration $\rightarrow$ Telemetry Configuration $\rightarrow$ Include timestamp (ms) as first column**.
 
---
 
## 4. Build & Flash
 
Ensure your ESP-IDF environment is activated:
 
```bash
# Set target to ESP32-S3 (only needed on first build)
idf.py set-target esp32s3
 
# Configure settings (optional: change pins, sample rate, timestamp mode, etc.)
idf.py menuconfig
 
# Build project
idf.py build
 
# Flash and monitor
idf.py -p /dev/ttyACM0 flash monitor
```
 
---
 
## 5. Connecting to Edge Impulse
 
### Option A: Live Streaming with Edge Impulse Data Forwarder (Default)
 
When using the `edge-impulse-data-forwarder`, **no timestamp column is needed** because the forwarder constructs the timing internally based on the `--frequency` parameter.
 
1. Install the [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/edge-impulse-cli/cli-installation):
   ```bash
   npm install -g edge-impulse-cli
   ```
2. Start the forwarder:
   ```bash
   edge-impulse-data-forwarder --frequency 50
   ```
3. When prompted, log into your Edge Impulse account and select your project.
4. Name the 6 sensor axes:
   ```text
   accX,accY,accZ,gyrX,gyrY,gyrZ
   ```
5. Navigate to the **Data Acquisition** tab in Edge Impulse Studio to view live data streams and record gestures!
 
### Option B: Direct CSV Recording & Upload
 
When uploading pre-recorded CSV files directly through the **Upload Data** tab in Edge Impulse Studio, Edge Impulse requires an explicit millisecond timestamp column.
 
1. Enable timestamps in the firmware:
   Run `idf.py menuconfig` $\rightarrow$ **Magic Wand Configuration** $\rightarrow$ **Telemetry Configuration** $\rightarrow$ enable **Include timestamp (ms) as first column** (or set `CONFIG_WAND_TELEMETRY_INCLUDE_TIMESTAMP=y` in `sdkconfig.defaults`).
2. Re-flash the device. The serial output will stream:
   ```text
   timestamp,accX,accY,accZ,gyrX,gyrY,gyrZ
   ```
3. Capture the stream directly to a `.csv` file:
   ```bash
   python3 -m serial.tools.miniterm /dev/ttyACM0 115200 > spell_lumos_01.csv
   ```
4. Upload the generated CSV file directly via the **Upload Data** tab in Edge Impulse Studio.
