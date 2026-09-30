# Magic Wand 🪄

An embedded TinyML gesture recognition project running on the **Seeed Studio XIAO ESP32S3**, using an **MPU6050** 6-DOF IMU, a **WS2812B** addressable RGB LED, and machine learning models trained with **Edge Impulse**.

---

## Subprojects

- **[`data_collector`](data_collector/)**: ESP-IDF v6.1 firmware to capture stationary-calibrated, high-rate IMU telemetry over serial for the Edge Impulse Data Forwarder and CSV dataset creation.
