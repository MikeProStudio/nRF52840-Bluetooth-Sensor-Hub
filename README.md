# nRF52840 Bluetooth Sensor Hub

A professional-grade BLE (Bluetooth Low Energy) beacon and sensor monitoring application developed for the **Seeed Studio XIAO nRF52840 Sense**. Built on the **Zephyr RTOS**, this project demonstrates high-performance sensor data acquisition, real-time audio processing, and intelligent power management.

## 🚀 Key Features

- **Multi-Sensor Fusion:** Real-time data collection from the onboard LSM6DS3TR-C IMU (Accelerometer & Gyroscope).
- **Advanced Audio Processing:** PDM microphone integration with real-time RMS (Root Mean Square) calculation for sound level monitoring.
- **Intelligent Power Management:** 
  - Dynamic BLE TX Power adjustment based on battery SoC (State of Charge).
  - Battery voltage monitoring with a Simple Moving Average (SMA) filter.
  - Automatic charging status detection and charge current control.
- **Visual Feedback:** Context-aware RGB LED logic (Startup sequences, audio-reactive level metering).
- **Industrial-Grade BLE Service:** Custom 128-bit GATT service for synchronized data transmission.
- **Modern Web Interface:** Integrated WebUI concept for live data visualization via Web Bluetooth API.

## 🛠 Hardware Architecture

- **Platform:** Seeed Studio XIAO nRF52840 Sense
- **Microcontroller:** Nordic Semiconductor nRF52840 (ARM Cortex-M4F)
- **Sensors:**
  - **IMU:** LSM6DS3TR-C (6-Axis Accel/Gyro) via I2C.
  - **Microphone:** Pulse Density Modulation (PDM) Mic.
- **Connectivity:** Bluetooth 5.0 LE, USB CDC ACM (Serial Console).

## 📂 Project Structure

- `src/main.c`: Core application logic, thread management, and BLE service definitions.
- `app.overlay`: Hardware abstraction layer and pin mapping.
- `prj.conf`: Zephyr kernel and subsystem configuration.
- `web_ui.html`: Live monitoring dashboard.
- `docs/images/`:
  - `hardware/`: Photos of the physical setup.
  - `webui/`: Screenshots of the configuration interface.

## 📸 Visual Documentation

### Hardware Setup
![Hardware Setup](docs/images/hardware/setup.jpg)
*Place your XIAO Sense hardware photo here.*

### Web Interface
![Web UI Screenshot](docs/images/webui/screenshot.png)
*View live sensor graphs and battery status in the web dashboard.*

## 💻 Software Technical Details

### Threading Model
The application utilizes Zephyr's preemptive multithreading:
1. **Main Thread:** Handles system initialization and battery/power logic.
2. **Audio Thread:** High-priority PDM sampling and RMS calculation.
3. **Sensor Thread:** Synchronized IMU data fetching at 52Hz.

### BLE GATT Service (Custom)
- **Service UUID:** `12345678-1234-5678-1234-56789abcdef0`
- **Characteristics:**
  - `...f1`: Accelerometer Data (Notify)
  - `...f2`: Gyroscope Data (Notify)
  - `...f3`: Audio Level (Notify)
  - `...f4`: TX Power Level (Read/Notify)
  - `...f5`: Battery Status (Read/Notify)

## 🔨 Development & Build

### Prerequisites
- [nRF Connect SDK](https://www.nordicsemi.com/Products/Development-software/nrf-connect-sdk) (v2.x.x recommended)
- [Zephyr RTOS](https://www.zephyrproject.org/) environment.

### Build Instructions
```bash
# Using west (Zephyr's meta-tool)
west build -b xiao_ble_sense
```

## 📜 License
This project is provided "as is" for educational and development purposes.
