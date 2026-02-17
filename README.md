# nRF52840(-Sense) Bluetooth Sensor Hub
 
<img src="https://github.com/user-attachments/assets/511be654-f444-4ab2-8e12-2eb7210b6b76" width="600">


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
<img src="https://github.com/user-attachments/assets/68a228c7-4f74-4885-9130-ee3c02ecdf7c" alt="Hardware Setup" width="200">


### Web Interface
<br>
1. Pair the Bluetooth Device with your Browser Online by clicking "Initiate Tactical Uplink"
<br>
<br>

<img src="https://github.com/user-attachments/assets/b3d4b0f9-e5e1-4ee7-8a7e-59152ee31883" width="600">

<br>
<br>
2. When connected you can view the Data online in your Browser:
<br>
<br>

<img src="https://github.com/user-attachments/assets/84708197-ca8a-4d86-9adb-92feb39d3839" width="600">

<br>
<br>
3. When the Battery-Level drops below a certain Level the Tx-Level will switch to lower Levels (instead of Highpower of +8 dBm)
<br>
<br>

<img src="https://github.com/user-attachments/assets/12450a12-f951-4af6-bf7e-758c23e12ec8" width="600">



## 💻 Software Technical Details

### Threading Model
The application utilizes Zephyr's preemptive multithreading:
1. **Main Thread:** Handles system initialization and battery/power logic.
2. **Audio Thread:** High-priority PDM sampling and RMS calculation.
Fun Gimmic: The RGB LED reacts on the Loudness directly on the Board from Quiet to Loud: Blue->Green->Red
<img src="https://github.com/user-attachments/assets/f94b9513-0fe7-49ea-8a32-3d8c6edf9d87" width="200">


3. **Sensor Thread:** Synchronized IMU data fetching at 52Hz.

### BLE GATT Service (Custom)
- **Service UUID:** `12345678-1234-5678-1234-56789abcdef0`
- **Characteristics:**
  - `...56789abcdef1`: Accelerometer Data (Notify)
  - `...56789abcdef2`: Gyroscope Data (Notify)
  - `...56789abcdef3`: Audio Level (Notify)
  - `...56789abcdef4`: TX Power Level (Read/Notify)
  - `...56789abcdef5`: Battery Status (Read/Notify)

## 🔨 Development & Build

### Prerequisites
- [nRF Connect SDK](https://www.nordicsemi.com/Products/Development-software/nrf-connect-sdk) (v3.2.2 recommended)
- [nRF Connect for VS Code Extension Pack](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-VS-Code)
- [Zephyr RTOS](https://www.zephyrproject.org/) environment.

### Build Instructions
```bash
# Using west (Zephyr's meta-tool)
west build -b xiao_ble_sense
```
If you want to flash via Windows Explorer: Take the zephyr.uf2 from the zephyr folder and double-tap the RST Button on the nRF52840 Board for USB-Massstorage Option. Copy & Past and the nRF will restart into Application.
RGB LED Sequenz at Bootup will visualize when the Application is ready.

## 📜 License
This project is provided "as is" for educational and development purposes.
