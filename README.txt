# nRF52 BLE Sensorbase (XIAO Sense)

This project is a Bluetooth Beacon application based on the Zephyr controller, 
specifically optimized for the **Seeed Studio XIAO nRF52840 Sense**.

## Hardware
* **Board:** Seeed Studio XIAO nRF52840 Sense
* **SoC:** Nordic Semiconductor nRF52840

## Building
To build the project for the XIAO Sense board, use the following command:

```powershell
west build -p -b xiao_ble/nrf52840/sense