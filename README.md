# Beacon 1 Project

This project is a beacon application based on the Zephyr RTOS. It includes a web interface for configuration and monitoring.

## Project Structure

- `src/`: Application source code (C)
- `docs/images/`: Documentation and images
  - `hardware/`: Photos of hardware components
  - `webui/`: Screenshots of the web interface
- `prj.conf`: Project configuration
- `app.overlay`: DeviceTree overlays

## Images

### Hardware
![Hardware Setup](docs/images/hardware/setup.jpg)
*(Placeholder for hardware photo)*

### Web Interface
![Web UI Screenshot](docs/images/webui/screenshot.png)
*(Placeholder for WebUI screenshot)*

## Development

### Prerequisites
- Zephyr SDK
- nRF Connect SDK (if using Nordic hardware)

### Build
Use the `build.bat` or the standard Zephyr workflow:
```bash
west build -b <your_board>
```

## Web UI
The web interface is defined in `web_ui.html`. A concept can be found in `web_ui_concept.md`.
