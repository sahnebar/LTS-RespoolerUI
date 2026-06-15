# ⚙️🧵 Welcome to the LTS Respooler GitHub! 🧵⚙️

The LTS Respooler is a compact and slightly over-engineered filament Respooler optimized for Bambu Lab spools. It integrates a filament runout sensor and wireless app-connectivity for ease of use. The LTS Respooler Pro uses a 9g servo motor to precisely position the Filament Guide. You can find all the files related to it here. That includes the raw .txt ESP32 Code, the .bin firmware files and the PCB Gerber files.

### 🌐 Web Console & Multi-Board Diagnostics (v1.2.1-Web)
This version introduces a **built-in Web Interface** served directly by the ESP32 on port 80. Key features:
- **Live Diagnostics Console:** View real-time log output from the device (motor starts/stops, StallGuard stall detections, filament runouts, and WiFi connection updates).
- **Control Interface:** Start, pause, and stop respooling directly from any web browser.
- **Interactive OTA Update:** Drag and drop `.bin` firmware files to upgrade the device wirelessly.
- **Crash Diagnostics:** Shows device boot/reset reasons (e.g. power-on reset, watchdog resets, or brownouts due to inadequate power supplies).
- **Generic Board Support:** Complete configuration for compiling on generic **ESP32-S3** and **ESP32-C3** dev boards (e.g. Super Mini form factor) with automated single-wire UART routing for the TMC2209 driver.

All build and configuration files can be found in the [Firmware/build](file:///root/LTS-Respooler/Firmware/build) directory. You can compile the project using PlatformIO.

## ➡️ Upcoming features and files:

- Official attachment for spools bigger than 1 kg

## 🚀 Getting started

- All the 3D files can be downloaded and printed from [MakerWorld](https://makerworld.com/@lukas.tu)
- Assembly Intructions and the Needed Hardware PDF can also be found there
- Most of the Hardware can be ordered from Bambu Lab's Maker's Supply
- PCBs can be ordered from my [Website](https://lts-design.com) or on [PCBWay](https://www.pcbway.com/project/member/?bmbno=7102E4BB-3522-48)
- Flash the newest firmware version to your ESP32 or LTS Control Board using the [Web Flasher](https://flash.lts-design.com)

## 📋 License

This project is licensed under the MIT license, giving you the freedom to use it however you like. Contributions, modifications, and improvements are very welcome! :)
