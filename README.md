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

## 🛠️ Step-by-Step Guide: Building & Flashing the Firmware

If you want to customize the code or build the binaries yourself, follow this guide.

### Prerequisites
1. Install [VS Code](https://code.visualstudio.com/) and add the **PlatformIO IDE** extension.
2. Connect your ESP32 board to your computer using a USB-C/Micro-USB data cable.

### Step 1: Project Setup
All unified build files are located in the [Firmware/build/](Firmware/build/) directory.
- Open the `Firmware/build` folder in VS Code. PlatformIO will automatically detect the configuration and download the required libraries (`NimBLE-Arduino`, `TMCStepper`, `ArduinoJson`, `Adafruit NeoPixel`).

### Step 2: Customizing and Regenerating the Code (Optional)
If you edit the raw source files (`Firmware/(.txt) Control Board Code` or `Firmware/(.txt) Driver Board Code`), you need to regenerate `main.cpp` so that it includes the web console modifications:
1. Run the Python generation script inside `Firmware/build/`:
   ```bash
   python3 modify_firmware.py
   ```
2. This will apply the HTML dashboard, logging hooks, and single-wire configs, and output a fresh `src/main.cpp`.

### Step 3: Compiling the Firmware
In PlatformIO, you can choose from four pre-configured environments depending on your hardware:
- `control_board` (LTS Control Board V4)
- `driver_board` (LTS Driver Board V1 / Standard ESP32)
- `esp32s3_generic` (Generic ESP32-S3 boards like Super Mini)
- `esp32c3_generic` (Generic ESP32-C3 boards like Super Mini)

To compile:
- **VS Code GUI:** Click the PlatformIO icon on the sidebar, select your environment, and click **Build**.
- **Command Line:** Run `pio run` inside the `Firmware/build/` directory.

### Step 4: Flashing the Firmware to the ESP32

There are three ways to flash the firmware onto your board:

#### Method A: Direct Flashing via USB (PlatformIO)
1. Plug your ESP32 board into your computer.
2. In VS Code PlatformIO, select your target environment and click **Upload** (or run `pio run -t upload`).
3. PlatformIO will compile, connect to the board, and flash the bootloader, partitions, and app.

#### Method B: Using a Web Flasher (Recommended for raw boards)
1. Open the [LTS Web Flasher](https://flash.lts-design.com/) in a Chrome or Edge browser.
2. Connect your ESP32 via USB and click **Connect** in the web flasher.
3. Select the appropriate **Merged** binary (e.g. `LTS_Respooler_ControlBoard_Web_Merged.bin` or `LTS_Respooler_S3_Generic_Web_Merged.bin`) from the `Firmware` folder. The merged binary already includes the bootloader and partition table mapped to the correct offsets.
4. Click **Program** to flash.

#### Method C: Over-the-Air (OTA) via Web Console
*Note: This only works if the Web-enabled firmware is already running on the device.*
1. Open the Web Console of your Respooler in a browser (enter its IP address).
2. Scroll to the **Firmware Update (OTA)** card.
3. Drag and drop the standard `.bin` OTA file (e.g., `LTS_Respooler_ControlBoard_Web.bin` — *not* the merged one) onto the card, or click to upload.
4. The device will upload, verify, and restart automatically with the new firmware.

---

## 📋 License

This project is licensed under the MIT license, giving you the freedom to use it however you like. Contributions, modifications, and improvements are very welcome! :)
