These are the firmware files for all the board versions. The .bin files are used for the web flasher and OTA updates.

# Changelog:

### 1.2.1-Web (Latest)
- **Web Console Diagnostics:** Added a responsive, modern HTML Web Console (served on port 80 over WiFi) with live state monitoring, progress reporting, filament sensor status, internal temperature, and boot-reason crash diagnostics.
- **OTA Updates:** Upload new `.bin` firmware files directly via drag-and-drop in the Web Console.
- **Real-Time Logs:** Built-in ring buffer logging for active diagnostics (motor status, StallGuard triggers, WiFi connection, and filament runout events).
- **Generic ESP32-S3 and C3 Support:** Added generic board variants with automated single-wire UART configuration for TMC2209 (using open-drain and internal pull-ups).
- **Unified Build System:** Configured PlatformIO environment to build and package for:
  - `control_board` (LTS CB V4)
  - `driver_board` (LTS DB V1)
  - `esp32s3_generic` (ESP32-S3 DevKit / Super Mini)
  - `esp32c3_generic` (ESP32-C3 DevKit / Super Mini)
- **Pre-compiled Merged Binaries:** Added combined binaries (`_Merged.bin`) containing the bootloader, partition table, and application firmware for quick and easy flashing.

### 1.2.1
- Changed OTA-Update URL

### 1.2.0
- Added support for upcoming Respooler Pro with precise servo control
- App needs to be uptated to 1.7.0

### 1.1.2
- Added ability to exit Auto-Stop state via long button press
- Fixed issue where speed would decrease after restart

### 1.1.1
- Added support for Control Board V4
- Increased LED brightness

### 1.1.0
- Added option to choose what amount to transfer
- Lowered default fan speed

### 1.0.0
- Added option to pause the Respooler
- Unified current state logic
- Bug fixes

### 0.9.3
- Increased speed and motor strength
- Warning sound in case of Auto-Stop
- Fixed issue where motor strength would decrease after Auto-Stop

### 0.9.2
- Improved stability of Bluetooth connection
  
### 0.9.1
- Fixed bug where High-Speed setting would change after restart
- Bug fixes for Control Board V3
  
### 0.9.0
- First beta version

---

## 🌐 Web Console Integration & Technical Details

### Performance & Resource Usage
The Web Console is designed to be extremely lightweight and has **no noticeable performance impact** compared to the original firmware.
- **Hardware-level Stepper/Servo Control:** The motor steps and servo PWM signals are driven by the ESP32's hardware peripherals (LEDC channels and timers). This means the critical timing-sensitive parts of the respooler run entirely in hardware background threads, completely independent of any network activity.
- **Zero-RAM HTML Storage:** The responsive web page is stored in the ESP32's Flash memory (`PROGMEM`) as a raw string literal, using no precious dynamic RAM (heap) when idle.
- **Asynchronous Execution:** The Web Server handles client requests in a non-blocking way, running concurrently alongside the standard BLE connection, so you can use the smartphone app and the Web Console at the same time.

---

## 🛠️ Step-by-Step Guide: Building & Flashing the Firmware

If you want to customize the code or build the binaries yourself, follow this guide.

### Prerequisites
1. Install [VS Code](https://code.visualstudio.com/) and add the **PlatformIO IDE** extension.
2. Connect your ESP32 board to your computer using a USB-C/Micro-USB data cable.

### Step 1: Project Setup
All unified build files are located in the [Firmware/build/](build/) directory.
- Open the `Firmware/build` folder in VS Code. PlatformIO will automatically detect the configuration and download the required libraries (`NimBLE-Arduino`, `TMCStepper`, `ArduinoJson`, `Adafruit NeoPixel`).

### Step 2: Customizing and Regenerating the Code (Optional)
If you edit the raw source files (`Firmware/(.txt) Control Board Code` or `Firmware/(.txt) Driver Board Code`), you need to regenerate `main.cpp` so that it includes the web console modifications:
1. Run the Python generation script:
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
- **Command Line:** Run `pio run` inside the `build/` directory.

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
