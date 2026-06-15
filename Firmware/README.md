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
