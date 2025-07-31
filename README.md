
<div align="center">

  <h1>True Wireless Stereo (TWS) Earbuds with ESP32</h1>
  <p><b>Open-Source Stereo Bluetooth Earbuds System</b></p>
  <p>
    <img src="https://img.shields.io/badge/platform-ESP32-blue"/>
    <img src="https://img.shields.io/badge/audio-Bluetooth%20A2DP-green"/>
    <img src="https://img.shields.io/badge/license-MIT-lightgrey"/>
  </p>
</div>

---

## Table of Contents
- [Motivation & Goals](#motivation--goals)
- [Project Overview](#project-overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Technical Deep Dive](#technical-deep-dive)
- [Hardware Requirements](#hardware-requirements)
- [Software Stack](#software-stack)
- [Setup & Installation](#setup--installation)
- [Usage Guide](#usage-guide)
- [Troubleshooting](#troubleshooting)
- [FAQ](#faq)
- [Limitations & Challenges](#limitations--challenges)
- [Future Improvements](#future-improvements)
- [Contributing](#contributing)
- [License](#license)
- [References](#references)

---

## Motivation & Goals
Commercial TWS earbuds are closed-source, expensive, and difficult to customize. This project aims to:
- Demystify wireless audio synchronization and Bluetooth protocols.
- Provide a hands-on, extensible platform for engineers, students, and hobbyists.
- Enable open-source innovation in consumer audio electronics.

---

## Project Overview
This project implements a custom-built True Wireless Stereo (TWS) Earbuds system using two ESP32 microcontrollers. It replicates the experience of commercial wireless earbuds, enabling synchronized stereo audio playback, dynamic master/slave switching, and seamless Bluetooth connectivity. Each ESP32 board acts as an independent earbud (left/right), capable of operating solo or in tandem for a true stereo experience.

---

## Features
- **Dynamic Master/Slave Switching:** Automatic role assignment and seamless failover for uninterrupted audio.
- **Bluetooth A2DP Audio Streaming:** High-quality stereo audio from any Bluetooth source (phone, laptop, etc.).
- **I2S Audio Output:** Low-latency, high-fidelity digital audio via external DACs.
- **Inter-Device Synchronization:** Peer-to-peer sync using ESP-NOW, UART, or Wi-Fi Direct.
- **Persistent Pairing:** Stores Bluetooth and peer pairing info in non-volatile memory.
- **Standalone or Stereo Use:** Each earbud can operate independently or together.
- **User Controls:** Optional button and LED for status, reset, and mode switching.
- **Expandable Hardware:** Support for battery, charging modules, and additional sensors.
- **Open for Expansion:** Add gesture/touch controls, sensors, smart charging case, OTA updates, BLE features, and more.

---

## System Architecture
<details>
<summary>Visual Diagram</summary>

```
+-------------------+      +-------------------+
|   ESP32 (Left)    |<---->|   ESP32 (Right)   |
|  (Master/Slave)   |      |  (Master/Slave)   |
+-------------------+      +-------------------+
        |                          |
   [I2S DAC]                 [I2S DAC]
        |                          |
   [Speaker/HP]              [Speaker/HP]
```
</details>

- **Master/Slave Negotiation:** First powered-on ESP32 becomes master, connects to Bluetooth source, and syncs with the second ESP32.
- **Audio Routing:** Left/Right channels handled by respective boards for true stereo.
- **Synchronization:** Audio data and control signals exchanged for minimal latency.

---

## Technical Deep Dive
### Bluetooth A2DP Sink
Uses ESP32's Bluetooth stack to receive stereo audio from any A2DP-compatible source. The master ESP32 connects and streams audio, while the slave synchronizes for stereo playback.

### I2S Audio Output
Digital audio is routed via I2S protocol to external DACs (PCM5102A, MAX98357A), ensuring high-fidelity, low-latency output to speakers or headphones.

### Inter-Device Communication
Synchronization between left and right earbuds is achieved using ESP-NOW, UART, or Wi-Fi Direct. This ensures minimal latency and robust stereo separation.

### FreeRTOS Task Management
ESP32's FreeRTOS environment manages concurrent Bluetooth, audio, and synchronization tasks for reliable operation.

### Persistent Storage
Pairing and configuration data are stored in NVS or Arduino Preferences, allowing automatic reconnection after power cycles.

---

## Hardware Requirements
- **2× ESP32 Development Boards** (ESP32-WROOM-32 or equivalent)
- **2× I2S-Compatible Audio DACs** (e.g., PCM5102A, MAX98357A)
- **Speakers or Headphone Jacks**
- **Li-ion Batteries** (for portable use)
- **TP4056 Charging Modules** (optional)
- **LEDs & Buttons** (for status and control)
- **Optional:** Reed switches, smart charging case components

---

## Software Stack
- **Frameworks:** Arduino Core for ESP32 or ESP-IDF
- **Bluetooth:** `BluetoothA2DPSink` library for A2DP audio streaming
- **Audio:** `I2S.h` for digital audio output
- **Synchronization:** `WiFi`, `ESP-NOW`, or `Serial` libraries
- **Storage:** NVS (Non-Volatile Storage) or Arduino Preferences
- **RTOS:** FreeRTOS for multitasking and concurrency

---

## Setup & Installation
### 1. Hardware Assembly
- Connect each ESP32 to its respective I2S DAC and audio output.
- (Optional) Integrate buttons, LEDs, battery, and charging modules.

### 2. Software Preparation
- Install [Arduino IDE](https://www.arduino.cc/en/software) or [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).
- Add ESP32 board support via Board Manager.
- Install required libraries:
  - `BluetoothA2DPSink`
  - `I2S.h`
  - `Preferences` (if using Arduino)

### 3. Flashing Firmware
- Upload the firmware to both ESP32 boards.
- Power on one board (it becomes master), then the second (slave).

### 4. Pairing & Testing
- Pair the master ESP32 with your Bluetooth audio source.
- Confirm stereo playback and synchronization.
- Test master/slave failover by powering off/on each board.

---

## Usage Guide
- **Power On:** First ESP32 becomes master, connects to Bluetooth source.
- **Stereo Mode:** Power on second ESP32; it syncs as slave and plays the other channel.
- **Single Bud Mode:** Use either ESP32 independently for mono playback.
- **Controls:** Use button for reset/mode switch; LED indicates status.
- **Reconnection:** If master is powered off, slave auto-promotes and reconnects.

---

## Troubleshooting
- **No Audio Output:** Check I2S wiring, DAC connections, and Bluetooth pairing status.
- **Desync Between Buds:** Ensure both ESP32s are using the same firmware and synchronization protocol.
- **Bluetooth Pairing Issues:** Reset pairing info via button or NVS erase.
- **LED Not Working:** Verify GPIO assignments and hardware connections.

---

## FAQ
**Q: Can I use only one earbud?**  
A: Yes, each ESP32 can operate independently for mono playback.

**Q: Is microphone support available?**  
A: Not natively. ESP32 does not support Bluetooth voice profiles (HFP/HSP). See Future Improvements for possible solutions.

**Q: Can I update firmware over-the-air (OTA)?**  
A: OTA is possible with ESP32, but not implemented in the base project. See ESP-IDF documentation for details.

**Q: What is the typical latency?**  
A: Latency is minimal, but depends on synchronization method and hardware.

---

## Limitations & Challenges
- **Microphone Support:** ESP32 does not natively support Bluetooth voice profiles (HFP/HSP), so microphone functionality is not available.
- **Audio Latency:** Minimal, but may vary based on synchronization method.
- **Bluetooth Stack:** Limited by ESP32’s current Bluetooth stack capabilities.
- **Battery Life:** Depends on chosen hardware and power management.

---

## Future Improvements
- Integrate external Bluetooth audio codec chips (e.g., CSR8675) for microphone support.
- Explore ESP32 Audio Development Framework (ESP-ADF) for advanced features.
- Add gesture/touch controls and in-ear detection sensors.
- Develop a smart charging case with automatic power management.
- Implement OTA (Over-The-Air) updates and BLE features.
- Enhance power management for commercial-grade portability.

---

## Contributing
Contributions, bug reports, and feature requests are welcome! Please open an issue or submit a pull request.

---

## License
This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

---

## References
- [ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [BluetoothA2DPSink Library](https://github.com/pschatzmann/ESP32-A2DP)
- [ESP-NOW](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html)
- [I2S Audio Protocol](https://en.wikipedia.org/wiki/I%C2%B2S)
- [Arduino Preferences Library](https://www.arduino.cc/reference/en/libraries/preferences/)
- [ESP32 FreeRTOS](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html)

---

<div align="center">
  <sub>Made with ❤️ using ESP32 • Wireless Audio • Open Source</sub>
</div>
