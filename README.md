# 🏎️ MX-5 Vent Display & Racetracker

A stealthy, high-performance gauge and lap timer designed to fit perfectly into the air vent of your Mazda MX-5. Powered by direct CAN-Bus integration and a highly accurate 20Hz GPS module.

---

## ✨ Features
* **Perfect Fit:** Designed to seamlessly replace the OEM air vent in the MX-5 (NB/NBFL).
* **Live Engine Data:** Reads direct CAN-Bus data (Temperatures, Pressures, RPM, etc.) with zero lag.
* **Precision Racetracker:** Built-in 20Hz GPS for extremely accurate lap timing and track telemetry.

## 🛠️ Hardware & Installation
This repository contains the firmware source code. If you purchased the hardware kit, installation is plug-and-play. 
* Provide 12V Power & Ground
* Connect CAN-High and CAN-Low
* Route the GPS antenna

## ⚖️ License & Commercial Use
This project is open for **personal, non-commercial use** and educational purposes. You are welcome to read the code, modify it for your own MX-5, and contribute improvements!
However, manufacturing, selling, or distributing this hardware/software commercially is strictly prohibited without explicit permission. See the `LICENSE` file for details.

## 💻 How to compile and flash (Example Code)

This repository provides the example firmware for the MX-5 Vent Display. There are no pre-compiled binaries available; you will need to compile the code yourself. The project is built using **PlatformIO**.

**Prerequisites:**
1. Download and install [Visual Studio Code (VS Code)](https://code.visualstudio.com/).
2. Install the **PlatformIO IDE** extension within VS Code.

**Steps to flash:**
1. Clone or download this repository as a ZIP file and extract it.
2. Open the extracted folder in VS Code (`File -> Open Folder...`).
3. PlatformIO will automatically read the `platformio.ini` file and download all necessary libraries (CAN, GPS, Display drivers, etc.). This might take a minute.
4. Connect the MX-5 Display to your PC via USB.
5. Click the **"Upload"** button (the right-pointing arrow `→`) in the PlatformIO bottom taskbar.

## 🔌 Wiring & Pinout

The hardware kit connects to your MX-5 using an 8-pin harness. Here is the pinout configuration:

| Pin | Wire Color | Function | Description |
|:---:|:---|:---|:---|
| **1** | 🔴 Red | `VCC IN` | 4-28V DC Power (Switched 12V Ignition recommended) |
| **2** | 🟤 Brown | `USB D+` | USB Data Plus (for firmware updates) |
| **3** | 🟢 Green | `USB D-` | USB Data Minus (for firmware updates) |
| **4** | 🟡 Yellow | `Dimmer` | Ambient Dimmer (Connect to car's illumination line) |
| **5** | ⚪ Grey | `Sensor` | External Sensor Input |
| **6** | 🟣 Pink | `CAN L` | CAN-Bus Low |
| **7** | 🔵 Blue | `CAN H` | CAN-Bus High |
| **8** | ⚪ White | `GND` | Ground (Chassis GND) |
