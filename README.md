# 📡 Antenna Tracker - Community Edition (CRSF & MAVLink)
A high-performance, open-source Antenna Tracker designed specifically for modern Long-Range FPV. 

No coding required! This tracker features a standalone Web UI for configuration and natively sniffs your telemetry directly out of the air. It grabs the GPS coordinates your flight controller is already broadcasting and points your high-gain patch antennas dead-center at your aircraft.

### ⚠️ Critical Prerequisite: Your Transmitter 
Your ExpressLRS transmitter **MUST have a hardware "Backpack" chip installed**. The backpack is a secondary ESP32 or ESP8285 chip inside your radio/module dedicated to communicating with ground station gear.
* **Supported:** Most modern external modules (Radiomaster Ranger, Happymodel ES24TX, BetaFPV Micro) and internal modules (Radiomaster Boxer, TX16S MKII, GX12).
* **Unsupported:** Older or ultra-budget internal modules. Please verify your radio's specifications before building!

---

## ✨ Key Features
* **100% Wireless Data Link:** Reads native CRSF telemetry packets over ESP-NOW, or MAVLink data via Wi-Fi UDP. No extra hardware required on the drone/plane.
* **Standalone Web UI:** Configure your network, servo limits, and hardware toggles directly from your phone. No Arduino IDE or C++ editing required.
* **The "Gatekeeper" Safety:** Refuses to calibrate until both the ground station and the aircraft have a rock-solid 8+ satellite 3D lock.
* **Auto-Expiring NVRAM Failsafe:** Survives mid-flight power losses by instantly restoring your calibration math.
* **Live RF Trim (Optional):** Supports a physical potentiometer knob to micro-adjust your pan tracking mid-flight for maximum video clarity.

---

## 🛒 Hardware Shopping List

> **⚠️ MICROCONTROLLER REQUIREMENT:**
> You MUST use a standard **ESP32 WROOM-32 Dev Board** (either the 30-pin or 38-pin version). 
> * **DO NOT** buy an ESP32-S2, ESP32-S3, ESP32-C3, or ESP8266. 
> * The pre-compiled `.bin` firmware and the 3D-printed case are specifically designed around the standard WROOM-32 pinout. If you buy an "S3" or "C3" board, the pins will not match, the firmware will crash, and it will not fit in the case.

| Component | Recommendation & Notes |
| :--- | :--- |
| **Microcontroller** | [ESP32 Dev Board](https://www.amazon.com/dp/B08D5ZD528) (Standard 30 or 38-pin module). |
| **Servos** | [2x DS3218 20kg 270° Servos](https://www.amazon.com/dp/B08MTQ1QD1). I have tested and used 270-degree servos. 180 may/should work but untested. |
| **Display** | [ELEGOO 0.96" OLED](https://www.amazon.com/dp/B0D2RMQQHR). Must be 4-Pin I2C and use the **SSD1306** chip (avoid SH1106). |
| **Ground GPS** | Any standard UBlox FPV GPS module (e.g., BN-220, Walksnail M10, Matek M10). |
| **Power (BEC)** | [Castle Creations 10A BEC](https://www.readymaderc.com/products/details/castle-creations-bec-switching-regulator-10-amp-peak). **Crucial:** Never power servos from the ESP32's 5V pin. Use a dedicated 5V BEC (3A-5A continuous minimum) wired to the servos, sharing a common ground with the ESP32. |
| **Wiring** | [2x Wago Lever Nuts](https://www.digikey.com/short/q0m7mf3m). Highly recommended for cleanly distributing shared 5V and Ground lines. |
| **Switches** | [2x Momentary Buttons](https://www.amazon.com/dp/B07931588C). |
| **Servo Tester** | [Standard RC Servo Tester](https://www.amazon.com/dp/B08DM2CP3) to find your physical center PWM values. |
| **Capacitor** | [1000µF to 3300µF (10V-25V)](https://www.amazon.com/dp/B07R432MR2). Wire across the 5V and Ground Wago connectors to prevent heavy servos from causing voltage drops/reboots. |
| **Resistors** | [2x 1kΩ - 4.7kΩ Resistors](https://www.digikey.com/short/29dhzrrp). Wire as pull-down resistors (between the servo signal wire and ground) to prevent violent startup twitches. |
| *(Optional)* **Compass** | [Adafruit BNO085 9-DOF IMU](https://www.amazon.com/dp/B0CDGZMLPP). Selected because it does not require the "figure-8" calibration standard drone compasses need. |
| *(Optional)* **Trim Knob** | [10k Linear Potentiometer](https://www.amazon.com/dp/B082FCRQS2). |
| 3S Battery | Wired to BEC for power input. Doesn't have to be 3S but recommended so BEC doesn't have to step down voltage down to 5v too much. You may be able to get away with a 2S battery but not sure. |

🖨️ **3D Model Files:** Print the custom pan/tilt mechanics and electronics housing here: [MakerWorld: CRSF Antenna Tracker] *https://makerworld.com/en/models/2561665-crsf-and-mavlink-antenna-tracker?from=search#profileId-2822572*

---

## 🚀 Setup & Configuration

### Step 1: Flash the Firmware
You do not need to install the Arduino IDE or edit any code!
1. Go to the [Espressif Web Flasher](https://espressif.github.io/esptool-js/).
2. Connect your ESP32 via USB and click **Connect**. *(Tip: Block the 5V pin on your USB cable with tape to prevent the board from trying to pull servo power from your PC).*
3. Select the `CRSF_Tracker.merged.bin` file from the releases page.
4. **CRITICAL:** Ensure the Flash Address is set to `0x0`.
5. Click **Program**.

### Step 2: Find your ELRS Binding MAC Address (CRSF Users)
Because this tracker sniffs raw packets directly out of the air, it must impersonate your specific transmitter by converting your ELRS Binding Phrase into a 6-digit UID array.
1. Go to the [ExpressLRS UID Generator](https://www.expresslrs.org/hardware/spi-receivers/#binding-phrase-via-cli).
2. Type your secret Binding Phrase into the box.
3. Copy the UID bytes output (e.g., `252, 223, 149, 33, 43, 223`). Keep this handy for the next step.

### Step 3: The Web Configuration Portal
On its very first boot, the tracker will realize it has no saved settings and will automatically enter **Config Mode**. 
1. Open your phone or laptop's Wi-Fi settings and look for a new network called **`Tracker_Config`**.
2. Connect using the password: **`anttracker`**
3. Open a web browser and navigate to `192.168.4.1`.
4. Fill out the web form:
   * **Telemetry Mode:** Choose ESP-NOW (CRSF) for a fast-booting direct link, or WiFi (MAVLink) to relay through Mission Planner. The form will dynamically hide inputs you don't need!
   * **Servo Tuning:** Enter your exact servo PWM centers and limits. 
   * **Hardware Toggles:** Tell the code if you installed the optional BNO085 compass or physical Trim Knob.
5. Click **Save & Reboot**. The ESP32 will save your settings permanently.

*(Note: If you ever change hardware or want to update your limits, simply hold down the physical Home/Reset button while powering on the tracker to force it back into Config Mode!)*

### Step 4: Radio & Flight Controller Setup
1. **CRSF Users:** Ensure your TX Backpack is flashed with your binding phrase. In your model setup, turn **Telemetry ON**. Run the ELRS Lua Script and ensure the Backpack is enabled.
2. **ArduPilot Users (Crucial Fix):** If you use ArduPilot and are using ESP-NOW/CRSF, you **must disable CRSF Passthrough** (Bit 8 / Value 256 in `RC_OPTIONS`). Passthrough bundles telemetry into a custom format the tracker cannot read. Disabling it restores the standard CRSF GPS packets the tracker needs.
3. **MAVLink / Wi-Fi Users (Read Carefully!):** First, follow the official [ExpressLRS MAVLink documentation](https://www.expresslrs.org/software/mavlink/) to set up your backpack correctly. 
   * ⚠️ **WARNING:** Do *not* try to manually turn on "Backpack Wi-Fi" from your radio's ELRS Lua script when trying to fly. Doing this forces the backpack into firmware-update mode and instantly breaks the telemetry relay. 
   * **Testing Tip:** Before trying to connect the antenna tracker, connect your laptop to your backpack's Wi-Fi network and open Mission Planner. If you can get UDP telemetry on your laptop, the tracker will work flawlessly. 

---

## 🎯 Daily Flight Operations

1. **Boot Sequence:** Power up the ground station. The tracker waits for its local GPS to hit 8 satellites. Power up your aircraft; the tracker LED will blink until it receives the drone's telemetry confirming it also has 8 satellites.
2. **Calibration (Required before every flight):**
   * **With Compass:** Face the tripod toward your flight area. When the screen says "Ready," hold the calibrate button for 1 second.
   * **Without Compass:** Walk your powered aircraft 20-30m directly in front of the tracker. Physically rotate the tripod so the antennas point dead-center at the plane, then hold the calibrate button for 1 second.
3. **Flight:** The servos lock dead-center until the aircraft flies beyond the `MIN_TRACKING_DIST` (default 2 meters), at which point smooth tracking begins.

---

## 💡 Advanced Features & Troubleshooting

<details>
<summary><b>🛡️ The "Power Bump" Failsafe (Memory Restore)</b></summary>

Every time you calibrate, the tracker saves its Home location, servo math, and a live GPS timestamp. If your tracker loses power mid-flight and reboots, it rapidly checks this memory. 

If the tracker is still within 100m of its saved home, it bypasses the normal calibration requirements, restores the math, and immediately resumes tracking. This memory automatically expires after 3 hours. **To manually clear the memory** (e.g., moving to a new spot within 3 hours), hold the calibrate button for 5 seconds until the screen reads "RELEASE TO CLEAR".
</details>

<details>
<summary><b>🌬️ The Cross-Wind Launch Trick</b></summary>

Your pan servo has 135° of travel left and right from its calibration center. If you calibrate while pointing at a cross-wind launch pad 90° away from your main flight area, you will limit your tracking range during the actual flight. 

Instead: Calibrate the tracker pointing toward the *center* of your intended flight airspace. Once calibrated, pick up your plane and walk to your launch pad. The tracker will follow you, and as you launch and turn toward your main area, it will smoothly center itself with maximum travel available.
</details>

<details>
<summary><b>📺 Troubleshooting: OLED Screen is Black</b></summary>

If the firmware flashed successfully but the screen is dead, your OLED likely uses an alternate I2C address. To fix this, you must compile from source: open `main.cpp`, find `if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))`, change `0x3C` to `0x3D`, and re-upload via the Arduino IDE.
</details>

<details>
<summary><b>🎛️ Live RF Trim Knob</b></summary>

If installed and enabled in the Web UI, turning the physical potentiometer sweeps the entire tracker array up to 20° left or right mid-flight. This lets you manually dial in the invisible RF lobe of your patch antennas for the absolute best video feed without having to land and recalibrate.
</details>
