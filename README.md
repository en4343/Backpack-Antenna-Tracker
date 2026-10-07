# 📡 Antenna Tracker - Community Edition (CRSF & MAVLink)
A high-performance, open-source Antenna Tracker designed specifically for modern Long-Range FPV. 

No coding required! This tracker features a standalone Web UI for configuration and natively sniffs your telemetry directly out of the air. It grabs the GPS coordinates your flight controller is already broadcasting and points your high-gain patch antennas dead-center at your aircraft.

⚠️**This is an independent community project and is not affiliated with or endorsed by the ExpressLRS project. "ExpressLRS" and "ELRS" are used only to describe compatibility.**

### ⚠️ Critical Prerequisite: Your Transmitter 
Your ExpressLRS transmitter **MUST have a hardware "Backpack" chip installed**. The backpack is a secondary ESP32 or ESP8285 chip inside your radio/module dedicated to communicating with ground station gear.
* **Supported:** Most modern external modules (Radiomaster Ranger, Happymodel ES24TX, BetaFPV Micro) and internal modules (Radiomaster Boxer, TX16S MKII, GX12).
* **Unsupported:** Older or ultra-budget internal modules. Please verify your radio's specifications before building!

---

## ✨ Key Features
* **100% Wireless Data Link:** Reads native CRSF telemetry over ESP-NOW or Wi-Fi UDP (Backpack 1.5.7+), or MAVLink (v1 or v2) over Wi-Fi UDP. No extra hardware required on the drone/plane.
* **Accurate Tilt from the Aircraft's Barometer:** Uses the aircraft's own height-above-launch (barometer-based) when its telemetry includes it, which is far steadier than GPS altitude; falls back to GPS automatically.
* **Two Saved WiFi Networks:** Store your radio's backpack *and* a MAVLink WiFi bridge (e.g. DroneBridge). At boot the tracker scans and joins whichever is on the air, or falls back to ESP-NOW, and the screen always shows which link it's using.
* **Checksummed Telemetry:** Every CRSF and MAVLink position packet is CRC-checked before it is used, so a corrupted packet can never yank the antennas to a bogus position.
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
| **Battery** | **3S LiPo/Li-Ion Battery.** Wired to BEC for power input. Doesn't have to be 3S but recommended no higher than 3S so BEC doesn't have to step down voltage to 5v too much. You may be able to get away with a 2S battery but not sure. |

🖨️ **3D Model Files:** Print the custom pan/tilt mechanics and electronics housing here: [MakerWorld: CRSF Antenna Tracker](https://makerworld.com/en/models/2561665-crsf-and-mavlink-antenna-tracker?from=search#profileId-2822572)

---

## 🔌 Wiring & Pinout Guide

> ⚡ **CRITICAL POWER DISTRIBUTION WARNING:**
> Do **NOT** power the Pan and Tilt servos directly from the ESP32's 5V/VIN pin. FPV patch antennas are heavy, and moving them will instantly overdraw the ESP32, causing brown-outs, reboots, and burnt voltage regulators. 
> * Wire your battery to the input of your 5V BEC.
> * Connect the 5V BEC output to a Wago Lever Nut, and run wires to the Servos and the ESP32 `VIN` / `5V` pin.
> * **All grounds must be shared.** Connect the ESP32 GND, BEC GND, GPS GND, and Servo GNDs together.

| Component | ESP32 WROOM-32 Pin | Wiring Notes |
| :--- | :--- | :--- |
| **Pan Servo** | `GPIO 14` | Signal wire only. Add a 1k-4.7k pull-down resistor to GND to stop startup twitches. |
| **Tilt Servo** | `GPIO 13` | Signal wire only. Add a 1k-4.7k pull-down resistor to GND to stop startup twitches. |
| **UBlox GPS** | `RX2: 18`, `TX2: 19` | ESP32 `18` goes to GPS `TX`. ESP32 `19` goes to GPS `RX`. |
| **OLED Display** | `SDA: 21`, `SCL: 22` | Powered via ESP32 3.3V or 5V (check your specific screen). |
| **Calibrate/Home Button** | `GPIO 4` | Connect between `GPIO 4` and `GND`. No resistor needed (uses internal pull-up). |
| **Status LED** | `GPIO 25` | Connect to LED positive. LED negative goes to `GND` via a small (220Ω - 1kΩ) resistor. |
| **BNO085 Compass** *(Opt)* | `SDA: 21`, `SCL: 22`, `INT: 27`, `RST: 26` | Shares the `21`/`22` I2C bus with the OLED screen. |
| **Trim Potentiometer** *(Opt)*| `GPIO 35` | Middle wiper pin connects to `GPIO 35`. Outer pins connect to `3.3V` and `GND`. |

---

## 🚀 Setup & Configuration

### Step 1: Flash the Firmware
You do not need to install the Arduino IDE or edit any code! Each release includes two firmware files:

| File | Use it for | Flash address | Keeps your settings? |
| :--- | :--- | :--- | :--- |
| `CRSF_Tracker.ino.merged.bin` | **First install** on a new/blank ESP32 | `0x0` | ❌ No, erases everything |
| `CRSF_Tracker.ino.bin` | **Updating** a tracker that already runs this firmware | `0x10000` | ✅ Yes |

**First install (new board):**
1. Go to the [Espressif Web Flasher](https://espressif.github.io/esptool-js/) (use Chrome or Edge).
2. Connect your ESP32 via USB and click **Connect**. *(Tip: Block the 5V pin on your USB cable with tape to prevent the board from trying to pull servo power from your PC).* If no port shows up, install the USB driver for your board's USB chip (CP2102 or CH340).
3. Select the `CRSF_Tracker.ino.merged.bin` file from the releases page.
4. **CRITICAL:** Ensure the Flash Address is set to `0x0`.
5. Click **Program**. When it finishes, the tracker boots into Config Mode (Step 3).

**Updating to a new release (keeps your settings):**
1. Connect to the [Espressif Web Flasher](https://espressif.github.io/esptool-js/) as above.
2. Select `CRSF_Tracker.ino.bin` (the file *without* "merged" in the name).
3. **CRITICAL:** Set the Flash Address to `0x10000`. *Do not* click "Erase Flash".
4. Click **Program**. Your Web UI settings (servo limits, UID, WiFi, etc.) are kept.

> ⚠️ Flashing the `merged.bin` again also works for an update, but it wipes your saved settings and any saved failsafe calibration, so you'll have to fill out the Config page again. If anything goes wrong with an app-only update (boot loop, strange behaviour), do a clean first install with the `merged.bin`.

**Compatibility:** The firmware works on any board built on the original ESP32 chip with 4MB (or more) flash: ESP32-WROOM-32 / 32D / 32E / 32UE and WROVER modules, 30-pin or 38-pin dev boards. It will **not** run on ESP32-S2, S3, C3 or C6 boards, or on rare 2MB-flash boards.

### Step 2: Find your ELRS Binding MAC Address (CRSF Users)
Because this tracker sniffs raw packets directly out of the air, it must impersonate your specific transmitter by converting your ELRS Binding Phrase into a 6-digit UID array.
1. Go to the [ExpressLRS UID Generator](https://www.expresslrs.org/hardware/spi-receivers/#binding-phrase-via-cli).
2. Type your secret Binding Phrase into the box.
3. Copy the UID bytes output (e.g., `252, 223, 149, 33, 43, 223`). Keep this handy for the next step.
   * Enter all six numbers exactly as shown, even if the first one is odd. The backpack clears the lowest bit of the first byte before using it as a MAC address, and the tracker now does the same automatically.
   * The binding phrase must be the same one flashed into your **TX backpack**. If the backpack was flashed with a different phrase than your TX module, use the backpack's.

### Step 3: The Web Configuration Portal
On its very first boot, the tracker will realize it has no saved settings and will automatically enter **Config Mode**. 
1. Open your phone or laptop's Wi-Fi settings and look for a new network called **`Tracker_Config`**.
2. Connect using the password: **`anttracker`**
3. Open a web browser and navigate to `192.168.4.1`.
4. Fill out the web form:
   * **Telemetry Mode:** Choose ESP-NOW (CRSF) for a fast-booting direct link, WiFi UDP to receive telemetry over your backpack's WiFi (MAVLink, or CRSF with Backpack 1.5.7+, detected automatically), or **Auto-Detect**, which scans for your saved WiFi networks at boot and falls back to ESP-NOW if none of them are on the air. The form will dynamically hide inputs you don't need!
   * **WiFi Networks:** You can save **two** networks, e.g. Network 1 = your radio's backpack (`ExpressLRS TX Backpack XXXXXX`) and Network 2 = a DroneBridge / mLRS WiFi bridge. At boot the tracker scans and joins whichever one is on the air (Network 1 wins if both are). The joined network's name is shown on the screen at boot and on the tracking screen, so you always know which link it's using.
   * **Servo Tuning:** Enter your exact servo PWM centers and limits (use the servo tester to find them):
     * **Pan Center / Min / Max:** Min and Max are the PWM values at the two ends of the servo's full travel; together with *Total Servo Travel* they set the degrees-per-microsecond scale.
     * **Tilt Horizon PWM:** antennas perfectly level (0°).
     * **Tilt Up PWM (90°):** antennas pointing straight up. This sets the tilt scale, so measure it rather than guessing. If straight-up is a *higher* number than horizon, that's fine, the tilt simply runs in reverse.
     * **Tilt Down Limit PWM:** the physical limit on the below-horizon side. The servo is never driven past Up or Down.
   * **Hardware Toggles:** Tell the code if you installed the optional BNO085 compass or physical Trim Knob.
   * **Altitude Source:**
     * **Auto (default):** tilt uses the aircraft's own height above its launch point, from its barometer, whenever the telemetry includes it: MAVLink `GLOBAL_POSITION_INT.relative_alt`, or the CRSF barometric-altitude frame sent by Betaflight/INAV when the flight controller has a baro. It's accurate to about a meter and doesn't wander like GPS altitude. If it isn't available, the tracker uses the GPS altitude difference instead.
     * **GPS only:** choose this if you launch well above or below the tracker (e.g. hilltop launch, tracker in the valley). The aircraft's barometer measures height above *its launch point*, so Auto assumes you launch roughly level with the tracker.
5. Click **Save & Reboot**. The ESP32 will save your settings permanently.

*(Note: If you ever change hardware or want to update your limits, hold down the physical Home/Reset button while powering on the tracker and keep holding for about 2 seconds to force it back into Config Mode!)*

### Step 4: Radio & Flight Controller Setup
1. **CRSF Users:** Ensure your TX Backpack is flashed with your binding phrase. In your model setup, turn **Telemetry ON**. Run the ELRS Lua Script and ensure the Backpack is enabled. Then pick how the backpack sends telemetry (ELRS Lua → **Backpack** → **Telemetry**):
   * **ESPNOW:** set the tracker to *ESP-NOW / CRSF Only*. No WiFi setup needed.
   * **WiFi** (Backpack firmware **1.5.7 or newer**): the backpack broadcasts the same CRSF telemetry over WiFi UDP. Set the tracker to *WiFi UDP Only* (or Auto-Detect), enter the backpack's WiFi network (its own AP is `ExpressLRS TX Backpack XXXXXX`, password `expresslrs`, unless you set home/hotspot credentials when flashing) and port `14550`. In this mode the backpack does **not** send ESP-NOW, so the tracker must be on WiFi.
   * **Tip, switching per aircraft:** set the tracker to **Auto-Detect** with the backpack as Network 1. When the backpack's Telemetry is set to *WiFi* (e.g. MAVLink over ELRS), its network appears and the tracker joins it. Set it back to *ESPNOW* and the network disappears, so the tracker falls back to ESP-NOW automatically. The backpack Telemetry setting is radio-wide (not per model), so change it in the ELRS Lua script before flying a different aircraft.
2. **ArduPilot Users (Crucial Fix):** If you use ArduPilot and are using ESP-NOW/CRSF, you **must disable CRSF Passthrough** (Bit 8 / Value 256 in `RC_OPTIONS`). Passthrough bundles telemetry into a custom format the tracker cannot read. Disabling it restores the standard CRSF GPS packets the tracker needs.
3. **MAVLink / Wi-Fi Users (Read Carefully!):** First, follow the official [ExpressLRS MAVLink documentation](https://www.expresslrs.org/software/mavlink/) to set up your backpack correctly. 
   * ⚠️ **WARNING:** Do *not* try to manually turn on "Backpack Wi-Fi" from your radio's ELRS Lua script when trying to fly. Doing this forces the backpack into firmware-update mode and instantly breaks the telemetry relay. 
   * **Testing Tip:** Before trying to connect the antenna tracker, connect your laptop to your backpack's Wi-Fi network and open Mission Planner. If you can get UDP telemetry on your laptop, the tracker will work flawlessly. 
   * **What the tracker reads:** `GLOBAL_POSITION_INT` (position) and `GPS_RAW_INT` (fix type and satellite count). Both are in ArduPilot's default telemetry streams. If your flight controller doesn't send `GPS_RAW_INT`, the tracker still works but can't check the aircraft's satellite count (the screen will show 15 as a placeholder).
4. **Separate MAVLink radio (mLRS, SiK, etc.) with a WiFi bridge such as DroneBridge:** add the bridge's WiFi as Network 2 and use WiFi UDP Only or Auto-Detect, port `14550`. The tracker and your laptop/Mission Planner can both be connected to the bridge at the same time; no forwarding through the laptop is needed. If the aircraft also has an ELRS receiver, ESP-NOW via the ELRS backpack works too and keeps the tracker independent of the bridge.
   * **How the tracker registers itself:** DroneBridge only sends telemetry to devices that have sent it something. On MAVLink links the tracker sends a standard MAVLink heartbeat once a second (as an antenna tracker, system ID `252`), just like a ground station, so it's registered automatically. It uses system ID 252 rather than 255, so it never affects ArduPilot's GCS failsafe. The heartbeat is not sent on the ELRS backpack's CRSF-over-WiFi.
   * **Recommended DroneBridge settings:** *Wi-Fi Access Point Mode*; change the default password (anyone who knows it could join and send commands to your aircraft); **UART serial protocol: Transparent** (in *MAVLink* mode DroneBridge adds itself as a separate device and Mission Planner may connect to it instead of your flight controller); **"Disable radio when autopilot is armed" OFF** (otherwise the tracker and Mission Planner lose telemetry at takeoff); UART baud must match your radio's serial port (e.g. mLRS *Tx Ser Baudrate*).
   * **Serial speeds must match on each end:** ground radio ↔ bridge (e.g. mLRS *Tx Ser Baudrate* = DroneBridge *UART baud*), and air radio ↔ flight controller (e.g. mLRS *Rx Ser Baudrate* = ArduPilot `SERIALx_BAUD`). If the bridge's "received bytes" counter stays at 0, check these and the TX/RX wiring.
   * **Telemetry rates:** the tracker aims each time a position message arrives. On slow links (e.g. mLRS 31 Hz) set ArduPilot's `SRx_POSITION` ≥ 2 and `SRx_EXT_STAT` ≥ 1 for that serial port, so tracking doesn't depend on Mission Planner being connected.

---

## 🎯 Daily Flight Operations

1. **Boot Sequence:** Power up the ground station. The tracker waits for its local GPS to hit 8 satellites. Power up your aircraft; the tracker LED will blink until it receives the drone's telemetry confirming it also has 8 satellites.
2. **Calibration (Required before every flight):** Walk your powered aircraft 20-30m directly in front of the tracker and set it **on the ground**. Physically rotate the tripod so the antennas point dead-center at the plane. When the screen says "Ready," hold the calibrate button for 1 second.
   * The tracking screen's `Alt:` line shows `[REL]` when tilt is using the aircraft's barometer altitude, or `[GPS]` when it's using GPS. In `[GPS]` mode, give both GPS units 2–3 minutes after their fix (and an open view of the sky, away from walls) before calibrating, as GPS altitude takes a while to settle.
   * Calibration records "the tracker is pointing straight at the aircraft right now" and "the aircraft is at ground level right now", so both the direction and the ground placement matter. This is the same with or without the compass.
   * **What the compass adds:** if the tripod gets bumped or rotated *after* calibration, the BNO085 detects the rotation and the tracker corrects for it automatically. Without a compass, a bumped tripod means recalibrating.
   * If the compass is enabled but failed to start (screen showed "Compass FAIL" at boot), calibration automatically falls back to visual mode.
3. **Flight:** The servos hold their position until the aircraft is more than `MIN_TRACKING_DIST` (default 5 meters) away or `MIN_TRACKING_ALT` (default 5 meters) above the calibration point. Then smooth tracking begins. (Closer than ~5m, normal GPS wander makes the bearing to the aircraft jump around.)
   * **Link lost:** the antennas hold their last position and tracking resumes automatically when telemetry returns. After **5 minutes** with no telemetry (e.g. aircraft unplugged and packed away) the screen shows "Servos resting" and the tracker stops driving the servos (no buzzing or battery drain); they wake at the same position as soon as telemetry is back. No reboot needed.
   * **Blind spot:** the pan axis can't spin all the way around. With a 270° servo, the 90° behind the tracker is out of reach; if the aircraft crosses through it, the pan swings across to the other side. Point the tracker at the center of your flying area to keep the aircraft out of that zone.

---

### 🔘 Button Quick Reference

| Action | When | Result |
| :--- | :--- | :--- |
| **Hold ~2 sec while powering on** | At power-on | Enter Config Mode (Wi-Fi `Tracker_Config`, `192.168.4.1`) |
| **Tap 5 times quickly** | Any time, even while booting | Clear the saved failsafe calibration ("FAILSAFE CLEARED") |
| **Hold 1–5 sec, then release** | Screen says "Ready" | Calibrate |
| **Hold 5+ sec, then release** | After boot | Clear the saved failsafe calibration ("RELEASE TO CLEAR") |

---

## 💡 Advanced Features & Troubleshooting

<details>
<summary><b>🛡️ The "Power Bump" Failsafe (Memory Restore)</b></summary>

Every time you calibrate, the tracker saves its Home location, servo math, and a live GPS timestamp. If your tracker loses power mid-flight and reboots, it rapidly checks this memory. 

If the tracker is still within 100m of its saved home, it bypasses the normal calibration requirements, restores the math (pan direction and altitude reference), and resumes tracking as soon as the aircraft is more than 5m away. This memory automatically expires after 3 hours. **To manually clear the memory** (e.g., moving to a new spot within 3 hours, or you just swapped the tracker battery and don't want the servos snapping to the old position):
* **Tap the calibrate button 5 times quickly** (taps under ~0.5s each, less than 1.5s apart). This works at any time, **including while the tracker is still booting**, so the old calibration is wiped before it can be restored. The screen shows "FAILSAFE CLEARED".
* Or hold the calibrate button for 5 to 10 seconds until the screen reads "RELEASE TO CLEAR".
</details>

<details>
<summary><b>🌬️ The Cross-Wind Launch Trick</b></summary>

Your pan servo has 135° of travel left and right from its calibration center. If you calibrate while pointing at a cross-wind launch pad 90° away from your main flight area, you will limit your tracking range during the actual flight. 

Instead: Calibrate the tracker pointing toward the *center* of your intended flight airspace. Once calibrated, pick up your plane and walk to your launch pad. The tracker will follow you, and as you launch and turn toward your main area, it will smoothly center itself with maximum travel available.
</details>

<details>
<summary><b>📺 Troubleshooting: OLED Screen is Black</b></summary>

If the firmware flashed successfully but the screen is dead, your OLED likely uses an alternate I2C address. To fix this, you must compile from source (see below): open `CRSF_Tracker/CRSF_Tracker.ino`, find `if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))`, change `0x3C` to `0x3D`, and re-upload via the Arduino IDE.
</details>

<details>
<summary><b>📡 Troubleshooting: "No Link" (ESP-NOW / CRSF)</b></summary>

* Check the six UID numbers in the Web UI against the UID generated from your **backpack's** binding phrase.
* Make sure the TX backpack is enabled in the ELRS Lua script and that model telemetry is ON.
* "No Link" while the drone is powered usually means the backpack isn't sending. A drone count stuck at 0 while the link is OK means telemetry is arriving but there's no GPS frame in it (check the GPS on the aircraft, and for ArduPilot see the CRSF passthrough note in Step 4).
* If the screen shows **"MAC FAIL"** at boot, the UID entered in the Web UI is invalid. Re-enter it.
</details>

<details>
<summary><b>📶 Troubleshooting: "No Link" (WiFi / MAVLink)</b></summary>

* "No WiFi / Searching..." means neither saved network is visible or the password is wrong: check the SSID spelling (it's case-sensitive) and password. The network must broadcast its name (hidden SSIDs aren't found by the scan). In WiFi-only mode the tracker rescans every 15 seconds, so you can power the radio or bridge on after the tracker.
* If WiFi is connected but there's still "No Link", the second line of the screen shows a live reception summary, e.g. `P412 ok96 last3s`:
  * **P** = WiFi packets received since boot, **ok** = valid position/GPS messages decoded since boot, **last** = seconds since the last packet (`-` = never).
  * **P stays at 0:** nothing is being sent to the tracker. Check the UDP port (usually 14550), and confirm Mission Planner on a laptop receives telemetry from the same backpack/bridge.
  * **P grows but ok stays at 0:** packets arrive but don't contain the aircraft's position. Check the telemetry rates (`SRx_POSITION`, `SRx_EXT_STAT`) and, for ArduPilot over CRSF, the passthrough note in Step 4.
  * **`last` keeps jumping to several seconds:** telemetry is arriving in bursts; raise the position telemetry rate.
  * With the tracker plugged into USB, the same statistics are printed every second in the Arduino Serial Monitor (115200 baud).
* Aircraft satellite count stuck at exactly **15** on MAVLink: position is arriving but `GPS_RAW_INT` isn't; raise `SRx_EXT_STAT` to 1 Hz or more.
* Joined the wrong network? If both saved networks are on the air, Network 1 always wins. Turn the other one off, or swap them in the Config page.
</details>

<details>
<summary><b>🐢 Tuning: Servo Smoothness</b></summary>

`SERVO_SPEED` (top of the sketch, default `0.3`) controls how quickly the servos chase the target every 20ms. Lower values (e.g. `0.1`) give smoother, slower motion, which is easier on the gears with heavy patch antennas. Higher values are snappier.
</details>

---

## 🛠️ Building from Source

Only needed if you want to change the code (e.g., the OLED address or `SERVO_SPEED`).

1. Install **Arduino IDE 2.x**.
2. In **Boards Manager**, install **esp32 by Espressif Systems**, version 3.x (tested with 3.3.12). Version 2.x will **not** compile, because the ESP-NOW receive callback changed.
3. In **Library Manager**, install:
   * `ESP32Servo` (Kevin Harrington / madhephaestus)
   * `Adafruit BNO08x` (pulls in Adafruit BusIO and Adafruit Unified Sensor)
   * `Adafruit SSD1306` and `Adafruit GFX Library`
   * `SparkFun u-blox GNSS Arduino Library` (the **v2** library, not "v3")
4. Open `CRSF_Tracker/CRSF_Tracker.ino`, select board **ESP32 Dev Module** (or **DOIT ESP32 DEVKIT V1**), and upload.
5. To make release files, use **Sketch → Export Compiled Binary**. In the `build/...` folder it creates, attach both `CRSF_Tracker.ino.merged.bin` (first install, `0x0`) and `CRSF_Tracker.ino.bin` (updates, `0x10000`) to the GitHub release.

<details>
<summary><b>🎛️ Live RF Trim Knob</b></summary>

If installed and enabled in the Web UI, turning the physical potentiometer sweeps the entire tracker array up to 20° left or right mid-flight. This lets you manually dial in the invisible RF lobe of your patch antennas for the absolute best video feed without having to land and recalibrate.
</details>
