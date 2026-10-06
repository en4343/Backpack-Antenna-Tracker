/*=================================================================================================
    Antenna Tracker - "Community Edition" (Standalone Web UI / Dynamic Link Toggles / Help Text)

    Built with: Arduino-ESP32 core 3.x (tested 3.3.12), board "DOIT ESP32 DEVKIT V1"
    Libraries:  ESP32Servo, Adafruit BNO08x, Adafruit GFX, Adafruit SSD1306,
                SparkFun u-blox GNSS Arduino Library (v2.x)

    Changes in this revision:
      - ESP-NOW: clear the multicast bit of UID byte 0 like the ELRS backpack does (odd first
        UID byte previously failed silently)
      - CRSF: GPS frame located by length/type and verified with CRC8
      - MAVLink: v1 + v2, CRC checked, can't lock up, real sat count from GPS_RAW_INT
      - WiFi mode also accepts CRSF telemetry over UDP (ELRS Backpack 1.5.7+ "WiFi" telemetry)
      - 5 quick taps on the button clears the saved failsafe calibration, even during boot
      - Config mode now needs the button held for 1.5s at power-on (stray taps won't enter it)
      - GPS: UBX-only + autoPVT so the main loop no longer blocks on every GPS read
      - Tilt now uses the web UI PWM settings instead of hard-coded 1000-1600us
      - Power-bump failsafe also restores the altitude reference
      - WiFi-only mode keeps retrying after a boot timeout instead of dead-ending
      - Tracking gate uses calibrated altitude; minimum tracking distance 2m -> 5m
=================================================================================================*/

#include <Arduino.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include <Preferences.h> 
#include <WebServer.h>

#include <Adafruit_BNO08x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 

// =======================================================================================
// SYSTEM PINS & CONSTANTS (Baked in for Community Standard Build)
// =======================================================================================
#define SDA 21             // I2C Data
#define SCL 22             // I2C Clock
#define BNO_INT 27         // Compass Interrupt
#define BNO_RST 26         // Compass Reset
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET -1 

#define gps_rxPin 18       // GPS RX
#define gps_txPin 19       // GPS TX
#define PIN_RESET_HOME 4   // Button to clear failsafe / config mode
#define StatusLed 25       // Indicator LED
#define azPWM_Pin 14       // Pan Servo
#define elPWM_Pin 13       // Tilt Servo
#define TRIM_POT_PIN 35    // Analog Trim Knob

#define MIN_SATS 8
#define FAILSAFE_TIMEOUT 180   // Minutes the saved calibration stays valid after a power loss
#define MIN_TRACKING_DIST 5    // Meters. GPS noise is ~2-3m, so bearings closer than this are mostly noise
#define MIN_TRACKING_ALT 5     // Meters above the calibrated ground level
#define SERVO_SPEED 0.3        // Smoothing factor per 20ms step (lower = smoother/slower, e.g. 0.1)
#define MAX_TRIM_ANGLE 20

#ifndef CRSF_FRAMETYPE_GPS
  #define CRSF_FRAMETYPE_GPS 0x02
#endif
#define CRSF_GPS_FRAME_LEN 0x11  // CRSF length byte for a GPS frame: type(1) + payload(15) + crc(1)

// MAVLink message IDs and their CRC_EXTRA seeds (from the MAVLink common.xml definitions)
#define MAVLINK_MSG_GPS_RAW_INT          24
#define MAVLINK_MSG_GPS_RAW_INT_CRC      24
#define MAVLINK_MSG_GLOBAL_POSITION_INT  33
#define MAVLINK_MSG_GLOBAL_POSITION_CRC  104

// =======================================================================================
// GLOBAL STATE VARIABLES & ACTIVE CONFIG
// =======================================================================================

portMUX_TYPE telemetryMux = portMUX_INITIALIZER_UNLOCKED;

// --- Active User Settings (Loaded from Flash) ---
bool is_configured = false;

int active_link_type; // 0=ESP-NOW, 1=WiFi, 2=Auto
String active_ssid;
String active_pass;
uint16_t active_port;
uint8_t active_mac[6];

bool active_use_compass;
bool active_use_trim_knob;
bool active_reverse_pan;

int active_pan_center;
int active_tilt_horizon;
int active_min_az_pwm;
int active_max_az_pwm;
int active_min_el_pwm;
int active_max_el_pwm;
int active_max_el;
int active_pan_servo_degrees; 
int active_max_pan;           

// --- Telemetry (Drone) Data ---
volatile double droneLat = 0;
volatile double droneLon = 0;
volatile float droneAlt = 0;
volatile int droneSats = 0;
volatile bool linkConnected = false;
volatile unsigned long lastPacketTime = 0;   // Written from the ESP-NOW callback (WiFi task)
unsigned long lastGpsRawTime = 0;            // Last MAVLink GPS_RAW_INT (real satellite count)
int currentChannel = 1;
bool channelLocked = false;
bool usingWiFi = false;
bool udpStarted = false;

// --- Ground Station (Box) Data ---
double boxLat = 0;
double boxLon = 0;
float boxAlt = 0;
bool boxGPSFixed = false;
int boxSats = 0;
float trackerHeading = 0;
bool compassGood = false;
bool gpsGood = false;
bool localGpsOk = false;

// --- Calibration & Tracking State ---
float panOffset = 0;
float altOffset = 0; 
bool homeEstablished = false; 
bool calibrationDone = false; 
int trimOffsetVal = 0; 
int currentTrim = 0;   

// --- Hardware Objects ---
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
SFE_UBLOX_GNSS myGNSS;
HardwareSerial gpsSerial(2);
WiFiUDP udp;
WebServer server(80);
Adafruit_BNO08x bno08x(BNO_RST);
sh2_SensorValue_t sensorValue;

Preferences preferences; 
Servo azServo;            
Servo elServo;   
bool servosAwake = false;
float currentPanPWM = 1500;   // Smoothed servo outputs (seeded when the servos wake)
float currentTiltPWM = 1500;
uint8_t ledState = LOW;
uint32_t millisLED = 0;

struct Location { double lat; double lon; float alt; float hdg; float alt_ag; };
struct Location hom = { 0,0,0,0,0};   
struct Location cur = { 0,0,0,0,0};   
struct Vector { float az; float el; int32_t dist; };
struct Vector hc_vector  = { 90, 0, 0};

// FORWARD DECLARATIONS
void pointServos(uint16_t az, uint16_t el);
void getAzEl(struct Location &home, struct Location &current);
float getDist(struct Location &a, struct Location &b);
void StartEspNow();
void PerformCalibration();
void ClearFailsafe();
void CheckFailsafe(); 
void BlinkLed(uint16_t period);
void ServiceTheStatusLed();
void ReadLocalGPS(); 
void ReadCompass();
void WakeServos();

// =======================================================================================
// LOGGING & DISPLAY
// =======================================================================================
void LogScreenPrintln(String s, String s2 = "") {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); 
  display.println("Antenna Tracker");
  display.println("----------------");
  display.setTextSize(2); display.println(s);
  if (s2 != "") { display.setTextSize(1); display.println(s2); }
  display.setTextSize(1);
  display.setCursor(0, 45); display.print("Tracker: "); display.print(boxSats);
  if(boxGPSFixed) display.print(" [FIX]");
  display.setCursor(0, 55); display.print("Drone:   "); display.print(droneSats);
  display.display();
}

void UpdateDisplay(int targetAz, int trimVal) {
  static unsigned long timer = 0;
  if (millis() - timer > 200) {
    timer = millis();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    float rawDiff = cur.alt - hom.alt;
    float trueDiff = rawDiff - altOffset; 
    String mode = "REL"; 
    
    display.print("Alt:"); display.print(trueDiff, 0);
    display.print("m ["); display.print(mode); display.println("]");
    display.print("Dist:"); display.print(hc_vector.dist); display.println("m");

    int tiltAngle = hc_vector.el; 
    if (tiltAngle < 0) tiltAngle = 0;
    display.print("TiltCMD: "); display.println(tiltAngle); 

    float trueHeading = trackerHeading + panOffset;
    while (trueHeading >= 360.0) trueHeading -= 360.0;
    while (trueHeading < 0.0) trueHeading += 360.0;
    display.print("Hdg:"); display.print(trueHeading, 0);
    
    if (hc_vector.dist < MIN_TRACKING_DIST) display.println(" [LOCK]");
    else {
        display.print(" Trim:"); 
        if(trimVal > 0) display.print("+");
        else if (trimVal == 0 && !active_use_trim_knob) display.print("OFF");
        display.print(trimVal);
        display.println("");
    }
    display.display();
  }
}

// =======================================================================================
// WEB CONFIGURATION PORTAL
// =======================================================================================
const char* htmlTemplate = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Tracker Config</title>
<style>
  body { font-family: 'Segoe UI', Arial, sans-serif; padding: 20px; background-color: #121212; color: #ffffff; max-width: 650px; margin: auto; }
  h2 { border-bottom: 2px solid #333; padding-bottom: 5px; margin-top: 30px; color: #007BFF; }
  label { display: block; margin-top: 15px; font-weight: bold; color: #ddd; font-size: 14px; }
  .help { display: block; font-size: 12px; color: #888; margin-top: 2px; margin-bottom: 8px; font-style: italic; }
  input, select { width: 100%; padding: 12px; margin-top: 2px; border-radius: 6px; border: 1px solid #444; background: #222; color: #fff; box-sizing: border-box; font-size: 16px; }
  .row { display: flex; gap: 15px; }
  .col { flex: 1; }
  button { background-color: #007BFF; color: white; padding: 15px; width: 100%; border: none; border-radius: 6px; font-size: 18px; font-weight: bold; margin-top: 35px; margin-bottom: 30px; cursor: pointer; }
  button:hover { background-color: #0056b3; }
  .req { color: #ff4444; }
</style></head><body>

<form action="/save" method="POST">
  <h2>Network & Telemetry</h2>
  
  <label>Telemetry Connection Mode <span class="req">*</span></label>
  <span class="help">How should the tracker receive GPS data from your radio?</span>
  <select name="link_type" id="link_type" onchange="toggleFields()" required>
    <option value="2" V_LINK2>Auto-Detect (Try WiFi first, fallback to ESP-NOW)</option>
    <option value="0" V_LINK0>ESP-NOW / CRSF Only (Fastest Boot)</option>
    <option value="1" V_LINK1>WiFi UDP Only (MAVLink or CRSF)</option>
  </select>

  <div id="wifi_settings">
    <div class="row">
      <div class="col">
        <label>WiFi SSID <span class="req">*</span></label>
        <span class="help">Backpack AP (e.g. ExpressLRS TX Backpack XXXXXX) or your hotspot</span>
        <input type="text" name="ssid" id="ssid" value="V_SSID">
      </div>
      <div class="col">
        <label>WiFi Password <span class="req">*</span></label>
        <span class="help">Your Backpack Password</span>
        <input type="text" name="pass" id="pass" value="V_PASS">
      </div>
    </div>
    <label>UDP Listen Port <span class="req">*</span></label>
    <span class="help">Backpack UDP port (default 14550). Works for MAVLink or CRSF-over-WiFi (Backpack 1.5.7+).</span>
    <input type="number" name="port" id="port" value="V_PORT">
  </div>

  <div id="esp_settings">
    <label>ELRS Binding MAC (Comma Separated) <span class="req">*</span></label>
    <span class="help">Your UID bytes from the ELRS Configurator (e.g., 252,223,149,33,213,228)</span>
    <input type="text" name="mac" id="mac" value="V_MAC" placeholder="e.g. 252,223,149,33,213,228">
  </div>

  <h2>Hardware Toggles</h2>
  <div class="row">
    <div class="col">
      <label>Use BNO085 Compass?</label>
      <span class="help">Requires physical I2C compass</span>
      <select name="use_comp">
        <option value="1" V_COMP1>Yes (Auto Calibrate)</option>
        <option value="0" V_COMP0>No (Visual Calibrate)</option>
      </select>
    </div>
    <div class="col">
      <label>Use Trim Knob?</label>
      <span class="help">Requires physical potentiometer</span>
      <select name="use_trim">
        <option value="1" V_TRIM1>Yes</option>
        <option value="0" V_TRIM0>No</option>
      </select>
    </div>
  </div>

  <h2>Servo Tuning (Pan Axis)</h2>
  <div class="row">
    <div class="col">
      <label>Pan Center PWM <span class="req">*</span></label>
      <span class="help">Dead-center forward (usually ~1500)</span>
      <input type="number" name="pan_c" value="V_PANC" required>
    </div>
    <div class="col">
      <label>Reverse Pan Axis</label>
      <span class="help">Flips left and right</span>
      <select name="rev_pan">
        <option value="1" V_REV1>Yes (Reverse)</option>
        <option value="0" V_REV0>No (Standard)</option>
      </select>
    </div>
  </div>

  <div class="row">
    <div class="col">
      <label>Pan Min PWM <span class="req">*</span></label>
      <span class="help">Absolute physical limit (e.g., 500)</span>
      <input type="number" name="pan_min" value="V_PANMIN" required>
    </div>
    <div class="col">
      <label>Pan Max PWM <span class="req">*</span></label>
      <span class="help">Absolute physical limit (e.g., 2500)</span>
      <input type="number" name="pan_max" value="V_PANMAX" required>
    </div>
  </div>

  <div class="row">
    <div class="col">
      <label>Total Servo Travel <span class="req">*</span></label>
      <span class="help">Hardware specs (Usually 270 or 180)</span>
      <input type="number" name="pan_deg" value="V_PANDEG" min="90" max="270" required>
    </div>
    <div class="col">
      <label>Max Pan Angle (Virtual Wall) <span class="req">*</span></label>
      <span class="help">Degrees to swing Left/Right from center. For full 270° sweep, enter 135 (135L+135R).</span>
      <input type="number" name="max_pan" value="V_MAXPAN" min="10" max="180" required>
    </div>
  </div>

  <h2>Servo Tuning (Tilt Axis)</h2>
  <label>Tilt Horizon PWM (Level) <span class="req">*</span></label>
  <span class="help">PWM value when patch antennas point perfectly level</span>
  <input type="number" name="tilt_h" value="V_TILTH" required>

  <div class="row">
    <div class="col">
      <label>Tilt Up PWM (90&deg;) <span class="req">*</span></label>
      <span class="help">PWM when antennas point straight up (90&deg;). Sets the tilt scale. Higher than Horizon = reversed tilt servo.</span>
      <input type="number" name="tilt_min" value="V_TILTMIN" required>
    </div>
    <div class="col">
      <label>Tilt Down Limit PWM <span class="req">*</span></label>
      <span class="help">Physical limit on the below-horizon side (servo is never driven past this)</span>
      <input type="number" name="tilt_max" value="V_TILTMAX" required>
    </div>
  </div>

  <label>Max Tilt Angle Limit <span class="req">*</span></label>
  <span class="help">Safety limit in degrees (e.g., 90 for straight up)</span>
  <input type="number" name="max_el" value="V_MAXEL" min="10" max="90" required>

  <button type="submit">Save & Reboot Tracker</button>
</form>

<script>
function toggleFields() {
  var val = document.getElementById('link_type').value;
  var w = document.getElementById('wifi_settings');
  var e = document.getElementById('esp_settings');
  var s = document.getElementById('ssid');
  var p = document.getElementById('pass');
  var pt = document.getElementById('port');
  var m = document.getElementById('mac');

  if(val == '0') {
    // ESP-NOW Only
    w.style.display = 'none'; s.required = false; p.required = false; pt.required = false;
    e.style.display = 'block'; m.required = true;
  } else if(val == '1') {
    // WiFi Only
    w.style.display = 'block'; s.required = true; p.required = true; pt.required = true;
    e.style.display = 'none'; m.required = false;
  } else {
    // Auto-Detect
    w.style.display = 'block'; s.required = true; p.required = true; pt.required = true;
    e.style.display = 'block'; m.required = true;
  }
}
window.onload = toggleFields;
</script>

</body></html>
)rawliteral";

// Escape user text before inserting it into an HTML value="..." attribute
String htmlEscape(const String &s) {
    String out; out.reserve(s.length() + 8);
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '&') out += "&amp;";
        else if (c == '"') out += "&quot;";
        else if (c == '<') out += "&lt;";
        else if (c == '>') out += "&gt;";
        else out += c;
    }
    return out;
}

void parseMacString(String macStr, uint8_t* macArr) {
    memset(macArr, 0, 6);
    int arrIdx = 0; int strIdx = 0;
    while(arrIdx < 6 && strIdx < macStr.length()) {
        int commaIdx = macStr.indexOf(',', strIdx);
        if(commaIdx == -1) commaIdx = macStr.length();
        macArr[arrIdx] = macStr.substring(strIdx, commaIdx).toInt();
        strIdx = commaIdx + 1;
        arrIdx++;
    }
}

void StartWebConfig() {
    display.clearDisplay(); display.setCursor(0,0);
    display.setTextSize(2); display.setTextColor(SSD1306_WHITE); 
    display.println("CONFIG"); display.println(" MODE");
    display.setTextSize(1);
    display.setCursor(0, 45); display.print("AP: Tracker_Config");
    display.setCursor(0, 55); display.print("IP: 192.168.4.1");
    display.display();

    WiFi.mode(WIFI_AP);
    WiFi.softAP("Tracker_Config", "anttracker");

    server.on("/", HTTP_GET, []() {
        String html = String(htmlTemplate);
        
        html.replace("V_LINK0", active_link_type == 0 ? "selected" : "");
        html.replace("V_LINK1", active_link_type == 1 ? "selected" : "");
        html.replace("V_LINK2", active_link_type == 2 ? "selected" : "");

        html.replace("V_SSID", htmlEscape(active_ssid));
        html.replace("V_PASS", htmlEscape(active_pass));
        html.replace("V_PORT", active_port ? String(active_port) : "14550");
        
        String macStr = "";
        if (is_configured) {
            macStr = String(active_mac[0]) + "," + String(active_mac[1]) + "," + 
                     String(active_mac[2]) + "," + String(active_mac[3]) + "," + 
                     String(active_mac[4]) + "," + String(active_mac[5]);
        }
        html.replace("V_MAC", macStr);

        html.replace("V_PANC", active_pan_center ? String(active_pan_center) : "1500");
        html.replace("V_PANMIN", active_min_az_pwm ? String(active_min_az_pwm) : "500");
        html.replace("V_PANMAX", active_max_az_pwm ? String(active_max_az_pwm) : "2500");
        html.replace("V_TILTH", active_tilt_horizon ? String(active_tilt_horizon) : "1500");
        html.replace("V_TILTMIN", active_min_el_pwm ? String(active_min_el_pwm) : "1000");
        html.replace("V_TILTMAX", active_max_el_pwm ? String(active_max_el_pwm) : "1700");
        html.replace("V_MAXEL", active_max_el ? String(active_max_el) : "90");
        html.replace("V_PANDEG", active_pan_servo_degrees ? String(active_pan_servo_degrees) : "270");
        html.replace("V_MAXPAN", active_max_pan ? String(active_max_pan) : "135");

        if (active_reverse_pan) { html.replace("V_REV1", "selected"); html.replace("V_REV0", ""); } 
        else { html.replace("V_REV1", ""); html.replace("V_REV0", "selected"); }

        if (active_use_compass) { html.replace("V_COMP1", "selected"); html.replace("V_COMP0", ""); } 
        else { html.replace("V_COMP1", ""); html.replace("V_COMP0", "selected"); }

        if (active_use_trim_knob) { html.replace("V_TRIM1", "selected"); html.replace("V_TRIM0", ""); } 
        else { html.replace("V_TRIM1", ""); html.replace("V_TRIM0", "selected"); }

        server.send(200, "text/html", html);
    });

    server.on("/save", HTTP_POST, []() {
        preferences.begin("tracker_cfg", false);
        
        preferences.putUInt("link_type", server.arg("link_type").toInt());
        
        preferences.putString("ssid", server.arg("ssid"));
        preferences.putString("pass", server.arg("pass"));
        preferences.putUInt("port", server.arg("port").toInt());
        
        uint8_t tempMac[6];
        parseMacString(server.arg("mac"), tempMac);
        preferences.putBytes("mac", tempMac, 6);

        preferences.putUInt("pan_c", server.arg("pan_c").toInt());
        preferences.putUInt("pan_min", server.arg("pan_min").toInt());
        preferences.putUInt("pan_max", server.arg("pan_max").toInt());
        
        // Server-side sanity limits (a 0 here would cause a divide-by-zero in the servo math)
        int p_deg = constrain((int)server.arg("pan_deg").toInt(), 90, 270);
        preferences.putUInt("pan_deg", p_deg);

        preferences.putUInt("max_pan", constrain((int)server.arg("max_pan").toInt(), 10, 180));
        preferences.putUInt("tilt_h", server.arg("tilt_h").toInt());
        preferences.putUInt("tilt_min", server.arg("tilt_min").toInt());
        preferences.putUInt("tilt_max", server.arg("tilt_max").toInt());
        preferences.putUInt("max_el", constrain((int)server.arg("max_el").toInt(), 10, 90));
        
        preferences.putBool("rev_pan", server.arg("rev_pan").toInt() == 1);
        preferences.putBool("use_comp", server.arg("use_comp").toInt() == 1);
        preferences.putBool("use_trim", server.arg("use_trim").toInt() == 1);

        preferences.putBool("configured", true); 
        preferences.end();
        
        server.send(200, "text/html", "<h2 style='font-family:Arial; color:green; text-align:center; margin-top:50px;'>Settings Saved! Rebooting Tracker...</h2>");
        delay(2000);
        ESP.restart();
    });

    server.begin();
    while (true) { server.handleClient(); delay(2); }
}

// =======================================================================================
// HARDWARE CONTROL
// =======================================================================================
void WakeServos() {
    if (!servosAwake) {
        // ESP32Servo ignores writes made before attach(), so attach first, then command
        // center/horizon. min()/max() keep the limits valid for reversed servos.
        azServo.attach(azPWM_Pin, min(active_min_az_pwm, active_max_az_pwm), max(active_min_az_pwm, active_max_az_pwm));
        elServo.attach(elPWM_Pin, min(active_min_el_pwm, active_max_el_pwm), max(active_min_el_pwm, active_max_el_pwm));
        azServo.writeMicroseconds(active_pan_center);
        elServo.writeMicroseconds(active_tilt_horizon);
        // Seed the smoothing filter from center so the first move is eased, not a jump
        currentPanPWM = active_pan_center;
        currentTiltPWM = active_tilt_horizon;
        servosAwake = true;
    }
}

// =======================================================================================
// TELEMETRY PARSERS (Universal CRSF & MAVLink)
// =======================================================================================
// --- CRSF ------------------------------------------------------------------------------
// The ELRS TX backpack forwards each CRSF telemetry frame over ESP-NOW wrapped in an MSP
// packet: [MSP header ...][sync][len][type][payload...][crc8][MSP crc]. Instead of guessing a
// fixed offset, look for a GPS frame anywhere in the packet (len byte 0x11 followed by type
// 0x02) and only accept it if the CRSF CRC8 matches. This stops random bytes in other frame
// types (battery, attitude, flight mode, etc.) from ever being decoded as a GPS position.
uint8_t crsfCrc8(const uint8_t *p, uint8_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *p++;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

void parseCRSF(const uint8_t *data, int data_len) {
  // i = index of the type byte. Need len byte before it and 15 payload + 1 crc after it.
  for (int i = 1; i + 16 < data_len; i++) {
    if (data[i] != CRSF_FRAMETYPE_GPS || data[i - 1] != CRSF_GPS_FRAME_LEN) continue;
    if (crsfCrc8(&data[i], 16) != data[i + 16]) continue;   // CRC covers type + payload

    const uint8_t *p = &data[i + 1];   // Payload (all fields big-endian)
    int32_t lat = (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
    int32_t lon = (int32_t)(((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) | ((uint32_t)p[6] << 8) | (uint32_t)p[7]);
    // p[8..9] ground speed, p[10..11] heading
    uint16_t altitudeRaw = ((uint16_t)p[12] << 8) | p[13];   // Meters MSL + 1000 offset
    uint8_t sats = p[14];

    portENTER_CRITICAL(&telemetryMux);
    droneLat = (double)lat / 10000000.0;
    droneLon = (double)lon / 10000000.0;
    droneAlt = (float)altitudeRaw - 1000.0f;
    droneSats = sats;
    portEXIT_CRITICAL(&telemetryMux);
    return;
  }
}

void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int data_len) {
    lastPacketTime = millis();
    linkConnected = true;
    if (!channelLocked) { channelLocked = true; Serial.println("ESP-NOW LINK OK!"); }
    parseCRSF(data, data_len);
}

// --- MAVLink -----------------------------------------------------------------------------
// Minimal MAVLink v1 + v2 parser with full CRC checking. Only two messages are decoded:
//   GLOBAL_POSITION_INT (33) - lat/lon/alt used for tracking
//   GPS_RAW_INT (24)         - real fix type + satellite count for the 8-sat "Gatekeeper"
static inline void mavCrcAccumulate(uint8_t b, uint16_t &crc) {
    uint8_t tmp = b ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    crc = (crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ (tmp >> 4);
}

static inline int32_t mavInt32(const uint8_t *p) {   // MAVLink fields are little-endian
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

void handleMavlinkMessage(uint32_t msgId, uint8_t *payload) {
    if (msgId == MAVLINK_MSG_GPS_RAW_INT) {
        // Wire order: time_usec(8) lat(4) lon(4) alt(4) eph(2) epv(2) vel(2) cog(2) fix_type(1) sats(1)
        uint8_t fixType = payload[28];
        uint8_t sats = payload[29];
        lastGpsRawTime = millis();
        portENTER_CRITICAL(&telemetryMux);
        droneSats = (fixType >= 3) ? sats : 0;   // Only count satellites once there's a 3D fix
        portEXIT_CRITICAL(&telemetryMux);
    }
    else if (msgId == MAVLINK_MSG_GLOBAL_POSITION_INT) {
        // Wire order: time_boot_ms(4) lat(4) lon(4) alt(4, mm MSL) relative_alt(4) vx vy vz hdg
        int32_t latInt = mavInt32(&payload[4]);
        int32_t lonInt = mavInt32(&payload[8]);
        int32_t altMm  = mavInt32(&payload[12]);
        if (latInt != 0 && lonInt != 0) {
            portENTER_CRITICAL(&telemetryMux);
            droneLat = (double)latInt / 10000000.0;
            droneLon = (double)lonInt / 10000000.0;
            droneAlt = altMm / 1000.0;
            // Older behaviour as a fallback: if the FC isn't streaming GPS_RAW_INT, assume a good fix
            if (lastGpsRawTime == 0 || millis() - lastGpsRawTime > 5000) droneSats = 15;
            portEXIT_CRITICAL(&telemetryMux);
        }
    }
}

void parseMavlink(uint8_t c) {
    enum { WAIT_STX, HEADER, PAYLOAD, CRC_LO, CRC_HI };
    static uint8_t state = WAIT_STX;
    static bool isV2 = false;
    static uint8_t hdr[9];          // v2: len,incompat,compat,seq,sys,comp,msgid x3 | v1: len,seq,sys,comp,msgid
    static uint8_t hdrIdx = 0;
    static uint8_t payloadLen = 0;
    static uint8_t payload[255];
    static uint16_t payloadIdx = 0;
    static uint32_t msgId = 0;
    static uint16_t crc = 0xFFFF;
    static uint8_t crcLo = 0;

    switch (state) {
        case WAIT_STX:
            if (c == 0xFD || c == 0xFE) { isV2 = (c == 0xFD); hdrIdx = 0; crc = 0xFFFF; state = HEADER; }
            break;

        case HEADER:
            hdr[hdrIdx++] = c;
            mavCrcAccumulate(c, crc);
            if (hdrIdx == (isV2 ? 9 : 5)) {
                payloadLen = hdr[0];
                msgId = isV2 ? ((uint32_t)hdr[6] | ((uint32_t)hdr[7] << 8) | ((uint32_t)hdr[8] << 16)) : hdr[4];
                payloadIdx = 0;
                memset(payload, 0, sizeof(payload));   // MAVLink2 trims trailing zero bytes; restore them
                state = (payloadLen == 0) ? CRC_LO : PAYLOAD;
            }
            break;

        case PAYLOAD:
            payload[payloadIdx++] = c;
            mavCrcAccumulate(c, crc);
            if (payloadIdx >= payloadLen) state = CRC_LO;
            break;

        case CRC_LO:
            crcLo = c;
            state = CRC_HI;
            break;

        case CRC_HI: {
            state = WAIT_STX;
            uint8_t crcExtra;
            if (msgId == MAVLINK_MSG_GLOBAL_POSITION_INT) crcExtra = MAVLINK_MSG_GLOBAL_POSITION_CRC;
            else if (msgId == MAVLINK_MSG_GPS_RAW_INT)    crcExtra = MAVLINK_MSG_GPS_RAW_INT_CRC;
            else break;                                   // Not a message we use
            uint16_t check = crc;
            mavCrcAccumulate(crcExtra, check);
            if (check == (uint16_t)(crcLo | (c << 8))) handleMavlinkMessage(msgId, payload);
            break;
        }
    }
}

// ELRS Backpack 1.5.7+ with Telemetry = "WiFi" broadcasts CRSF telemetry over UDP (same port
// as MAVLink, default 14550), one MSP v2 packet per datagram - the exact same bytes it sends
// over ESP-NOW: '$' 'X' dir flags func(2) size(2) payload(size) crc  ->  size + 9 bytes total
#define MSP_ELRS_BACKPACK_CRSF_TLM 0x0011
bool isBackpackCrsfPacket(const uint8_t *b, int len) {
    if (len < 9 || b[0] != '$' || b[1] != 'X') return false;
    uint16_t func = b[4] | ((uint16_t)b[5] << 8);
    uint16_t size = b[6] | ((uint16_t)b[7] << 8);
    return func == MSP_ELRS_BACKPACK_CRSF_TLM && (int)size + 9 == len;
}

void readUDP() {
    // Drain every queued datagram each loop so telemetry never backs up
    for (int n = 0; n < 16; n++) {
        int packetSize = udp.parsePacket();
        if (packetSize <= 0) break;

        lastPacketTime = millis();
        linkConnected = true;
        if (!channelLocked) { channelLocked = true; Serial.println("WIFI UDP LINK OK!"); }

        uint8_t buffer[512];
        int len = udp.read(buffer, sizeof(buffer));
        if (len <= 0) continue;

        // Auto-detect per packet: CRSF-over-WiFi from the backpack, otherwise MAVLink
        if (isBackpackCrsfPacket(buffer, len)) parseCRSF(buffer, len);
        else for (int i = 0; i < len; i++) parseMavlink(buffer[i]);
    }
}

// =======================================================================================
// SENSORS (GPS & COMPASS)
// =======================================================================================
void ReadLocalGPS() {
  if (myGNSS.getGnssFixOk()) {
      boxGPSFixed = true;
      boxSats = myGNSS.getSIV();
      boxLat = (double)myGNSS.getLatitude() / 10000000.0;
      boxLon = (double)myGNSS.getLongitude() / 10000000.0;
      
      if (!calibrationDone) {
          // MSL altitude, to match what CRSF and MAVLink report for the aircraft
          // (getAltitude() is height above the ellipsoid, which differs by tens of meters)
          boxAlt = myGNSS.getAltitudeMSL() / 1000.0;
          hom.alt = boxAlt;
      }
      hom.lat = boxLat;
      hom.lon = boxLon;
      homeEstablished = true;
  } else {
      boxGPSFixed = false;
      boxSats = myGNSS.getSIV(); 
  }
}

void ReadCompass() {
    if (!active_use_compass || !compassGood) return;
    if (bno08x.wasReset()) bno08x.enableReport(SH2_ROTATION_VECTOR, 50000); 
    
    while (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            float qr = sensorValue.un.rotationVector.real;
            float qi = sensorValue.un.rotationVector.i;
            float qj = sensorValue.un.rotationVector.j;
            float qk = sensorValue.un.rotationVector.k;

            float sqr = sq(qr); float sqi = sq(qi);
            float sqj = sq(qj); float sqk = sq(qk);
            
            float siny_cosp = 2.0 * (qr * qk + qi * qj);
            float cosy_cosp = 1.0 - 2.0 * (sqj + sqk);
            trackerHeading = atan2(siny_cosp, cosy_cosp) * 180.0 / PI;

            if (trackerHeading < 0) trackerHeading += 360.0;
            trackerHeading = 360.0 - trackerHeading;
            // Magnetic declination. Because calibration stores (bearing - heading), any constant
            // offset here cancels out, so this only changes the "Hdg" shown on the OLED.
            trackerHeading += 13.5;
            if (trackerHeading >= 360.0) trackerHeading -= 360.0;
            if (trackerHeading < 0) trackerHeading += 360.0;
        }
    }
}

// =======================================================================================
// BUTTON & CALIBRATION
// =======================================================================================

// --- 5-tap failsafe clear ---------------------------------------------------------------
// Runs from a pin interrupt so taps are counted even while setup() is busy (GPS init,
// WiFi scan, etc.). Quick taps never trigger calibration (that needs a 1-5s hold), so the
// two can't be confused. The actual clear happens at the top of loop(), before the saved
// calibration can be restored, so the servos never snap to the old position.
#define TAP_MAX_PRESS_MS  600    // A press shorter than this counts as a tap
#define TAP_MAX_GAP_MS    1500   // Max time between taps in a sequence
#define TAPS_TO_CLEAR     5
#define BUTTON_DEBOUNCE_MS 30

volatile bool tapClearRequested = false;
volatile uint8_t tapCount = 0;

void IRAM_ATTR buttonISR() {
    static bool pressed = false;
    static unsigned long pressStart = 0;
    static unsigned long lastEdge = 0;
    static unsigned long lastTap = 0;
    unsigned long now = millis();

    if (digitalRead(PIN_RESET_HOME) == LOW) {
        if (!pressed && now - lastEdge > BUTTON_DEBOUNCE_MS) { pressed = true; pressStart = now; }
    } else if (pressed && now - pressStart > BUTTON_DEBOUNCE_MS) {
        pressed = false;
        if (now - pressStart < TAP_MAX_PRESS_MS) {
            tapCount = (now - lastTap < TAP_MAX_GAP_MS) ? tapCount + 1 : 1;
            lastTap = now;
            if (tapCount >= TAPS_TO_CLEAR) { tapClearRequested = true; tapCount = 0; }
        } else {
            tapCount = 0;   // A long press breaks any tap sequence
        }
    }
    lastEdge = now;
}

void checkHomeButton() {
    static unsigned long pressStart = 0;
    static bool isPressed = false;

    if (digitalRead(PIN_RESET_HOME) == LOW) { 
        if (!isPressed) { isPressed = true; pressStart = millis(); }
        if (millis() - pressStart > 5000) {
            display.clearDisplay(); display.setCursor(0,0);
            display.setTextSize(2); display.setTextColor(SSD1306_WHITE); 
            display.println("RELEASE TO"); display.println("  CLEAR"); display.display();
        }
    } else {
        if (isPressed) {
            unsigned long duration = millis() - pressStart;
            isPressed = false;
            
            if (duration > 5000) ClearFailsafe();
            else if (duration > 1000) {
                if (!homeEstablished || boxSats < MIN_SATS) LogScreenPrintln("Tracker GPS", "Need Sats");
                else if (!linkConnected) LogScreenPrintln("No Link", "Cannot Cal");
                else if (droneSats < MIN_SATS) LogScreenPrintln("Drone GPS", "Need Sats");
                else PerformCalibration();
            }
        }
    }
}

void ClearFailsafe() {
    preferences.begin("anttrack", false);
    preferences.remove("offset"); preferences.remove("altOffset");
    preferences.remove("lat"); preferences.remove("lon"); preferences.remove("epoch");
    preferences.remove("homeAlt");
    preferences.end();
    panOffset = 0; altOffset = 0; calibrationDone = false;
    LogScreenPrintln("FAILSAFE", "CLEARED");
    delay(2000);
}

void PerformCalibration() {
    LogScreenPrintln("Calibrating...");
    
    portENTER_CRITICAL(&telemetryMux);
    cur.lat = droneLat; 
    cur.lon = droneLon;
    cur.alt = droneAlt; 
    portEXIT_CRITICAL(&telemetryMux);
    
    getAzEl(hom, cur);

    // NOTE: in both modes the tracker assumes its center is pointing at the aircraft right now.
    // With a compass, later rotation of the tripod is compensated for; without one it isn't.
    if (active_use_compass && compassGood) {
        panOffset = hc_vector.az - trackerHeading;
    } else {
        // No compass (or it failed to start): fall back to pure visual calibration
        trackerHeading = 0; panOffset = hc_vector.az;
    }

    while (panOffset < 0) panOffset += 360;
    while (panOffset >= 360) panOffset -= 360;
    altOffset = cur.alt - hom.alt;

    if (active_use_trim_knob) {
        long total = 0;
        for(int i=0; i<10; i++) { total += analogRead(TRIM_POT_PIN); delay(10); }
        trimOffsetVal = total / 10;
    } else {
        trimOffsetVal = 0; currentTrim = 0;
    }
    
    calibrationDone = true;
    uint32_t currentEpoch = myGNSS.getUnixEpoch(); 
    
    preferences.begin("anttrack", false);
    preferences.putFloat("offset", (float)panOffset);
    preferences.putFloat("altOffset", (float)altOffset);
    preferences.putFloat("homeAlt", hom.alt);   // altOffset is only meaningful paired with this
    preferences.putDouble("lat", hom.lat);
    preferences.putDouble("lon", hom.lon);
    preferences.putUInt("epoch", currentEpoch); 
    preferences.end();
    
    WakeServos();
    LogScreenPrintln("Locked & Zeroed!", "Tracker Ready");
}

void CheckFailsafe() {
    if (!boxGPSFixed) return; 

    preferences.begin("anttrack", true);
    double savedLat = preferences.getDouble("lat", 0);
    double savedLon = preferences.getDouble("lon", 0);
    float savedOffset = preferences.getFloat("offset", 0);
    uint32_t savedEpoch = preferences.getUInt("epoch", 0);
    bool hasAltData = preferences.isKey("homeAlt");
    float savedAltOffset = preferences.getFloat("altOffset", 0);
    float savedHomeAlt = preferences.getFloat("homeAlt", 0);
    preferences.end();
    
    if (savedLat == 0 || savedEpoch == 0) return; 

    uint32_t currentEpoch = myGNSS.getUnixEpoch();
    if (currentEpoch == 0) return; 
    if ((currentEpoch - savedEpoch) > (FAILSAFE_TIMEOUT * 60)) return; 

    struct Location savedLoc; savedLoc.lat = savedLat; savedLoc.lon = savedLon;
    float distToSavedHome = getDist(hom, savedLoc); 

    if (distToSavedHome < 100) { 
        portENTER_CRITICAL(&telemetryMux);
        cur.lat = droneLat; 
        cur.lon = droneLon;
        cur.alt = droneAlt;
        portEXIT_CRITICAL(&telemetryMux);
        
        getAzEl(hom, cur); 
        if (hc_vector.dist > MIN_TRACKING_DIST) { 
            panOffset = savedOffset;
            // Restore the altitude reference too, otherwise tilt is off after a power bump
            if (hasAltData) { hom.alt = savedHomeAlt; altOffset = savedAltOffset; }
            calibrationDone = true;
            WakeServos();
            LogScreenPrintln("FAILSAFE!", "Restored Cal");
            delay(2000);
        }
    }
}

// =======================================================================================
// ESP-NOW (CRSF) LINK
// =======================================================================================
void StartEspNow() {
    usingWiFi = false;
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // The ELRS backpack clears the lowest bit of the first UID byte before using the UID as
    // its MAC address (a MAC with that bit set is a multicast address and is rejected by the
    // ESP32). We must do the same, otherwise any binding phrase whose first UID byte is odd
    // fails silently and the tracker never hears the backpack.
    uint8_t mac[6];
    memcpy(mac, active_mac, 6);
    mac[0] &= ~0x01;
    if (esp_wifi_set_mac(WIFI_IF_STA, mac) != ESP_OK) {
        LogScreenPrintln("MAC FAIL", "Check UID bytes");
        delay(3000);
    }

    if (esp_now_init() != ESP_OK) ESP.restart();
    esp_now_register_recv_cb(OnDataRecv);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}

// =======================================================================================
// SETUP
// =======================================================================================
void setup() {
  Serial.begin(115200);
  delay(100); 
  
  Wire.begin(SDA, SCL);  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println(F("SSD1306 allocation failed"));
  display.clearDisplay(); display.display();
  
  pinMode(PIN_RESET_HOME, INPUT_PULLUP);
  pinMode(StatusLed, OUTPUT); 

  // --- 1. LOAD CONFIGURATION OVERRIDES ---
  preferences.begin("tracker_cfg", true); 
  is_configured = preferences.getBool("configured", false);
  
  if (is_configured) {
      active_link_type = preferences.getUInt("link_type", 2);
      active_ssid = preferences.getString("ssid", ""); 
      active_pass = preferences.getString("pass", "");
      active_port = preferences.getUInt("port", 14550);
      preferences.getBytes("mac", active_mac, 6); 

      active_pan_center = preferences.getUInt("pan_c", 1500);
      active_min_az_pwm = preferences.getUInt("pan_min", 500);
      active_max_az_pwm = preferences.getUInt("pan_max", 2500);
      active_pan_servo_degrees = preferences.getUInt("pan_deg", 270);
      active_max_pan = preferences.getUInt("max_pan", 135); 
      
      active_tilt_horizon = preferences.getUInt("tilt_h", 1500);
      active_min_el_pwm = preferences.getUInt("tilt_min", 1000);
      active_max_el_pwm = preferences.getUInt("tilt_max", 1700);
      active_max_el = preferences.getUInt("max_el", 90);
      
      active_reverse_pan = preferences.getBool("rev_pan", false);
      active_use_compass = preferences.getBool("use_comp", false);
      active_use_trim_knob = preferences.getBool("use_trim", false);
  }
  preferences.end();

  // --- 2. THE "FIRST BOOT" TRAP ---
  // Config mode needs the button held through the first 1.5s of boot, so a stray tap
  // (e.g. starting the 5-tap failsafe clear a bit early) doesn't land you in config mode.
  bool configHeld = false;
  if (digitalRead(PIN_RESET_HOME) == LOW) {
      configHeld = true;
      unsigned long t0 = millis();
      while (millis() - t0 < 1500) {
          if (digitalRead(PIN_RESET_HOME) == HIGH) { configHeld = false; break; }
          delay(10);
      }
  }
  if (!is_configured || configHeld) {
      StartWebConfig();
  }

  // Count quick taps from here on (5 taps = clear saved failsafe calibration)
  attachInterrupt(digitalPinToInterrupt(PIN_RESET_HOME), buttonISR, CHANGE);
  
  // --- 3. INIT COMPASS (If Toggled ON) ---
  if (active_use_compass) {
      pinMode(BNO_INT, INPUT_PULLUP);
      LogScreenPrintln("Compass Init...");
      if (!bno08x.begin_I2C(0x4A, &Wire)) {
          LogScreenPrintln("Compass FAIL", "Check Wires!");
          delay(3000);
      } else {
          bno08x.enableReport(SH2_ROTATION_VECTOR, 50000); 
          compassGood = true;
          LogScreenPrintln("Compass OK!");
          delay(1000);
      }
  } else {
      LogScreenPrintln("Compass OFF", "Visual Cal Mode");
      delay(1500);
  }

  // --- 4. INIT GPS ---
  LogScreenPrintln("GPS Init...");
  gpsSerial.begin(9600, SERIAL_8N1, gps_rxPin, gps_txPin);
  if (myGNSS.begin(gpsSerial) == false) {
      gpsSerial.updateBaudRate(38400);
      if (myGNSS.begin(gpsSerial) == false) {
          gpsSerial.updateBaudRate(115200);
          if (myGNSS.begin(gpsSerial) == false) { LogScreenPrintln("GPS FAIL", "Check Wires"); delay(2000); }
          else { localGpsOk = true; LogScreenPrintln("GPS OK", "115200bd"); }
      } else { localGpsOk = true; LogScreenPrintln("GPS OK", "38400bd"); }
  } else { localGpsOk = true; LogScreenPrintln("GPS OK", "9600bd"); }

  if (localGpsOk) {
      // UBX only on the UART: NMEA sentences at 5Hz would saturate a 9600 baud link
      myGNSS.setUART1Output(COM_TYPE_UBX);
      myGNSS.setNavigationFrequency(5);
      // Have the GPS push position reports on its own. Without this, every getXXX() call
      // sends a poll and blocks the main loop until the GPS answers (~100ms+ per loop).
      myGNSS.setAutoPVT(true);
  }

  // --- 5. TELEMETRY LINK INITIALIZATION ---
  if (active_link_type == 1 || active_link_type == 2) {
      LogScreenPrintln("Scanning WiFi...");
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);   // Modem sleep adds latency and drops UDP packets
      WiFi.begin(active_ssid.c_str(), active_pass.c_str());

      unsigned long startAttempt = millis();
      int timeout = (active_link_type == 1) ? 30000 : 12000;
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < timeout) delay(100);

      if (WiFi.status() == WL_CONNECTED) {
          usingWiFi = true;
          udp.begin(active_port);
          udpStarted = true;
          String ipStr = WiFi.localIP().toString();
          LogScreenPrintln("WiFi UDP Lock", ipStr);
      } else {
          if (active_link_type == 2) {
              StartEspNow();
              LogScreenPrintln("CRSF Fallback", "ESP-NOW");
          } else {
              // WiFi-only mode: stay in WiFi mode and keep trying in the background.
              // UDP is started from loop() as soon as the backpack AP shows up.
              usingWiFi = true;
              LogScreenPrintln("WiFi Timeout", "Still trying...");
          }
      }
  } else if (active_link_type == 0) {
      StartEspNow();
      LogScreenPrintln("CRSF Locked", "ESP-NOW");
  }
  delay(1500);
  
  // --- FINISH BOOT ---
  if (active_use_trim_knob) {
      LogScreenPrintln("Zeroing Knob..."); delay(500); 
      long total = 0;
      for(int i=0; i<10; i++) { total += analogRead(TRIM_POT_PIN); delay(10); }
      trimOffsetVal = total / 10; 
  }
  
  LogScreenPrintln("Ready!"); delay(500);
}

// =======================================================================================
// MAIN LOOP
// =======================================================================================
void loop() {
  // 5 quick taps on the button (any time, even during boot) wipes the saved calibration
  if (tapClearRequested) { tapClearRequested = false; ClearFailsafe(); }

  ReadCompass();
  ReadLocalGPS();

  // Signed math: the ESP-NOW callback runs on the other core and may update lastPacketTime
  // between our two reads; unsigned subtraction would then wrap and look like a timeout.
  unsigned long nowMs = millis();
  long sinceLastPacket = (long)(nowMs - lastPacketTime);

  if (usingWiFi) {
      // (Re)open the UDP socket whenever the backpack WiFi (re)connects
      bool wifiUp = (WiFi.status() == WL_CONNECTED);
      if (wifiUp && !udpStarted) { udp.stop(); udp.begin(active_port); udpStarted = true; }
      else if (!wifiUp && udpStarted) { udpStarted = false; }

      if (udpStarted) readUDP();
      if (sinceLastPacket > 2000) { channelLocked = false; linkConnected = false; }
  } else {
      if (sinceLastPacket > 2000) {
        channelLocked = false; linkConnected = false; 
        currentChannel++; if (currentChannel > 13) currentChannel = 1;
        esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
        delay(10); 
      }
  }

  checkHomeButton();

  if (linkConnected) {
      portENTER_CRITICAL(&telemetryMux);
      cur.lat = droneLat; 
      cur.lon = droneLon; 
      cur.alt = droneAlt;
      portEXIT_CRITICAL(&telemetryMux);
      
      cur.alt_ag = cur.alt - hom.alt - altOffset;   // Height above the calibrated ground level
      gpsGood = true;       
  }

  if (homeEstablished && calibrationDone && linkConnected) {
      getAzEl(hom, cur); 
      
      float targetAzBase = hc_vector.az - trackerHeading - panOffset;
      while (targetAzBase < 0) targetAzBase += 360;
      while (targetAzBase > 360) targetAzBase -= 360;
      
      if (active_use_trim_knob) {
          int rawKnob = analogRead(TRIM_POT_PIN);
          int knobDiff = rawKnob - trimOffsetVal; 
          currentTrim = knobDiff / 30;
          if (currentTrim < -MAX_TRIM_ANGLE) currentTrim = -MAX_TRIM_ANGLE;
          if (currentTrim > MAX_TRIM_ANGLE)  currentTrim = MAX_TRIM_ANGLE;
      } else {
          currentTrim = 0; 
      }

      static unsigned long debugTimer = 0;
      if (millis() - debugTimer > 200) { 
         debugTimer = millis(); UpdateDisplay(0, currentTrim); 
      }

      if ( (hc_vector.dist >= MIN_TRACKING_DIST) || ((int)cur.alt_ag >= MIN_TRACKING_ALT) ) {
          long finalAzLong = (long)targetAzBase + currentTrim;
          while (finalAzLong < 0) finalAzLong += 360;
          while (finalAzLong >= 360) finalAzLong -= 360;
          
          int tiltAngle = hc_vector.el; 
          if (tiltAngle < 0) tiltAngle = 0;
          if (tiltAngle > active_max_el) tiltAngle = active_max_el; 
          
          static uint16_t lastValidAz = finalAzLong;
          
          int deadbandThreshold = active_max_el - 5;
          if (tiltAngle > deadbandThreshold) {
              pointServos(lastValidAz, tiltAngle);
          } else {
              lastValidAz = finalAzLong;
              pointServos((uint16_t)finalAzLong, tiltAngle);
          }
      }
  } 
  else if (homeEstablished && calibrationDone && !linkConnected) {
     static unsigned long warnTimer = 0;
     if (millis() - warnTimer > 1000) {
        warnTimer = millis();
        if (usingWiFi) LogScreenPrintln("LINK LOST", WiFi.localIP().toString());
        else LogScreenPrintln("LINK LOST", "ESP-NOW");
     }
  }
  else if (!homeEstablished || boxSats < MIN_SATS) {
     static unsigned long warnTimer = 0;
     if (millis() - warnTimer > 2000) {
        warnTimer = millis();
        LogScreenPrintln("Tracker: " + String(boxSats) + "/" + String(MIN_SATS)); 
     }
  }
  else if (!calibrationDone) {
     // Check for a saved calibration once a second (it reads flash + GPS time)
     static unsigned long failsafeTimer = 0;
     if (linkConnected && millis() - failsafeTimer > 1000) { failsafeTimer = millis(); CheckFailsafe(); }
     static unsigned long warnTimer = 0;
     if (millis() - warnTimer > 1000) { 
        warnTimer = millis();
        if (!linkConnected) {
            if (active_link_type == 1 && WiFi.status() != WL_CONNECTED) LogScreenPrintln("No Link", "WiFi Disconnected");
            else if (usingWiFi) LogScreenPrintln("No Link", WiFi.localIP().toString());
            else LogScreenPrintln("No Link", "ESP-NOW");
        }
        else if (droneSats < MIN_SATS) LogScreenPrintln("Drone: " + String(droneSats) + "/" + String(MIN_SATS)); 
        else LogScreenPrintln("Ready", "Hold 1 sec");
     }
  }
  
  ServiceTheStatusLed();
} 

// =======================================================================================
// MATH & NAVIGATION
// =======================================================================================
float getDist(struct Location &a, struct Location &b) {
  double dLon = (b.lon - a.lon) * PI / 180.0;
  double lat1 = a.lat * PI / 180.0;
  double lat2 = b.lat * PI / 180.0;
  double dLat = (b.lat - a.lat) * PI / 180.0;
  double x = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
  double c = 2 * atan2(sqrt(x), sqrt(1-x));
  return 6371000.0 * c; 
}

void getAzEl(struct Location &hom, struct Location &cur) {
  double dLon = (cur.lon - hom.lon) * PI / 180.0;
  double lat1 = hom.lat * PI / 180.0;
  double lat2 = cur.lat * PI / 180.0;

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double az = atan2(y, x) * 180.0 / PI;
  if (az < 0) az += 360;
  hc_vector.az = az;

  double dLat = (cur.lat - hom.lat) * PI / 180.0;
  double a = sin(dLat/2) * sin(dLat/2) + sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  hc_vector.dist = 6371000 * c; 

  float rawDiff = cur.alt - hom.alt;
  float diff = rawDiff - altOffset;
  if (diff < 0) diff = 0; 
  hc_vector.el = atan2(diff, hc_vector.dist) * 180.0 / PI;
}

void pointServos(uint16_t az, uint16_t el) {
  int targetAz = az; 

  float deviation = targetAz;
  if (deviation > 180) deviation -= 360; 

  if (active_reverse_pan) {
      deviation = -deviation;
  }

  deviation = constrain(deviation, -active_max_pan, active_max_pan);

  float usPerDegree = abs(active_max_az_pwm - active_min_az_pwm) / (float)max(active_pan_servo_degrees, 1);
  int targetPanPWM = active_pan_center + (int)(deviation * usPerDegree);
  
  targetPanPWM = constrain(targetPanPWM, min(active_min_az_pwm, active_max_az_pwm), max(active_min_az_pwm, active_max_az_pwm));

  if (el > active_max_el) el = active_max_el;

  // Tilt scale comes from the web UI: Horizon PWM = 0 deg, "Tilt Up PWM" = 90 deg (straight up).
  // (Previously hard-coded to 1000us = 90 deg and clamped to 1000-1600us, ignoring the settings.)
  // If Up PWM is higher than Horizon the mapping simply runs the other way (reversed servo).
  int targetTiltPWM = map(el, 0, 90, active_tilt_horizon, active_min_el_pwm);
  targetTiltPWM = constrain(targetTiltPWM, min(active_min_el_pwm, active_max_el_pwm), max(active_min_el_pwm, active_max_el_pwm));

  // currentPanPWM / currentTiltPWM are globals, seeded in WakeServos()
  static unsigned long lastMoveTime = 0;

  if (millis() - lastMoveTime > 20) {
      lastMoveTime = millis();
      currentPanPWM += (targetPanPWM - currentPanPWM) * SERVO_SPEED;
      currentTiltPWM += (targetTiltPWM - currentTiltPWM) * SERVO_SPEED;
      azServo.writeMicroseconds((int)currentPanPWM);
      elServo.writeMicroseconds((int)currentTiltPWM);
  }
}

// =======================================================================================
// STATUS LED HANDLERS
// =======================================================================================
void ServiceTheStatusLed() {
  if (linkConnected) {
    if (calibrationDone) digitalWrite(StatusLed, HIGH); 
    else BlinkLed(100); 
  } else BlinkLed(1000);     
}

void BlinkLed(uint16_t period) {
  uint32_t cMillis = millis();
  if (cMillis - millisLED >= period) {
    millisLED = cMillis; ledState = !ledState;
    digitalWrite(StatusLed, ledState);
  }
}