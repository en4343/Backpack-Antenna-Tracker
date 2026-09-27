/*================================================================================================= 
    Antenna Tracker - "Community Edition" (Standalone Web UI / Dynamic Link Toggles / Help Text)
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
#define FAILSAFE_TIMEOUT 180 
#define MIN_TRACKING_DIST 2  
#define MIN_TRACKING_ALT 2   
#define SERVO_SPEED 0.3      
#define MAX_TRIM_ANGLE 20    

#ifndef CRSF_FRAMETYPE_GPS
  #define CRSF_FRAMETYPE_GPS 0x02
#endif

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
unsigned long lastPacketTime = 0;
int currentChannel = 1;
bool channelLocked = false;
bool usingWiFi = false;

// --- Ground Station (Box) Data ---
double boxLat = 0;
double boxLon = 0;
float boxAlt = 0;
bool boxGPSFixed = false;
int boxSats = 0;
float trackerHeading = 0;
bool compassGood = false;
bool gpsGood = false;

// --- Calibration & Tracking State ---
int panOffset = 0; 
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
    <option value="2" V_LINK2>Auto-Detect (Try WiFi MAVLink, fallback to ESP-NOW CRSF)</option>
    <option value="0" V_LINK0>ESP-NOW / CRSF Only (Fastest Boot)</option>
    <option value="1" V_LINK1>WiFi UDP / MAVLink Only</option>
  </select>

  <div id="wifi_settings">
    <div class="row">
      <div class="col">
        <label>WiFi SSID <span class="req">*</span></label>
        <span class="help">Your ELRS TX Backpack Wi-Fi Name</span>
        <input type="text" name="ssid" id="ssid" value="V_SSID">
      </div>
      <div class="col">
        <label>WiFi Password <span class="req">*</span></label>
        <span class="help">Your Backpack Password</span>
        <input type="text" name="pass" id="pass" value="V_PASS">
      </div>
    </div>
    <label>UDP Listen Port <span class="req">*</span></label>
    <span class="help">Mission Planner relay port (default is usually 14550)</span>
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
      <input type="number" name="max_pan" value="V_MAXPAN" required>
    </div>
  </div>

  <h2>Servo Tuning (Tilt Axis)</h2>
  <label>Tilt Horizon PWM (Level) <span class="req">*</span></label>
  <span class="help">PWM value when patch antennas point perfectly level</span>
  <input type="number" name="tilt_h" value="V_TILTH" required>

  <div class="row">
    <div class="col">
      <label>Tilt Min PWM (Up) <span class="req">*</span></label>
      <span class="help">PWM for maximum upward tilt</span>
      <input type="number" name="tilt_min" value="V_TILTMIN" required>
    </div>
    <div class="col">
      <label>Tilt Max PWM (Down) <span class="req">*</span></label>
      <span class="help">PWM for maximum downward tilt</span>
      <input type="number" name="tilt_max" value="V_TILTMAX" required>
    </div>
  </div>

  <label>Max Tilt Angle Limit <span class="req">*</span></label>
  <span class="help">Safety limit in degrees (e.g., 90 for straight up)</span>
  <input type="number" name="max_el" value="V_MAXEL" required>

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

void parseMacString(String macStr, uint8_t* macArr) {
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

        html.replace("V_SSID", active_ssid);
        html.replace("V_PASS", active_pass);
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
        
        int p_deg = server.arg("pan_deg").toInt();
        if (p_deg > 270) p_deg = 270; 
        preferences.putUInt("pan_deg", p_deg);
        
        preferences.putUInt("max_pan", server.arg("max_pan").toInt());
        preferences.putUInt("tilt_h", server.arg("tilt_h").toInt());
        preferences.putUInt("tilt_min", server.arg("tilt_min").toInt());
        preferences.putUInt("tilt_max", server.arg("tilt_max").toInt());
        preferences.putUInt("max_el", server.arg("max_el").toInt());
        
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
        azServo.writeMicroseconds(active_pan_center);
        elServo.writeMicroseconds(active_tilt_horizon);
        azServo.attach(azPWM_Pin, active_min_az_pwm, active_max_az_pwm); 
        elServo.attach(elPWM_Pin, active_min_el_pwm, active_max_el_pwm);
        servosAwake = true;
    }
}

// =======================================================================================
// TELEMETRY PARSERS (Universal CRSF & MAVLink)
// =======================================================================================
void parseCRSF(const uint8_t *data, int data_len) {
  int offset = -1;
  for (int i = 0; i < 15; i++) {
      if (i < data_len && data[i] == CRSF_FRAMETYPE_GPS && data_len >= i + 16) { 
          offset = i; break; 
      }
  }

  if (offset != -1) {
    uint32_t latBytes = ((uint32_t)data[offset + 1] << 24) | ((uint32_t)data[offset + 2] << 16) | ((uint32_t)data[offset + 3] << 8) | (uint32_t)data[offset + 4];
    uint32_t lonBytes = ((uint32_t)data[offset + 5] << 24) | ((uint32_t)data[offset + 6] << 16) | ((uint32_t)data[offset + 7] << 8) | (uint32_t)data[offset + 8];
    uint16_t altitudeRaw = (data[offset + 13] << 8) | data[offset + 14];
    int altitude1 = altitudeRaw - 1000;
    
    portENTER_CRITICAL(&telemetryMux);
    droneLat = (double)((int32_t)latBytes) / 10000000.0;
    droneLon = (double)((int32_t)lonBytes) / 10000000.0;
    droneAlt = (altitude1 > 32767 || altitude1 < -32768) ? (altitude1 - 4294967296) : altitude1;
    droneSats = data[offset + 15];
    portEXIT_CRITICAL(&telemetryMux);
  }
}

void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *data, int data_len) {
    lastPacketTime = millis();
    linkConnected = true;
    if (!channelLocked) { channelLocked = true; Serial.println("ESP-NOW LINK OK!"); }
    parseCRSF(data, data_len);
}

void parseMavlink(uint8_t c) {
    static uint8_t state = 0;
    static uint8_t payloadLen = 0;
    static uint8_t msgId = 0;
    static uint8_t payload[30];
    static uint8_t payloadIdx = 0;

    switch (state) {
        case 0: if (c == 0xFD) state = 1; break; 
        case 1: payloadLen = c; state = 2; break; 
        case 2: case 3: case 4: case 5: case 6: state++; break; 
        case 7: msgId = c; state = 8; break; 
        case 8: if (c == 0) state = 9; else state = 0; break; 
        case 9: if (c == 0 && msgId == 33) { payloadIdx = 0; state = 10; } else state = 0; break; 
        case 10:
            if (payloadIdx < payloadLen && payloadIdx < 30) payload[payloadIdx++] = c;
            if (payloadIdx == payloadLen) { 
                if (payloadLen >= 16) {
                    uint32_t rawLat = ((uint32_t)payload[7] << 24) | ((uint32_t)payload[6] << 16) | ((uint32_t)payload[5] << 8) | (uint32_t)payload[4];
                    uint32_t rawLon = ((uint32_t)payload[11] << 24) | ((uint32_t)payload[10] << 16) | ((uint32_t)payload[9] << 8) | (uint32_t)payload[8];
                    uint32_t rawAlt = ((uint32_t)payload[15] << 24) | ((uint32_t)payload[14] << 16) | ((uint32_t)payload[13] << 8) | (uint32_t)payload[12]; 

                    int32_t latInt = (int32_t)rawLat;
                    int32_t lonInt = (int32_t)rawLon;

                    if (latInt != 0 && lonInt != 0) { 
                        portENTER_CRITICAL(&telemetryMux);
                        droneLat = (double)latInt / 10000000.0;
                        droneLon = (double)lonInt / 10000000.0;
                        droneAlt = ((int32_t)rawAlt) / 1000.0;
                        droneSats = 15; 
                        portEXIT_CRITICAL(&telemetryMux);
                    }
                }
                state = 0;
            }
            break;
    }
}

void readUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize) {
        lastPacketTime = millis();
        linkConnected = true;
        if (!channelLocked) { channelLocked = true; Serial.println("WIFI UDP LINK OK!"); }
        
        uint8_t buffer[512];
        int len = udp.read(buffer, sizeof(buffer));
        if (len > 0) {
            for (int i = 0; i < len; i++) parseMavlink(buffer[i]);
        }
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
          boxAlt = myGNSS.getAltitude() / 1000.0; 
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
            trackerHeading += 13.5; 
            if (trackerHeading >= 360.0) trackerHeading -= 360.0;
            if (trackerHeading < 0) trackerHeading += 360.0;
        }
    }
}

// =======================================================================================
// BUTTON & CALIBRATION
// =======================================================================================
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
    preferences.end();
    panOffset = 0; calibrationDone = false;
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
    
    if (active_use_compass) {
        panOffset = hc_vector.az - trackerHeading;
    } else {
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
            panOffset = (int)savedOffset;
            calibrationDone = true;
            WakeServos();
            LogScreenPrintln("FAILSAFE!", "Restored Cal");
            delay(2000);
        }
    }
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
  if (!is_configured || digitalRead(PIN_RESET_HOME) == LOW) {
      StartWebConfig(); 
  }
  
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
          else { LogScreenPrintln("GPS OK", "115200bd"); }
      } else { LogScreenPrintln("GPS OK", "38400bd"); }
  } else { LogScreenPrintln("GPS OK", "9600bd"); }

  myGNSS.setI2COutput(COM_TYPE_UBX); 
  myGNSS.setNavigationFrequency(5);  
  
  // --- 5. TELEMETRY LINK INITIALIZATION ---
  if (active_link_type == 1 || active_link_type == 2) {
      LogScreenPrintln("Scanning WiFi...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(active_ssid.c_str(), active_pass.c_str());
      
      unsigned long startAttempt = millis();
      int timeout = (active_link_type == 1) ? 30000 : 12000;
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < timeout) delay(100);

      if (WiFi.status() == WL_CONNECTED) {
          usingWiFi = true;
          udp.begin(active_port);
          String ipStr = WiFi.localIP().toString();
          LogScreenPrintln("WiFi UDP Lock", ipStr);
      } else {
          if (active_link_type == 2) {
              usingWiFi = false;
              WiFi.disconnect();
              esp_wifi_set_mac(WIFI_IF_STA, active_mac);
              if (esp_now_init() != ESP_OK) ESP.restart();
              esp_now_register_recv_cb(OnDataRecv);
              esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
              LogScreenPrintln("CRSF Fallback", "ESP-NOW");
          } else {
              LogScreenPrintln("WiFi Timeout", "Check AP/Pass");
          }
      }
  } else if (active_link_type == 0) {
      usingWiFi = false;
      WiFi.mode(WIFI_STA);
      WiFi.disconnect();
      esp_wifi_set_mac(WIFI_IF_STA, active_mac);
      if (esp_now_init() != ESP_OK) ESP.restart();
      esp_now_register_recv_cb(OnDataRecv);
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
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
  ReadCompass();
  ReadLocalGPS();

  if (usingWiFi) {
      readUDP();
      if (millis() - lastPacketTime > 2000) { channelLocked = false; linkConnected = false; }
  } else {
      if (millis() - lastPacketTime > 2000) {
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
      
      cur.alt_ag = cur.alt - hom.alt; 
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
     if (linkConnected) CheckFailsafe(); 
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

  float usPerDegree = (active_max_az_pwm - active_min_az_pwm) / (float)active_pan_servo_degrees;
  int targetPanPWM = active_pan_center + (int)(deviation * usPerDegree);
  
  targetPanPWM = constrain(targetPanPWM, active_min_az_pwm, active_max_az_pwm);

  if (el < 0) el = 0;
  if (el > active_max_el) el = active_max_el; 
  
  int targetTiltPWM = map(el, 0, 90, active_tilt_horizon, 1000); 
  targetTiltPWM = constrain(targetTiltPWM, 1000, 1600);

  static float currentPanPWM = targetPanPWM;   
  static float currentTiltPWM = targetTiltPWM; 
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