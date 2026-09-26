#ifndef CONFIG_H
#define CONFIG_H

#include <ESP32Servo.h>
#include <ESP32PWM.h>

//================================================================================================= 
//                              C O N F I G U R A T I O N 
//================================================================================================= 

#define MAJOR_VERSION       2
#define MINOR_VERSION      20
#define PATCH_LEVEL        16 // Open Source Release (Hardware Toggles)

//================================================================================================= 
//                              WIRELESS SETTINGS 
//================================================================================================= 
#define WIFI_SSID "ExpressLRS TX Backpack 21D5E4" // Exactly as it appears on your laptop
#define WIFI_PASS "expresslrs"
#define UDP_PORT 14550

//================================   E L R S   B I N D I N G   ===============================
// CRITICAL: Replace this with your transmitter's ExpressLRS UID MAC Address!
const uint8_t BINDING_MAC[] = {0, 0, 0, 0, 0, 0};


//================================   H A R D W A R E   O P T I O N S   =======================
#define USE_COMPASS   true  // Set to false if you are NOT using a BNO085 compass.
                            // (Requires manual visual calibration before launch)
                            
#define USE_TRIM_KNOB true  // Set to false if you don't have a 10k potentiometer for live pan trim.

//================================   G P S   S E T T I N G S   ===============================
#define MIN_SATS 8          // Minimum GPS satellites required to allow calibration
#define FAILSAFE_TIMEOUT 180 // Minutes before the saved calibration memory expires (180 = 3 hours)

//================================   T R A C K I N G   T U N I N G   =========================
#define MIN_TRACKING_DIST 2   // Meters drone must be away before Pan servo activates
#define MIN_TRACKING_ALT  2   // Meters drone must be high before Tilt servo activates
#define SERVO_SPEED       0.3 // Servo smoothing: 0.05 = Slow/Smooth, 0.30 = Fast/Responsive

//================================   S E R V O   C E N T E R I N G   =========================
// Adjust these to match the physical center of your specific hardware build. Recommend obtaining these valuses using a servo tester
#define PAN_CENTER_PWM   1480 // Microseconds where your pan servo faces exactly forward
#define TILT_HORIZON_PWM 1550 // Microseconds where your tilt servo is perfectly level

//================================   S E R V O   S E T T I N G S   ===========================
#define REVERSE_PAN     true   // Reverses pan direction logic
#define SERVO_CENTER    90     // Theoretical center for 180/270 math
#define PAN_TRIM        0      // Digital offset (Live trim knob handles this now)

// Start positions (0 = Front / Horizon)
#define azStart  0       
#define elStart  0      

// Elevation Safety Limits
int16_t minEl = 0;          
int16_t maxEl = 70;        // Safety wall: Prevents tilt servo from binding against mount

// Physical PWM Limits for 270 Degree Servos. Recommend getting these values using servo tester.
uint16_t minAzPWM = 500;   
uint16_t maxAzPWM = 2500;  
uint16_t minElPWM = 1000;   // Vertical (Up)
uint16_t maxElPWM = 1700;   // Down (Ground)

//================================   L I V E   T R I M   =====================================
// (Ignored if USE_TRIM_KNOB is false)
#define TRIM_POT_PIN    35  // Analog Pin for 10k Potentiometer
#define MAX_TRIM_ANGLE  20  // Max degrees to adjust left/right via knob

//============================================================================================= 
//======================= H A R D W A R E   P I N O U T S =====================================
//=============================================================================================

// --- I2C BUS (OLED DISPLAY & BNO085 COMPASS) ---
#define SDA 21             // I2C Data
#define SCL 22             // I2C Clock

#define BNO_INT 27         // Compass Interrupt Pin (Ignored if USE_COMPASS is false)
#define BNO_RST 26         // Compass Reset Pin (Ignored if USE_COMPASS is false)

#define OLED_WIDTH  128
#define OLED_HEIGHT 64

// --- GPS PINS ---
int8_t gps_rxPin = 18;     // Connected to TX on the GPS module
#define gps_txPin  19      // Connected to RX on the GPS module
   
// --- BUTTONS & LEDS ---
#define PIN_RESET_HOME  4  // Button to clear failsafe/calibrate
#define StatusLed      25  // Blinks when waiting for sats/link, solid when locked
   
// --- SERVOS ---
#define azPWM_Pin      14  // Pan Servo Data Pin
#define elPWM_Pin      13  // Tilt Servo Data Pin
   
#endif
