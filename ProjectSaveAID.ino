#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "MAX30105.h"
#include "heartRate.h"

// ---------------- ADXL345 ----------------
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ---------------- MAX30105 ----------------
MAX30105 particleSensor;

// --- Threshold Settings ---
const float FALL_THRESHOLD_LOW = 0.5;   // Free-fall (g)
const float FALL_THRESHOLD_HIGH = 2.5;  // Impact (g)
const int HR_LOW_LIMIT = 45;
const int HR_HIGH_LIMIT = 130;
bool freeFallDetected = false;
unsigned long freeFallTime = 0;

const unsigned long FALL_WINDOW = 600; // ms


// --- Heart Rate Variables ---
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

const int buttonPin = 2;
// ---- Heart Rate Emergency Timing ----
const unsigned long HR_CONFIRM_TIME = 13000; // 7 seconds

bool hrAbnormal = false;
unsigned long hrAbnormalStart = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);  // To ESP8266

  pinMode(buttonPin, INPUT_PULLUP);
  Wire.begin();

  // Initialize ADXL345
  if (!accel.begin()) {
    Serial.println("ADXL345 not detected!");
    while (1);
  }
 
  accel.setRange(ADXL345_RANGE_16_G);  // Needed for fall impact

  // Initialize MAX30105
  particleSensor.begin(Wire, I2C_SPEED_FAST);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0);
  particleSensor.setPulseAmplitudeIR(0x1F);

  Serial.println("System initialized.");
}

void triggerEmergency(String reason) {
  Serial.println("!!! EMERGENCY: " + reason + " !!!");
  Serial1.println("EMERGENCY_ALERT");
  delay(2000); // prevent spamming
}

void loop() {
  // ---------- 1. FALL DETECTION (ADXL345) ----------
  // ---------- FALL DETECTION (IMPROVED) ----------
sensors_event_t event;
accel.getEvent(&event);

// Convert to G
float ax = event.acceleration.x / 9.81;
float ay = event.acceleration.y / 9.81;
float az = event.acceleration.z / 9.81;

float totalAcc = sqrt(ax * ax + ay * ay + az * az);

// Stage 1: Detect free fall (low G)
if (totalAcc < FALL_THRESHOLD_LOW && !freeFallDetected) {
  freeFallDetected = true;
  freeFallTime = millis();
}

// Stage 2: Detect impact shortly after free fall
if (freeFallDetected) {
  if (totalAcc > FALL_THRESHOLD_HIGH &&
      millis() - freeFallTime < FALL_WINDOW) {

    triggerEmergency("FALL_DETECTED");
    freeFallDetected = false;
  }

  // Timeout → cancel
  if (millis() - freeFallTime > FALL_WINDOW) {
    freeFallDetected = false;
  }
}


  // ---------- 2. HEART RATE ----------
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 220 && beatsPerMinute > 30) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte i = 0; i < RATE_SIZE; i++) beatAvg += rates[i];
      beatAvg /= RATE_SIZE;

      // ---- Abnormal heart rate detection with delay ----
      if (beatAvg > HR_HIGH_LIMIT || beatAvg < HR_LOW_LIMIT) {
        if (!hrAbnormal) {
          hrAbnormal = true;
          hrAbnormalStart = millis();  // start timer
        } else if (millis() - hrAbnormalStart >= HR_CONFIRM_TIME) {
          triggerEmergency("ABNORMAL_HEART_RATE");
          hrAbnormal = false;  // reset after emergency
        }
      } else {
        // Heart rate back to normal
        hrAbnormal = false;
      }
    }
  }


  // ---------- 3. SOS BUTTON ----------
  if (digitalRead(buttonPin) == LOW) {
    triggerEmergency("SOS_BUTTON_PRESSED");
  }

  // ---------- LOGGING ----------
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 500) {
    Serial.print("Acc (g): ");
    Serial.print(totalAcc);
    Serial.print(" | Avg BPM: ");
    Serial.println(beatAvg);
    lastLog = millis();
  }
}
