// parking_assist.ino
// Ultrasonic parking assist / collision warning system for Arduino Uno.
//
// - HC-SR04 distance sensing with a 5-sample median filter
// - 4-zone state machine (CLEAR / CAUTION / WARNING / DANGER) with hysteresis
// - LED bar + buzzer whose beep rate rises as the obstacle gets closer
// - Debounced mute button (auto-unmutes once the path is clear again)
// - Fully non-blocking main loop based on millis(), no delay()
// - CSV telemetry over serial for plotting / logging

#include "distance_logic.h"

// ---------------- Pin map ----------------
const uint8_t PIN_TRIG   = 9;
const uint8_t PIN_ECHO   = 10;
const uint8_t PIN_LED_G  = 4;
const uint8_t PIN_LED_Y  = 5;
const uint8_t PIN_LED_R  = 6;
const uint8_t PIN_BUZZER = 8;
const uint8_t PIN_BUTTON = 2;  // to GND, uses INPUT_PULLUP

// ---------------- Tuning ----------------
const pa::Thresholds THRESH = {
  /*caution_cm=*/100,
  /*warning_cm=*/50,
  /*danger_cm=*/20,
  /*hysteresis_cm=*/5
};
const uint16_t MEASURE_PERIOD_MS = 60;    // HC-SR04 datasheet: >= 60 ms between pings
const uint32_t ECHO_TIMEOUT_US   = 25000; // ~4.3 m max range
const uint16_t BEEP_ON_MS        = 50;
const uint16_t BUZZER_FREQ_HZ    = 2000;
const uint16_t DEBOUNCE_MS       = 30;

// ---------------- State ----------------
pa::MedianFilter5 filter;
pa::Zone zone = pa::ZONE_CLEAR;
uint16_t filteredCm = pa::MAX_RANGE_CM;
bool muted = false;

uint32_t lastMeasureMs = 0;
uint32_t lastBeepStartMs = 0;
bool buzzerOn = false;

bool buttonStable = HIGH;
bool buttonLastRead = HIGH;
uint32_t buttonChangedMs = 0;

// ---------------- Sensor ----------------
uint32_t readEchoUs() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);  // 10 us trigger pulse
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
}

// ---------------- Outputs ----------------
void updateLeds(pa::Zone z) {
  digitalWrite(PIN_LED_G, z == pa::ZONE_CLEAR || z == pa::ZONE_CAUTION);
  digitalWrite(PIN_LED_Y, z == pa::ZONE_CAUTION || z == pa::ZONE_WARNING);
  digitalWrite(PIN_LED_R, z == pa::ZONE_WARNING || z == pa::ZONE_DANGER);
}

void setBuzzer(bool on) {
  if (on == buzzerOn) return;
  buzzerOn = on;
  if (on) tone(PIN_BUZZER, BUZZER_FREQ_HZ);
  else noTone(PIN_BUZZER);
}

void updateBuzzer(uint32_t now) {
  pa::BuzzerPattern p = pa::buzzerPattern(filteredCm, zone, THRESH);
  if (muted || !p.enabled) { setBuzzer(false); return; }
  if (p.continuous)        { setBuzzer(true);  return; }

  uint32_t elapsed = now - lastBeepStartMs;  // unsigned math survives millis() rollover
  if (elapsed >= p.period_ms) {
    lastBeepStartMs = now;
    elapsed = 0;
  }
  uint16_t onTime = BEEP_ON_MS < p.period_ms / 2 ? BEEP_ON_MS : p.period_ms / 2;
  setBuzzer(elapsed < onTime);
}

// ---------------- Input ----------------
void updateButton(uint32_t now) {
  bool reading = digitalRead(PIN_BUTTON);
  if (reading != buttonLastRead) {
    buttonLastRead = reading;
    buttonChangedMs = now;
  }
  if (now - buttonChangedMs >= DEBOUNCE_MS && reading != buttonStable) {
    buttonStable = reading;
    if (buttonStable == LOW) {  // pressed
      muted = !muted;
      Serial.print(F("# mute=")); Serial.println(muted);
    }
  }
}

// ---------------- Main ----------------
void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println(F("# Parking assist ready"));
  Serial.println(F("ms,raw_cm,filtered_cm,zone,muted"));
  updateLeds(zone);
}

void loop() {
  uint32_t now = millis();
  updateButton(now);

  if (now - lastMeasureMs >= MEASURE_PERIOD_MS) {
    lastMeasureMs = now;

    uint16_t rawCm = pa::echoToCm(readEchoUs());
    filter.push(rawCm);
    filteredCm = filter.value();

    pa::Zone next = pa::classify(filteredCm, zone, THRESH);
    if (next != zone) {
      zone = next;
      updateLeds(zone);
      if (zone == pa::ZONE_CLEAR) muted = false;  // safety: re-arm alarm when clear
    }

    Serial.print(now);        Serial.print(',');
    Serial.print(rawCm);      Serial.print(',');
    Serial.print(filteredCm); Serial.print(',');
    Serial.print(pa::zoneName(zone)); Serial.print(',');
    Serial.println(muted);
  }

  updateBuzzer(now);
}
