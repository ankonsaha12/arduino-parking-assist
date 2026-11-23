// distance_logic.h
// Hardware-independent logic for the parking assist system.
// Kept free of Arduino APIs so it can be unit-tested on a PC with g++.

#pragma once
#include <stdint.h>

namespace pa {

const uint16_t MAX_RANGE_CM = 400;  // HC-SR04 practical limit

enum Zone : uint8_t {
  ZONE_CLEAR = 0,
  ZONE_CAUTION = 1,
  ZONE_WARNING = 2,
  ZONE_DANGER = 3
};

struct Thresholds {
  uint16_t caution_cm;     // closer than this -> CAUTION
  uint16_t warning_cm;     // closer than this -> WARNING
  uint16_t danger_cm;      // closer than this -> DANGER
  uint16_t hysteresis_cm;  // extra distance needed before moving to a safer zone
};

inline const char* zoneName(Zone z) {
  switch (z) {
    case ZONE_CLEAR:   return "CLEAR";
    case ZONE_CAUTION: return "CAUTION";
    case ZONE_WARNING: return "WARNING";
    default:           return "DANGER";
  }
}

// Converts an HC-SR04 echo pulse width to centimetres.
// Sound travels ~0.0343 cm/us and the pulse covers the round trip, so cm = us / 58.
// A pulse of 0 means pulseIn() timed out (no echo): treat it as "nothing in range".
inline uint16_t echoToCm(uint32_t echo_us) {
  if (echo_us == 0) return MAX_RANGE_CM;
  uint32_t cm = echo_us / 58;
  return cm > MAX_RANGE_CM ? MAX_RANGE_CM : static_cast<uint16_t>(cm);
}

// Median of the last 5 samples. Rejects single-sample spikes that are common
// with ultrasonic sensors (stray echoes, soft surfaces) without lagging like
// a large moving average would.
class MedianFilter5 {
 public:
  void push(uint16_t v) {
    buf_[idx_] = v;
    idx_ = static_cast<uint8_t>((idx_ + 1) % N);
    if (count_ < N) count_++;
  }

  uint8_t count() const { return count_; }

  // Median of the samples seen so far (works before the buffer is full).
  uint16_t value() const {
    if (count_ == 0) return MAX_RANGE_CM;
    uint16_t tmp[N];
    for (uint8_t i = 0; i < count_; i++) tmp[i] = buf_[i];
    // Insertion sort: tiny array, no heap, predictable timing.
    for (uint8_t i = 1; i < count_; i++) {
      uint16_t key = tmp[i];
      int8_t j = static_cast<int8_t>(i - 1);
      while (j >= 0 && tmp[j] > key) {
        tmp[j + 1] = tmp[j];
        j--;
      }
      tmp[j + 1] = key;
    }
    return tmp[count_ / 2];
  }

 private:
  static const uint8_t N = 5;
  uint16_t buf_[N] = {0, 0, 0, 0, 0};
  uint8_t idx_ = 0;
  uint8_t count_ = 0;
};

// Zone for a distance with no memory of the previous state.
inline Zone rawZone(uint16_t d, const Thresholds& t) {
  if (d <= t.danger_cm) return ZONE_DANGER;
  if (d <= t.warning_cm) return ZONE_WARNING;
  if (d <= t.caution_cm) return ZONE_CAUTION;
  return ZONE_CLEAR;
}

// Zone with hysteresis: moving to a MORE severe zone happens immediately,
// moving to a SAFER zone needs the distance to clear the threshold by
// hysteresis_cm. This stops LEDs/buzzer flickering at a zone boundary.
inline Zone classify(uint16_t d, Zone prev, const Thresholds& t) {
  Zone now = rawZone(d, t);
  if (now >= prev) return now;  // same or more dangerous: react at once

  uint16_t backed_off = d > t.hysteresis_cm ? d - t.hysteresis_cm : 0;
  Zone safer = rawZone(backed_off, t);
  return safer < prev ? safer : prev;
}

struct BuzzerPattern {
  bool enabled;      // false -> silent
  bool continuous;   // true -> solid tone
  uint16_t period_ms;  // time between beep starts when not continuous
};

// Beep faster as the obstacle gets closer, like a car parking sensor.
// Period goes linearly from 800 ms at the caution threshold to 100 ms at the
// danger threshold; inside the danger zone the tone is continuous.
inline BuzzerPattern buzzerPattern(uint16_t d, Zone zone, const Thresholds& t) {
  if (zone == ZONE_CLEAR) return {false, false, 0};
  if (zone == ZONE_DANGER) return {true, true, 0};

  const uint16_t slow = 800, fast = 100;
  if (d >= t.caution_cm) return {true, false, slow};
  if (d <= t.danger_cm) return {true, false, fast};
  uint32_t span = t.caution_cm - t.danger_cm;
  uint32_t pos = d - t.danger_cm;
  uint16_t period = static_cast<uint16_t>(fast + (uint32_t)(slow - fast) * pos / span);
  return {true, false, period};
}

}  // namespace pa
