// Host-side unit tests for distance_logic.h (no Arduino needed).
// Build & run:  g++ -std=c++11 -Wall -Wextra -o test_logic test/test_distance_logic.cpp && ./test_logic

#include <cstdio>
#include "../parking_assist/distance_logic.h"

static int failures = 0;
static int checks = 0;

#define CHECK_EQ(actual, expected)                                              \
  do {                                                                          \
    checks++;                                                                   \
    long a_ = (long)(actual), e_ = (long)(expected);                            \
    if (a_ != e_) {                                                             \
      failures++;                                                               \
      std::printf("FAIL %s:%d  %s == %ld, expected %ld\n", __FILE__, __LINE__, \
                  #actual, a_, e_);                                             \
    }                                                                           \
  } while (0)

using namespace pa;

static const Thresholds T = {100, 50, 20, 5};

void test_echo_conversion() {
  CHECK_EQ(echoToCm(0), MAX_RANGE_CM);      // timeout -> nothing in range
  CHECK_EQ(echoToCm(58), 1);
  CHECK_EQ(echoToCm(5800), 100);
  CHECK_EQ(echoToCm(60000), MAX_RANGE_CM);  // clamp
}

void test_median_rejects_spikes() {
  MedianFilter5 f;
  CHECK_EQ(f.value(), MAX_RANGE_CM);  // empty
  f.push(80);
  CHECK_EQ(f.value(), 80);
  f.push(82); f.push(400); f.push(81); f.push(79);  // one bogus 400 cm echo
  CHECK_EQ(f.value(), 81);
  f.push(3);  // one bogus short echo (replaces oldest: 80)
  CHECK_EQ(f.value(), 81);
}

void test_raw_zones() {
  CHECK_EQ(rawZone(150, T), ZONE_CLEAR);
  CHECK_EQ(rawZone(100, T), ZONE_CAUTION);
  CHECK_EQ(rawZone(50, T), ZONE_WARNING);
  CHECK_EQ(rawZone(20, T), ZONE_DANGER);
  CHECK_EQ(rawZone(0, T), ZONE_DANGER);
}

void test_hysteresis() {
  // Getting closer reacts immediately.
  CHECK_EQ(classify(49, ZONE_CAUTION, T), ZONE_WARNING);
  CHECK_EQ(classify(10, ZONE_CLEAR, T), ZONE_DANGER);
  // Backing out by less than the hysteresis band keeps the stricter zone.
  CHECK_EQ(classify(52, ZONE_WARNING, T), ZONE_WARNING);
  CHECK_EQ(classify(55, ZONE_WARNING, T), ZONE_WARNING);
  // Clearing the band moves to the safer zone.
  CHECK_EQ(classify(56, ZONE_WARNING, T), ZONE_CAUTION);
  // Jumping far away goes straight to CLEAR.
  CHECK_EQ(classify(300, ZONE_DANGER, T), ZONE_CLEAR);
  // Far but inside one band: DANGER -> WARNING only.
  CHECK_EQ(classify(24, ZONE_DANGER, T), ZONE_DANGER);
  CHECK_EQ(classify(30, ZONE_DANGER, T), ZONE_WARNING);
}

void test_buzzer_pattern() {
  BuzzerPattern p = buzzerPattern(150, ZONE_CLEAR, T);
  CHECK_EQ(p.enabled, false);

  p = buzzerPattern(10, ZONE_DANGER, T);
  CHECK_EQ(p.enabled, true);
  CHECK_EQ(p.continuous, true);

  p = buzzerPattern(100, ZONE_CAUTION, T);
  CHECK_EQ(p.period_ms, 800);
  p = buzzerPattern(60, ZONE_CAUTION, T);  // 40/80 of the way -> 100 + 350
  CHECK_EQ(p.period_ms, 450);
  p = buzzerPattern(21, ZONE_WARNING, T);
  CHECK_EQ(p.period_ms, 108);

  // Beeps get strictly faster as the obstacle approaches.
  uint16_t prev = 0xFFFF;
  for (uint16_t d = 100; d > 20; d--) {
    uint16_t period = buzzerPattern(d, rawZone(d, T), T).period_ms;
    if (period > prev) { failures++; std::printf("FAIL period rose at %u cm\n", (unsigned)d); }
    prev = period;
    checks++;
  }
}

int main() {
  test_echo_conversion();
  test_median_rejects_spikes();
  test_raw_zones();
  test_hysteresis();
  test_buzzer_pattern();
  std::printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
