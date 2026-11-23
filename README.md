# Arduino Parking Assist — Ultrasonic Collision Warning System

An embedded C++ project for the Arduino Uno that works like a car's rear parking sensor:
it measures distance with an HC-SR04 ultrasonic sensor, classifies it into safety zones,
and warns the driver with an LED bar and a buzzer that beeps faster as an obstacle gets closer.

![CI](../../actions/workflows/ci.yml/badge.svg)

## Features

| Feature | Why it matters in embedded systems |
|---|---|
| **5-sample median filter** | Rejects false echoes (single-sample spikes) without the lag of a big moving average |
| **4-zone state machine with hysteresis** | CLEAR → CAUTION → WARNING → DANGER; no LED/buzzer flicker at zone borders |
| **Non-blocking main loop** (`millis()`, no `delay()`) | Sensor, buzzer and button all run "concurrently" on a single core |
| **Proportional beep rate** | 800 ms period at 100 cm → 100 ms at 20 cm → continuous tone in DANGER |
| **Debounced mute button** | Auto re-arms when the path is clear again (fail-safe design) |
| **Hardware-independent logic** (`distance_logic.h`) | Unit-tested on a PC with g++, no board needed |
| **CI on GitHub Actions** | Every push runs unit tests and compiles the sketch for the Uno |
| **CSV serial telemetry** | `ms,raw_cm,filtered_cm,zone,muted` at 115200 baud — open the Serial Plotter to see filtering in action |

## Zones (default thresholds)

| Distance | Zone | LEDs | Buzzer |
|---|---|---|---|
| > 100 cm | CLEAR | 🟢 | off |
| 51–100 cm | CAUTION | 🟢🟡 | slow beeps |
| 21–50 cm | WARNING | 🟡🔴 | fast beeps |
| ≤ 20 cm | DANGER | 🔴 | continuous |

Moving to a *safer* zone needs an extra 5 cm (hysteresis). All values are in `THRESH` at the top of the sketch.

## Hardware

- Arduino Uno (or Nano)
- HC-SR04 ultrasonic sensor
- 3 LEDs (green, yellow, red) + 3 × 220 Ω resistors
- Passive/active buzzer
- Push button
- Breadboard and jumper wires

### Wiring

| Component | Arduino pin |
|---|---|
| HC-SR04 VCC / GND | 5V / GND |
| HC-SR04 TRIG | D9 |
| HC-SR04 ECHO | D10 |
| Green LED (via 220 Ω) | D4 |
| Yellow LED (via 220 Ω) | D5 |
| Red LED (via 220 Ω) | D6 |
| Buzzer + | D8 (− to GND) |
| Button | D2 ↔ GND (internal pull-up) |

## Run it

**No hardware? Simulate it online:**
1. Go to [wokwi.com](https://wokwi.com) → *New Project* → *Arduino Uno*.
2. Paste `parking_assist/parking_assist.ino` into `sketch.ino`, add a new file `distance_logic.h` with that file's contents, and replace `diagram.json` with the one in this repo.
3. Press ▶, click the HC-SR04 and drag the distance slider. Watch the LEDs, buzzer and Serial Monitor.

**Real board:** open the `parking_assist/` folder in the Arduino IDE, select *Arduino Uno*, upload, and open the Serial Monitor at 115200 baud.

**Unit tests (PC):**
```bash
g++ -std=c++11 -Wall -Wextra -o test_logic test/test_distance_logic.cpp
./test_logic
```

## Project structure

```
parking_assist/
  parking_assist.ino   # hardware layer: pins, timing, I/O
  distance_logic.h     # pure logic: conversion, filter, state machine, beep timing
test/
  test_distance_logic.cpp
diagram.json           # Wokwi simulation circuit
.github/workflows/ci.yml
```

## Possible extensions

- Show distance on a 16×2 I2C LCD or OLED
- Two sensors (left/right) for wider coverage
- Replace `pulseIn()` with a pin-change interrupt so measurement never blocks
- Port to an STM32 / ESP32 using FreeRTOS tasks
