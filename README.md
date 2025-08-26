![CI](https://github.com/LordofRobots/LoR_v3/actions/workflows/ci.yml/badge.svg)
 
# LoR_v3 — LoR Core V3 Board Support (Arduino)

Helpers for the **Lord of Robots – LoR Core V3** ESP32 board used in MiniBots.
This library packages board-specific pins, status LEDs, watchdog + boot diagnostics,
battery monitor, Bluepad32 gamepad pairing/connection logic, and 12-slot servo
motor outputs.

- **MCU:** ESP32 (bluepad32 core)
- **Dependencies:**
  - [Bluepad32](https://github.com/ricardoquesada/bluepad32)
  - [ESP32Servo] v3.0.7 (keep 3.0.7 to avoid timing regressions)
  - [FastLED]

> Tested with **Board**: `esp32_bluepad32 / ESP32 Dev Module`.

## Features

- **Pin maps** for AUX and IO ports plus internal inputs (buttons, switch, VIN ADC).
- **LED states** with FastLED (ice blue idle, green OK, red error, rainbow activity,
  blue/white pair blink, plus boot-cause colors).
- **Watchdog** (3 s) and **boot diagnostics** LED display.
- **Gamepad** manager (Bluepad32): single controller, A+D at boot enters pair mode,
  virtual device disabled, pairing disabled after successful connect.
- **Battery monitor**: ADC -> voltage using calibrated slope/offset; low-batt scale
  output (0/0.25/1.0) with 100 ms cadence.
- **12 motor outputs** (1..12) with common RC PWM ranges per motor type.

## Installation

1. Arduino IDE → **File → Preferences → Additional Boards Manager URLs**:
https://dl.espressif.com/dl/package_esp32_index.json
https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json

2. **Boards Manager**:
- Install **esp32** by Espressif Systems.
- Install **esp32_bluepad32** by Ricardo Quesada.
3. **Library Manager**:
- Install **FastLED** (latest).
- Install **ESP32Servo** **v3.0.7**.
- Install **Bluepad32**.
4. Clone/copy this repo to:
- `Documents/Arduino/libraries/LoR_v3/`
5. Select **Board**: `esp32_bluepad32 → ESP32 Dev Module`.

## Pinout (as used by the library)

- **AUX_PINS** (index 1..8): `{5, 18, 23, 19, 22, 21, 1, 3}`  
(UART/I2C/SPI capable, also usable as GPIO)
- **IO_PINS** (index 1..12): `{32, 25, 26, 27, 14, 12, 13, 15, 2, 4, 22, 21}`
- **Internal Inputs:**  
- `BTN_A=35`, `BTN_B=39`, `BTN_C=38`, `BTN_D=37`, `USER_SW=36`
- `VIN_SENSE=34` (ADC)
- **LED data:** `LED=33`, `LED_COUNT=4`

## Voltage Calibration

Two-point calibration from measured data `(ADC, Volt) = (775, 6.0 V)` and `(1720, 12.0 V)`:

- Slope `m = (12 − 6) / (1720 − 775) = 0.006349206`
- Offset `b = 6 − m × 775 = 1.079365`
- Conversion: `V = 0.006349206 * adc + 1.079365`

Used internally to compute VIN.

## LED States

- **Idle / Disconnected:** ice blue
- **OK / Ack:** green
- **Error / Disconnect:** red
- **Active:** rainbow
- **Pair Mode:** blinks blue ↔ white
- **Boot cause:** white (WDT), yellow (brownout), green (power-on), blue (software),
red (panic), purple (unknown)

## Pairing Flow (A + D on boot)

- Hold **BTN_A + BTN_D**, reset.
- Library forgets keys, enables pairing, blinks blue/white until a controller pairs.
- Then pairing is disabled; future boots auto-connect.

## Example

Open via **File → Examples → LoR_v3 → LoR_Core_V3_KitBot**. Minimal skeleton:

```cpp
#include <LoR_Core_V3.h>
using namespace LoR;

CoreV3 core;

void setup() {
core.begin();

// Configure motors (1..12), keep your preferred mapping
auto& mot = core.motors();
for (int i=1;i<=11;++i) mot.configure(i, MotorType::N20Plus, 90);
mot.configure(12, MotorType::MG90_CR, 90); // example continuous-rotation
}

void loop() {
core.tick(); // feeds WDT, updates LEDs/gamepad/battery

if (core.gamepadConnected()) {
 int sw = digitalRead(PIN_USER_SW);
 // Same mapping as original sketch:
 int left  = (sw == HIGH) ?  core.gamepad().axisRightY() : -core.gamepad().axisRightY();
 int right = (sw == HIGH) ? -core.gamepad().axisLeftY()  :  core.gamepad().axisLeftY();

 if (left  > -50 && left  < 50) left = 0;
 if (right > -50 && right < 50) right = 0;

 int L = constrain(map(left,  -512, 512, 0, 180), 0, 180);
 int R = constrain(map(right, -512, 512, 0, 180), 0, 180);

 auto& mot = core.motors();
 mot.writeDeg(1, L);  mot.writeDeg(2, L);  mot.writeDeg(3, L);
 mot.writeDeg(4, L);  mot.writeDeg(5, L);  mot.writeDeg(6, L);
 mot.writeDeg(7, R);  mot.writeDeg(8, R);  mot.writeDeg(9, R);
 mot.writeDeg(10, R); mot.writeDeg(11, R); mot.writeDeg(12, R);

 delay(50); // ~20 Hz
} else {
 core.motors().stopAll();
}
}
```

##Troubleshooting
- LoR_Core_V3.h: No such file : Ensure the library is installed at Documents/Arduino/libraries/LoR_v3/ with library.properties
and sources under src/. Restart the IDE.

- Board not found / upload fails : Select esp32_bluepad32 / ESP32 Dev Module. Install CH340 driver if required.

- Servos jitter / wrong range : Use ESP32Servo v3.0.7. Verify each output is configured with correct min/max pulse.

- Gamepad won’t pair : Hold A + D and reset to enter pair mode. Ensure Bluepad32 virtual device is disabled and new connections are re-disabled after pairing.

##Folder Structure
LoR_v3/
├─ library.properties
├─ keywords.txt
├─ src/
│  ├─ LoR_Core_V3.h / .cpp
│  ├─ LoR_Core_V3_Pins.h
│  ├─ LoR_LED.h / .cpp
│  ├─ LoR_Battery.h / .cpp
│  ├─ LoR_Gamepad.h / .cpp
│  └─ LoR_Motors.h / .cpp
└─ examples/
   └─ LoR_Core_V3_KitBot/LoR_Core_V3_KitBot.ino

##License
Apache-2.0
