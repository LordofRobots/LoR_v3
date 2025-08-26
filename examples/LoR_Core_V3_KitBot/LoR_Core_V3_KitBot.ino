// Lord of Robots - LoR Core V3 - AUG 3 2025
// Sample MiniBot Control Program (refactored to LoR_v3 library)

///////////////////////////////////////////////////////////////////////////////////////////
//            --- Environment Setup Prerequisites and Important Notes: ---               //
///////////////////////////////////////////////////////////////////////////////////////////
// 1. Add the following custom URLs to the Arduino IDE - File / Preferences / Additional Boards Manager URLs
//  - https://dl.espressif.com/dl/package_esp32_index.json
//  - https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
// 2. Install Boards in Board Manager:
//  - "esp32" by Espressif Systems
//  - "esp32_bluepad32" by Ricardo Quesada
// 3. Install libraries (Library Manager):
//  - "FastLED" (latest) by Daniel Garcia
//  - "ESP32Servo" (3.0.7) by Kevin Harrington, John K. Bennett
// 4. Driver: CH340 if needed
// 5. Select Target Board:
//  - USE "esp32_bluepad32 / ESP32 Dev Module"
//  - DO NOT use "esp32 / ESP32 Dev Module"

#include <LoR_Core_V3.h>
using namespace LoR;

static CoreV3 core;

void setup() {
  core.begin();

  // Motors startup (keep your mapping and types)
  auto& mot = core.motors();
  mot.configure(1,  MotorType::N20Plus, 90);
  mot.configure(2,  MotorType::N20Plus, 90);
  mot.configure(3,  MotorType::N20Plus, 90);
  mot.configure(4,  MotorType::N20Plus, 90);
  mot.configure(5,  MotorType::N20Plus, 90);
  mot.configure(6,  MotorType::N20Plus, 90);
  mot.configure(7,  MotorType::N20Plus, 90);
  mot.configure(8,  MotorType::N20Plus, 90);
  mot.configure(9,  MotorType::N20Plus, 90);
  mot.configure(10, MotorType::N20Plus, 90);
  mot.configure(11, MotorType::N20Plus, 90);
  mot.configure(12, MotorType::MG90_CR,  90);

  Serial.println("LoR Core V3 System Ready!");
}

void loop() {
  core.tick();

  // --- Drive logic (kept identical to your mapping) ---
  if (core.gamepadConnected()) {
    auto& gp  = core.gamepad();
    auto& mot = core.motors();

    // User_SW selects invert
    int sw = digitalRead(PIN_USER_SW);
    int currentLeft  = (sw == HIGH) ?  gp.axisRightY() : -gp.axisRightY();
    int currentRight = (sw == HIGH) ? -gp.axisLeftY()  :  gp.axisLeftY();   // inverted like your code

    // Deadband
    if (currentLeft  > -50 && currentLeft  < 50) currentLeft  = 0;
    if (currentRight > -50 && currentRight < 50) currentRight = 0;

    // Map to servo degrees
    int MappedLeft  = map(currentLeft,  -512, 512, 0, 180);
    int MappedRight = map(currentRight, -512, 512, 0, 180);
    MappedLeft  = constrain(MappedLeft,  0, 180);
    MappedRight = constrain(MappedRight, 0, 180);

    // Write to your slots (left group 1..6, right group 7..12)
    mot.writeDeg(1,  MappedLeft);
    mot.writeDeg(2,  MappedLeft);
    mot.writeDeg(3,  MappedLeft);
    mot.writeDeg(4,  MappedLeft);
    mot.writeDeg(5,  MappedLeft);
    mot.writeDeg(6,  MappedLeft);
    mot.writeDeg(7,  MappedRight);
    mot.writeDeg(8,  MappedRight);
    mot.writeDeg(9,  MappedRight);
    mot.writeDeg(10, MappedRight);
    mot.writeDeg(11, MappedRight);
    mot.writeDeg(12, MappedRight);

    delay(50);  // ~20 Hz, aligns with your original
  } else {
    core.motors().stopAll();
    // LED handled in core.tick()
  }
}
