#pragma once
#include <ESP32Servo.h>
#include "LoR_Core_V3_Pins.h"

namespace LoR {

enum class MotorType : uint8_t {
  MG90_CR,
  MG90_Degree,
  N20Plus,
  Victor_SPX,
  Talon_SRX,
  STD_SERVO,
  CUSTOM
};

struct MotorTypeConfig {
  MotorType type;
  float pwmHz;
  int minUs;
  int maxUs;
  float inMin;
  float inMax;
};

// 12 slots (1..12); slot 0 unused to match your code
class MotorOutputs {
public:
  void begin();
  void configure(uint8_t slot, MotorType type, int startupDeg = 90);
  void writeDeg(uint8_t slot, int deg);        // 0..180
  void stopAll();                              // write 90 to all
private:
  Servo _s[13];
  static const MotorTypeConfig* lookup(MotorType type);
};

} // namespace LoR
