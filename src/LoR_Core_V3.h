#pragma once
#include <Arduino.h>
#include "esp_task_wdt.h"
#include "LoR_Core_V3_Pins.h"
#include "LoR_LED.h"
#include "LoR_Battery.h"
#include "LoR_Gamepad.h"
#include "LoR_Motors.h"

namespace LoR {

class CoreV3 {
public:
  void begin();
  void tick(); // call at ~20 Hz in loop
  bool gamepadConnected() const { return _gp.connected(); }

  // Expose subsystems (read-only usage suggested)
  LedStrip&      led()     { return _led; }
  BatteryMonitor& battery(){ return _bat; }
  GamepadMgr&    gamepad() { return _gp; }
  MotorOutputs&  motors()  { return _mot; }

  // Boot diagnostics LED (reset cause like your code)
  void bootDiagnosticsLED();

private:
  LedStrip       _led;
  BatteryMonitor _bat;
  GamepadMgr     _gp;
  MotorOutputs   _mot;

  // low-batt scaling factor
  float _lowScale = 1.0f;
};

} // namespace LoR
