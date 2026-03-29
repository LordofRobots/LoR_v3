#pragma once
#include <Bluepad32.h>
#include "LoR_Core_V3_Pins.h"
#include "LoR_LED.h"

namespace LoR {

class GamepadMgr {
public:
  void begin(LedStrip* led=nullptr);
  void update();
  bool connected() const { return _ctl && _ctl->isConnected(); }
  int16_t axisLeftY();
  int16_t axisRightY();
  int16_t axisLeftX();
  int16_t axisRightX();
  int16_t axisR2();
  int16_t axisL2();
  int16_t index()
  int16_t dpad() 
  int16_t buttons()
  int16_t miscButtons()
  int16_t gyroX()
  int16_t gyroY()
  int16_t gyroZ()
  int16_t accelX()
  int16_t accelY()
  int16_t accelZ()

  void handleBatteryLED();      // 1 Hz update like your code
  void ensurePairModeIfHeld();  // A + D on boot

private:
  ControllerPtr _ctl = nullptr;
  LedStrip* _led = nullptr;

  static void onConnected(ControllerPtr ctl);
  static void onDisconnected(ControllerPtr ctl);
  static GamepadMgr* _self;     // singleton-ish to bridge static callbacks

  uint32_t _lastBattMs = 0;
};

} // namespace LoR
