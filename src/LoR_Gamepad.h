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
