#pragma once
#include <FastLED.h>
#include "LoR_Core_V3_Pins.h"

namespace LoR {

enum class LedState : uint8_t {
  Idle_IceBlue,     // waiting / disconnected
  Ok_Green,         // system OK / ack
  Error_Red,        // error / disconnect
  Pair_Blink,       // blue <-> white blink
  Rainbow_Active,   // when receiving gamepad data
  Boot_White, Boot_Yellow, Boot_Green, Boot_Blue, Boot_PanicRed, Boot_Purple
};

class LedStrip {
public:
  void begin(uint8_t pin = PIN_LED_DATA, int count = LED_COUNT, uint8_t brightness = 255);
  void showState(LedState s);
  void rainbowStep();     // call in loop when active
  void off();
private:
  CRGB* _leds = nullptr;
  int   _count = 0;
  uint8_t _h = 0;
};

} // namespace LoR
