#include "LoR_LED.h"

using namespace LoR;

void LedStrip::begin(uint8_t pin, int count, uint8_t brightness) {
  _count = count;
  _leds = new CRGB[_count];
  FastLED.addLeds<WS2812B, PIN_LED_DATA, GRB>(_leds, _count);
  FastLED.setBrightness(brightness);
  off();
}

void LedStrip::off() {
  if (!_leds) return;
  fill_solid(_leds, _count, CRGB::Black);
  FastLED.show();
}

static inline void fill_and_show(CRGB* leds, int n, uint8_t r, uint8_t g, uint8_t b) {
  fill_solid(leds, n, CRGB(r,g,b));
  FastLED.show();
}

void LedStrip::showState(LedState s) {
  if (!_leds) return;
  switch (s) {
    case LedState::Idle_IceBlue:   fill_and_show(_leds, _count, 0, 80, 255); break;
    case LedState::Ok_Green:       fill_and_show(_leds, _count, 0, 255, 0); break;
    case LedState::Error_Red:      fill_and_show(_leds, _count, 255, 0, 0); break;
    case LedState::Pair_Blink:     /* handled by caller as alternating Blue/White */ break;
    case LedState::Boot_White:     fill_and_show(_leds, _count, 255,255,255); break;
    case LedState::Boot_Yellow:    fill_and_show(_leds, _count, 255,255,0);   break;
    case LedState::Boot_Green:     fill_and_show(_leds, _count, 0,255,0);     break;
    case LedState::Boot_Blue:      fill_and_show(_leds, _count, 0,0,255);     break;
    case LedState::Boot_PanicRed:  fill_and_show(_leds, _count, 255,0,0);     break;
    case LedState::Boot_Purple:    fill_and_show(_leds, _count, 255,0,255);   break;
    case LedState::Rainbow_Active: /* rainbowStep() drives frames */ break;
  }
}

void LedStrip::rainbowStep() {
  if (!_leds) return;
  fill_rainbow(_leds, _count, _h++, 20);
  FastLED.show();
}
