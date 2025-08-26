#pragma once
#include <Arduino.h>
#include "LoR_Core_V3_Pins.h"

namespace LoR {

struct BatteryState {
  float vin;          // Volts
  float threshold;    // cellCount * perCellLowV
  float scale;        // 1.0 (normal) or throttled (0..1)
};

class BatteryMonitor {
public:
  void begin(uint8_t adcPin = PIN_VIN_SENSE) { _adc = adcPin; pinMode(_adc, INPUT); }
  BatteryState sample(uint8_t cellCount, float perCellLowV = 3.0f, bool debug = false, uint32_t dbgPeriodMs = 500);

private:
  uint8_t _adc = PIN_VIN_SENSE;
  uint32_t _nextDbg = 0;
  // flasher for low-batt scale modulation
  bool _toggle = false;
  uint32_t _nextToggle = 0;
};

} // namespace LoR
