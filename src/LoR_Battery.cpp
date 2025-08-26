#include "LoR_Battery.h"

using namespace LoR;

BatteryState BatteryMonitor::sample(uint8_t cellCount, float perCellLowV, bool debug, uint32_t dbgPeriodMs) {
  int raw = analogRead(_adc);
  float vin = raw * VIN_SLOPE + VIN_OFFSET;
  float threshold = cellCount * perCellLowV;

  static float scale = 1.0f;

  if (vin < threshold) {
    if (millis() >= _nextToggle) {
      _toggle = !_toggle;
      _nextToggle = millis() + 100;       // blink / throttle cadence
      scale = _toggle ? 0.25f : 0.0f;     // your original behavior
    }
  } else {
    scale = 1.0f;
  }

  if (debug && millis() >= _nextDbg) {
    _nextDbg = millis() + dbgPeriodMs;
    Serial.printf("VIN: %.2f V, Threshold: %.2f V\n", vin, threshold);
    if (vin < threshold) Serial.printf("LOW Battery: %.2f V\n", vin);
  }

  return BatteryState{vin, threshold, scale};
}
