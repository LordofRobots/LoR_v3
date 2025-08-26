#pragma once
#include <Arduino.h>

// ---------------- LoR Core V3 Board Pin Map ----------------

// AUX port (slot 0 is sentinel / unused for 1-based indexing)
static constexpr uint8_t AUX_PINS[9]   = { 0, 5, 18, 23, 19, 22, 21, 1, 3 };

// IO port (slot 0 sentinel; 1..12 valid)
static constexpr uint8_t IO_PINS[13]   = { 0, 32, 25, 26, 27, 14, 12, 13, 15, 2, 4, 22, 21 };

// Internal inputs
static constexpr uint8_t PIN_BTN_A = 35;
static constexpr uint8_t PIN_BTN_B = 39;
static constexpr uint8_t PIN_BTN_C = 38;
static constexpr uint8_t PIN_BTN_D = 37;
static constexpr uint8_t PIN_USER_SW = 36;

// Battery sense (ADC)
static constexpr uint8_t PIN_VIN_SENSE = 34;

// Addressable LED
static constexpr uint8_t PIN_LED_DATA  = 33;
static constexpr int      LED_COUNT    = 4;

// Voltage scale from your two-point cal:
//  (ADC,Volt): (775,6.0) and (1720,12.0)
// slope = (12-6)/(1720-775) = 0.006349206
// offset = 6 - slope*775 = 1.079365
static constexpr float VIN_SLOPE  = 0.006349206f;
static constexpr float VIN_OFFSET = 1.079365f;
