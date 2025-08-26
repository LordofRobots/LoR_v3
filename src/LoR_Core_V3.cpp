#include "LoR_Core_V3.h"

using namespace LoR;

static constexpr uint32_t WDT_TIMEOUT_SEC = 3;

void CoreV3::bootDiagnosticsLED() {
  Serial.print("BOOT Condition: ");
  auto rr = esp_reset_reason();
  switch (rr) {
    case ESP_RST_TASK_WDT: Serial.println("Watchdog Reset Detected"); _led.showState(LedState::Boot_White); break;
    case ESP_RST_BROWNOUT: Serial.println("Brownout Reset Detected"); _led.showState(LedState::Boot_Yellow); delay(3000); break;
    case ESP_RST_POWERON:  Serial.println("Power-on Reset Detected"); _led.showState(LedState::Boot_Green); break;
    case ESP_RST_SW:       Serial.println("Software Reset Detected"); _led.showState(LedState::Boot_Blue); break;
    case ESP_RST_PANIC:    Serial.println("Panic Reset Detected");    _led.showState(LedState::Boot_PanicRed); break;
    default:               Serial.println("UNKNOWN Reset Detected");  _led.showState(LedState::Boot_Purple); break;
  }
  FastLED.show();
  delay(500);
  _led.off();
  delay(100);
}

void CoreV3::begin() {
  // Serial
  Serial.begin(115200);

  // Watchdog
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);

  // Inputs
  pinMode(PIN_BTN_A, INPUT);
  pinMode(PIN_BTN_B, INPUT);
  pinMode(PIN_BTN_C, INPUT);
  pinMode(PIN_BTN_D, INPUT);
  pinMode(PIN_USER_SW, INPUT);
  pinMode(PIN_VIN_SENSE, INPUT);

  // LED strip
  _led.begin(PIN_LED_DATA, LED_COUNT, 255);

  // Boot diag LED
  bootDiagnosticsLED();

  // Battery
  _bat.begin(PIN_VIN_SENSE);

  // Gamepad (handles pair mode if A+D held)
  _gp.begin(&_led);
  _gp.ensurePairModeIfHeld();

  // Motors
  _mot.begin();
}

void CoreV3::tick() {
  // feed watchdog
  esp_task_wdt_reset();

  // bluepad update
  _gp.update();

  // battery sampling (2S, 3.0V/cell as your default)
  auto st = _bat.sample(2, 3.0f, /*debug*/false);
  _lowScale = st.scale;

  // LED states
  if (_gp.connected()) {
    _gp.handleBatteryLED();     // gamepad LED color updates
    _led.rainbowStep();         // active animation
  } else {
    _led.showState(LedState::Idle_IceBlue);
  }
}
