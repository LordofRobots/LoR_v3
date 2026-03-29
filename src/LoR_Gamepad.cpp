#include "LoR_Gamepad.h"

using namespace LoR;

GamepadMgr* GamepadMgr::_self = nullptr;

static void vibrate_and_green(ControllerPtr c) {
  if (!c) return;
  c->playDualRumble(0x00, 0xc0, 0xc0, 0xc0);
  c->setColorLED(0,255,0);
}

void GamepadMgr::onConnected(ControllerPtr ctl) {
  if (_self && !_self->_ctl) {
    Serial.println("! GamePad connected !");
    _self->_ctl = ctl;
    vibrate_and_green(ctl);
    BP32.enableNewBluetoothConnections(false);
    if (_self->_led) _self->_led->showState(LedState::Ok_Green);
    delay(500);
  } else {
    Serial.println("Another controller tried to connect but is rejected");
  }
}

void GamepadMgr::onDisconnected(ControllerPtr ctl) {
  if (_self && _self->_ctl == ctl) {
    Serial.println("! GamePad disconnected !");
    _self->_ctl = nullptr;
    if (_self->_led) _self->_led->showState(LedState::Error_Red);
    delay(1000);
  }
}

void GamepadMgr::begin(LedStrip* led) {
  _self = this;
  _led = led;
  // Optional virtual device off
  BP32.enableVirtualDevice(false);
}

void GamepadMgr::ensurePairModeIfHeld() {
  // Pair mode when A + D held at boot
  pinMode(PIN_BTN_A, INPUT);
  pinMode(PIN_BTN_D, INPUT);

  if (!digitalRead(PIN_BTN_A) && !digitalRead(PIN_BTN_D)) {
    BP32.forgetBluetoothKeys();
    Serial.println("Gamepad Unpaired!");
    BP32.enableNewBluetoothConnections(true);
    BP32.setup(&onConnected, &onDisconnected);

    // blink blue/white until paired
    while (!connected()) {
      if (_led) { fill_solid((CRGB*)nullptr, 0, CRGB::Black); } // no-op guard
      if (_led) { _led->showState(LedState::Boot_Blue); FastLED.show(); }
      delay(100);
      if (_led) { _led->showState(LedState::Boot_White); FastLED.show(); }
      delay(100);
      BP32.update();
      // feed WDT externally
    }
    BP32.enableNewBluetoothConnections(false);
  } else {
    BP32.setup(&onConnected, &onDisconnected);
  }
}

void GamepadMgr::update() {
  BP32.update();
}

int16_t GamepadMgr::axisLeftY()  { return _ctl ? _ctl->axisY()  : 0; }// (-511 - 512) left Y axis
int16_t GamepadMgr::axisRightY() { return _ctl ? _ctl->axisRY() : 0; }// (-511 - 512) right Y axis
int16_t GamepadMgr::axisLeftX()  { return _ctl ? _ctl->axisX()  : 0; }// (-511 - 512) left X Axis
int16_t GamepadMgr::axisRightX() { return _ctl ? _ctl->axisRX() : 0; }// (-511 - 512) right X axis
int16_t GamepadMgr::axisR2() { return _ctl ? _ctl->throttle() : 0; }// (0 - 1023): throttle (AKA gas) button
int16_t GamepadMgr::axisL2() { return _ctl ? _ctl->brake() : 0; }// (0 - 1023): brake button
int16_t GamepadMgr::index() { return _ctl ? _ctl->index() : 0; }// Controller Index
int16_t GamepadMgr::dpad() { return _ctl ? _ctl->dpad() : 0; }// D-pad
int16_t GamepadMgr::buttons() { return _ctl ? _ctl->buttons() : 0; }// bitmask of pressed buttons
int16_t GamepadMgr::miscButtons() { return _ctl ? _ctl->miscButtons() : 0; }// bitmask of pressed "misc" buttons
int16_t GamepadMgr::gyroX() { return _ctl ? _ctl->gyroX() : 0; }// Gyro X
int16_t GamepadMgr::gyroY() { return _ctl ? _ctl->gyroY() : 0; }// Gyro Y
int16_t GamepadMgr::gyroZ() { return _ctl ? _ctl->gyroZ() : 0; }// Gyro Z
int16_t GamepadMgr::accelX() { return _ctl ? _ctl->accelX() : 0; }// Accelerometer X
int16_t GamepadMgr::accelY() { return _ctl ? _ctl->accelY() : 0; }// Accelerometer Y
int16_t GamepadMgr::accelZ() { return _ctl ? _ctl->accelZ() : 0; }// Accelerometer Z



void GamepadMgr::handleBatteryLED() {
  if (!_ctl) return;
  if (millis() - _lastBattMs < 1000) return;
  _lastBattMs = millis();

  int battery = _ctl->battery();
  if (battery == 0) {
    _ctl->setColorLED(255,0,0);
  } else if (battery <= 64) {
    _ctl->setColorLED(255,0,0);
    Serial.println("! GamePad Low Battery !");
    _ctl->playDualRumble(0x00, 0xc0, 0xc0, 0xc0);
  } else if (battery <= 128) {
    _ctl->setColorLED(255,255,0);
  } else {
    _ctl->setColorLED(0,255,0);
  }
}
