#include "LoR_Motors.h"

using namespace LoR;

static const MotorTypeConfig kTable[] = {
  { MotorType::MG90_CR,     50,  500, 2500, -1, 1 },
  { MotorType::MG90_Degree, 50,  500, 2500,  1, 180 },
  { MotorType::N20Plus,     50, 1000, 2000, -1, 1 },
  { MotorType::Victor_SPX,  50, 1000, 2000, -1, 1 },
  { MotorType::Talon_SRX,   50, 1000, 2000, -1, 1 },
  { MotorType::STD_SERVO,   50, 1000, 2000,  0, 180 },
};

const MotorTypeConfig* MotorOutputs::lookup(MotorType type) {
  for (auto& cfg : kTable) if (cfg.type == type) return &cfg;
  return nullptr;
}

void MotorOutputs::begin() {
  // nothing persistent to do here; pins attached per configure()
}

void MotorOutputs::configure(uint8_t slot, MotorType type, int startupDeg) {
  if (slot == 0 || slot > 12) return;
  const auto *cfg = lookup(type);
  float pwmHz = cfg ? cfg->pwmHz : 50.0f;
  int minUs    = cfg ? cfg->minUs : 1000;
  int maxUs    = cfg ? cfg->maxUs : 2000;

  uint8_t pin = IO_PINS[slot];
  pinMode(pin, OUTPUT);
  _s[slot].setPeriodHertz(pwmHz);
  _s[slot].attach(pin, minUs, maxUs);
  _s[slot].write(startupDeg);

  Serial.printf("Motor slot %u on pin %u type %u: %.1f Hz, %d-%d us, start=%d deg\n",
      slot, pin, (unsigned)type, pwmHz, minUs, maxUs, startupDeg);
}

void MotorOutputs::writeDeg(uint8_t slot, int deg) {
  if (slot == 0 || slot > 12) return;
  _s[slot].write(constrain(deg, 0, 180));
}

void MotorOutputs::stopAll() {
  for (int i=1;i<=12;++i) _s[i].write(90);
}
