#pragma once
#include <Arduino.h>
struct TwoWire : public Print {
  void begin();
  void beginTransmission(uint8_t);
  size_t write(uint8_t);
  uint8_t endTransmission();
  uint8_t endTransmission(bool);
  uint8_t requestFrom(uint8_t, uint8_t);
  int read();
  int available();
  void setClock(unsigned long);
};
extern TwoWire Wire;
