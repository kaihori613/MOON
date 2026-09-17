#pragma once
#include <Arduino.h>
struct EEPROMClass {
  uint8_t read(int);
  void write(int, uint8_t);
  void update(int, uint8_t);
  template<class T> T& get(int, T&);
  template<class T> const T& put(int, const T&);
};
extern EEPROMClass EEPROM;
