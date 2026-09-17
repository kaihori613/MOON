#pragma once
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cctype>
#include <cstdlib>
typedef bool boolean;
typedef uint8_t byte;
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
#define FALLING 2
#define abs(x) ((x)>0?(x):-(x))
#define constrain(a,l,h) ((a)<(l)?(l):((a)>(h)?(h):(a)))
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
#define PROGMEM
class String;
// Mirrors Arduino's Print class overload set, so an ambiguous or missing
// overload in the sketch fails here exactly as it would on the real toolchain.
struct Print {
  size_t print(const __FlashStringHelper*);
  size_t print(const char[]);
  size_t print(char);
  size_t print(unsigned char, int = DEC);
  size_t print(int, int = DEC);
  size_t print(unsigned int, int = DEC);
  size_t print(long, int = DEC);
  size_t print(unsigned long, int = DEC);
  size_t print(double, int = 2);
  size_t println(const __FlashStringHelper*);
  size_t println(const char[]);
  size_t println(char);
  size_t println(unsigned char, int = DEC);
  size_t println(int, int = DEC);
  size_t println(unsigned int, int = DEC);
  size_t println(long, int = DEC);
  size_t println(unsigned long, int = DEC);
  size_t println(double, int = 2);
  size_t println(void);
};
struct HardwareSerial : public Print {
  void begin(unsigned long);
  int available();
  int read();
  void flush();
  operator bool();
};
extern HardwareSerial Serial;
unsigned long millis();
unsigned long micros();
void delay(unsigned long);
void delayMicroseconds(unsigned int);
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int  digitalRead(uint8_t);
void analogWrite(uint8_t, int);
int  analogRead(uint8_t);
void attachInterrupt(uint8_t, void(*)(void), int);
uint8_t digitalPinToInterrupt(uint8_t);
long random(long);
void noInterrupts();
void interrupts();
long labs(long);
long map(long, long, long, long, long);
// Analog pin numbers as the Uno core defines them.
static const uint8_t A0=14, A1=15, A2=16, A3=17, A4=18, A5=19, A6=20, A7=21;
// AVR-libc, pulled in by the real Arduino.h on AVR targets.
char* dtostrf(double, signed char, unsigned char, char*);
