#pragma once
#include "Arduino.h"
struct SimEEPROM {
  uint8_t mem[4096];
  SimEEPROM() { memset(mem, 0xFF, sizeof mem); }
  // ESP32 flash emulation: begin() must come first, writes only stick after commit().
  int begun = 0, commits = 0;
  bool begin(size_t) { begun++; return true; }
  bool commit() { commits++; return true; }
  uint8_t read(int a) { return mem[a]; }
  void write(int a, uint8_t v) { mem[a] = v; }
  template <class T> T &get(int a, T &t) { memcpy(&t, mem + a, sizeof(T)); return t; }
  template <class T> const T &put(int a, const T &t) { memcpy(mem + a, &t, sizeof(T)); return t; }
};
extern SimEEPROM EEPROM;
