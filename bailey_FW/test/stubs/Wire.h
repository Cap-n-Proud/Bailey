#pragma once
struct SimWire {
  int sda = -1, scl = -1;
  void begin() {}
  void begin(int d, int c) { sda = d; scl = c; }   // ESP32: pins chosen in software
  void setClock(long) {}
};
extern SimWire Wire;
