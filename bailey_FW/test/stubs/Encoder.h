#pragma once
extern long simEncoder[64];
class Encoder {
  int pin;
 public:
  Encoder(int a, int) : pin(a) {}
  long read() { return simEncoder[pin]; }
};
