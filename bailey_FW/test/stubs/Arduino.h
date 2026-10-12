// Minimal Arduino API so the firmware can be compiled and simulated on a PC.
#ifndef SIM_ARDUINO_H
#define SIM_ARDUINO_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <deque>

typedef bool boolean;
typedef uint8_t byte;

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define F_CPU 16000000UL

#define PROGMEM
#define PSTR(s) (s)
#define F(s) ((const __FlashStringHelper *)(s))
class __FlashStringHelper;
inline int strcmp_P(const char *a, const char *b) { return strcmp(a, b); }

template <class T, class L, class H>
inline T constrain(T x, L lo, H hi) { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }

// Simulated clock (microseconds), advanced by the simulator.
extern uint64_t simMicros;
inline unsigned long micros() { return (unsigned long)simMicros; }
inline unsigned long millis() { return (unsigned long)(simMicros / 1000); }
inline void delay(unsigned long ms) { simMicros += ms * 1000ULL; }
inline void noInterrupts() {}
inline void interrupts() {}

extern int simPinValue[64];
extern int simPwm[64];
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, uint8_t v) { simPinValue[pin] = v; }
inline void analogWrite(uint8_t pin, int v) { simPwm[pin] = v; }
inline int analogRead(uint8_t) { return 0; }
inline void randomSeed(unsigned long s) { srand(s); }
inline long random(long lo, long hi) { return lo + rand() % (hi - lo); }

class SimSerial {
 public:
  std::deque<char> in;
  std::string out;
  std::string line;          // current output line
  void (*onLine)(const std::string &) = nullptr;
  void begin(long) {}
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return c; }
  void send(const std::string &s) { for (char c : s) in.push_back(c); }
  void put(const std::string &s) {
    for (char c : s) {
      out += c;
      if (c == '\n') { if (onLine) onLine(line); line.clear(); }
      else if (c != '\r') line += c;
    }
  }
  void print(const __FlashStringHelper *s) { put((const char *)s); }
  void print(const char *s) { put(s); }
  void print(char c) { put(std::string(1, c)); }
  void print(int v) { put(std::to_string(v)); }
  void print(unsigned int v) { put(std::to_string(v)); }
  void print(long v) { put(std::to_string(v)); }
  void print(unsigned long v) { put(std::to_string(v)); }
  void print(double v, int d = 2) { char b[64]; snprintf(b, sizeof b, "%.*f", d, v); put(b); }
  template <class T> void println(T v) { print(v); put("\r\n"); }
  void println(double v, int d) { print(v, d); put("\r\n"); }
  void println() { put("\r\n"); }
};
extern SimSerial Serial;

#endif
