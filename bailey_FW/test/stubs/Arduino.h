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
inline int strncmp_P(const char *a, const char *b, size_t n) { return strncmp(a, b, n); }
#define sq(x) ((x) * (x))

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

// Like Arduino's Print: everything goes through write(uint8_t).
class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  size_t write(const char *s) { size_t n = 0; while (*s) n += write((uint8_t)*s++); return n; }
  size_t print(const __FlashStringHelper *s) { return write((const char *)s); }
  size_t print(const char *s) { return write(s); }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(int v) { return write(std::to_string(v).c_str()); }
  size_t print(unsigned int v) { return write(std::to_string(v).c_str()); }
  size_t print(long v) { return write(std::to_string(v).c_str()); }
  size_t print(unsigned long v) { return write(std::to_string(v).c_str()); }
  size_t print(double v, int d = 2) { char b[64]; snprintf(b, sizeof b, "%.*f", d, v); return write(b); }
  template <class T> size_t println(T v) { return print(v) + write("\r\n"); }
  size_t println(double v, int d) { return print(v, d) + write("\r\n"); }
  size_t println() { return write("\r\n"); }
};

class SimSerial : public Print {
 public:
  std::deque<char> in;
  std::string line;          // current output line
  void (*onLine)(const std::string &) = nullptr;
  void begin(long) {}
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return c; }
  void send(const std::string &s) { for (char c : s) in.push_back(c); }
  using Print::write;
  size_t write(uint8_t c) override {
    if (c == '\n') { if (onLine) onLine(line); line.clear(); }
    else if (c != '\r') line += (char)c;
    return 1;
  }
};
extern SimSerial Serial;

#endif
