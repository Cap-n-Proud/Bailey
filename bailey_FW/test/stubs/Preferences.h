// Simulator stand-in for the ESP32 Preferences library (settings in NVS
// flash). The store lives as long as the process, like flash over a reboot.
#pragma once
#include <map>
#include <string>
#include <vector>
#include <stdint.h>
#include <string.h>
struct SimNvs {
  std::map<std::string, std::vector<uint8_t>> data;
  int writes = 0;
};
extern SimNvs simNvs;
class Preferences {
  std::string ns;
  bool open = false, readOnly = false;
  std::string key(const char *k) const { return ns + "/" + k; }
 public:
  bool begin(const char *name, bool ro = false) {
    ns = name;
    readOnly = ro;
    open = true;
    return true;
  }
  void end() { open = false; }
  size_t getBytes(const char *k, void *buf, size_t len) {
    auto it = simNvs.data.find(key(k));
    if (!open || it == simNvs.data.end()) return 0;
    size_t n = it->second.size() < len ? it->second.size() : len;
    memcpy(buf, it->second.data(), n);
    return n;
  }
  size_t putBytes(const char *k, const void *buf, size_t len) {
    if (!open || readOnly) return 0;
    simNvs.data[key(k)] = std::vector<uint8_t>((const uint8_t *)buf, (const uint8_t *)buf + len);
    simNvs.writes++;
    return len;
  }
  bool remove(const char *k) {
    if (!open || readOnly) return false;
    return simNvs.data.erase(key(k)) > 0;
  }
};
