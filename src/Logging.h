#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <Globals.h>
#include <SD.h>

// Fixed-size CSV row builder; avoids heap use in the flight loop.
struct LogBuffer {
  char data[256];
  int pos = 0;

  void appendStr(const char *s) {
    while (*s && pos < (int)sizeof(data) - 1) data[pos++] = *s++;
    data[pos] = '\0';
  }

  void appendFloat(float v, int decimals = 2) {
    char tmp[24];
    dtostrf(v, 0, decimals, tmp);
    appendStr(tmp);
  }

  void appendLong(unsigned long v) {
    char tmp[12];
    ultoa(v, tmp, 10);
    appendStr(tmp);
  }

  void appendInt(int v) {
    char tmp[12];
    itoa(v, tmp, 10);
    appendStr(tmp);
  }

  void comma() { appendStr(","); }
  void field(float v, int d = 2) { comma(); appendFloat(v, d); }
  void field(int v) { comma(); appendInt(v); }
  void field(const char *s) { comma(); appendStr(s); }

  const char *str() { return data; }
};

// SD logger with optional USB/WiFi mirroring.
class Logging {
 public:
  Logging(bool debug, bool logToSD, int SD_CS);

  // Enable telemetry mirroring after a dashboard connects.
  void setDebug(bool d) { debug = d; }

  // Log one line or partial line.
  void log(const char* message, bool newline = true);

  // Open the next LOG###.TXT file.
  bool begin();

  // Flush pending SD writes.
  void flush();

  // Write one telemetry row.
  void logTelemetry(float altitude, float velocity, const SensorData_t& sens,
                    const BrakeState_t& brake, const I2CControl_t& i2c,
                    States st, bool armed);

  // Preflight SD checks.
  bool sdOk() const { return sd_ok; }
  const char* fileName() const { return log_filename; }
  bool selfTest();

 private:
  bool debug;
  bool logToSD;
  int SD_CS;
  bool sd_ok = false;
  char log_filename[16] = {0};
  int getNextLogFileNumber();
  int logNumber;
  String logFileName;  // unused, kept for ABI compatibility
  File dataFile;
  unsigned long lastFlush = 0;
};

#endif
