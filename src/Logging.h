#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <Globals.h>
#include <SD.h>

// Stack-allocated CSV row builder. appendStr / appendFloat / appendLong /
// appendInt write into `data`; field() prepends a comma; str() returns a
// null-terminated view. No heap, no dynamic allocation — sized to fit one
// telemetry row plus header.
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

// Logging: writes telemetry rows to SD and (optionally) mirrors them to the
// active CommLink transport (USB / WiFi). Writes are gated on debug + a
// transport actually being connected so a dropped cable / WiFi never blocks
// the main loop on a full TX buffer.
class Logging {
 public:
  Logging(bool debug, bool logToSD, int SD_CS);

  // Toggle telemetry mirroring at runtime. Used to enable streaming when a
  // dashboard connects post-boot.
  void setDebug(bool d) { debug = d; }

  // Logs an arbitrary line (CSV row, marker, status message). Appends \r\n
  // when newline is true.
  void log(const char* message, bool newline = true);

  // Opens the next available LOG###.TXT on the SD card. Returns false if SD
  // init fails or the file can't be opened.
  bool begin();

  // Flushes the SD file. logTelemetry() also flushes once per second.
  void flush();

  // Builds and writes one row of the per-tick CSV schema (see setup() in
  // main.cpp for the column header).
  void logTelemetry(float altitude, float velocity, const SensorData_t& sens,
                    const BrakeState_t& brake, const I2CControl_t& i2c,
                    States st, bool armed);

  // Pre-launch checklist support.
  //   sdOk()      reflects the most recent open or write attempt.
  //   fileName()  returns the active log filename (e.g. "LOG003.TXT").
  //   selfTest()  writes a marker line and force-flushes — catches a card
  //               that opened OK at boot but has since been pulled or filled.
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
