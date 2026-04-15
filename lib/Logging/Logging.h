#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <SD.h>

struct LogBuffer {
  char data[384];
  int pos = 0;

  void appendStr(const char *s) {
    while (*s && pos < (int)sizeof(data) - 1) data[pos++] = *s++;
    data[pos] = '\0';
  }

  void appendFloat(float v, int decimals = 2) {
    char tmp[16];
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

class Logging {
 public:
  Logging(int SD_CS);

  void log(const char* message, bool newline = true);
  bool begin();
  void flush();

 private:
  int SD_CS;
  int getNextLogFileNumber();
  File dataFile;
  unsigned long lastFlush = 0;
};

#endif
