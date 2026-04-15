#include "Logging.h"

#include <SD.h>
#include <States.h>
#include <TimeLib.h>

static time_t getTeensy3Time() { return Teensy3Clock.get(); }

void dateTime(uint16_t *date, uint16_t *time) {
  *date = FAT_DATE(year(), month(), day());
  *time = FAT_TIME(hour(), minute(), second());
}

Logging::Logging(bool debug, bool logToSD, int SD_CS) {
  this->debug = debug;
  this->logToSD = logToSD;
  this->SD_CS = SD_CS;
}

int Logging::getNextLogFileNumber() {
  int number = 1;
  char filename[20];

  // Increment until we find a number that does NOT exist
  while (true) {
    sprintf(filename, "LOG%03d.TXT", number);
    if (!SD.exists(filename)) {
      break;
    }
    number++;
  }
  return number;
}

bool Logging::begin() {
  if (debug) {
    delay(100);
    Serial.println(F("Logger starting..."));
  }

  setSyncProvider(getTeensy3Time);
  if (timeStatus() != timeSet) {
    if (debug) Serial.println(F("Teensy RTC not set, using compile time"));
  }

  if (logToSD) {
    if (!SD.begin(SD_CS)) {
      Serial.println(F("SD init failed"));
      return false;
    } else {
      Serial.println("SD init succeeded");
    }

    // callback so SD sets file timestamps
    SdFile::dateTimeCallback(dateTime);

    int logNumber = getNextLogFileNumber();
    char logFileName[20];
    sprintf(logFileName, "LOG%03d.TXT", logNumber);
    Serial.print("Opening log file: ");
    Serial.println(logFileName);
    dataFile = SD.open(logFileName, FILE_WRITE);
    if (!dataFile) {
      Serial.print("Failed to open file: ");
      Serial.println(logFileName);
      return false;
    }

    Serial.print("Logging to ");
    Serial.println(logFileName);
  }

  return true;
}

void Logging::log(const char *message, bool newline) {
  if (debug) {
    if (newline)
      Serial.println(message);
    else
      Serial.print(message);
  }
  if (logToSD) {
    if (newline)
      dataFile.println(message);
    else
      dataFile.print(message);
  }
}

void Logging::flush() {
  if (logToSD) {
    dataFile.flush();
  }
}

void Logging::logTelemetry(float altitude, const SensorData_t &sens, const BrakeState_t &brake, const I2CControl_t &i2c, States st) {
  LogBuffer buf;
  buf.appendLong(millis());
  buf.field(sens.accel_x);
  buf.field(sens.accel_y);
  buf.field(sens.accel_z);
  buf.field(sens.accel_x_high_g);
  buf.field(sens.accel_y_high_g);
  buf.field(sens.accel_z_high_g);
  buf.field(sens.pressure);
  buf.field(sens.temperature);
  buf.field(altitude);
  buf.field(sens.bno_x);
  buf.field(sens.bno_y);
  buf.field(sens.bno_z);
  buf.field(sens.bno_i, 4);
  buf.field(sens.bno_j, 4);
  buf.field(sens.bno_k, 4);
  buf.field(sens.bno_real, 4);
  buf.field(stateToString(st));
  buf.field(brake.pct, 1);
  buf.field(brake.direction);
  buf.field(i2c.predicted_apogee);
  buf.field(i2c.cd_add_cmd, 4);
  buf.field(i2c.fallback ? 1 : 0);
  buf.field(i2c.failCount);
  buf.field((int)sens.potentiometer_value);
  log(buf.str());

  if (millis() - lastFlush > 1000) {
    flush();
    lastFlush = millis();
  }
}
