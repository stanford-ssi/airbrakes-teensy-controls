#include "Logging.h"

#include <CommLink.h>
#include <SD.h>
#include <States.h>
#include <TimeLib.h>

// Sets the SD library's wall-clock callback so files get real timestamps.
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

// Walks LOG001.TXT, LOG002.TXT, ... until it finds one that doesn't exist.
// Linear scan — O(N) in the number of past flights, but N stays small.
int Logging::getNextLogFileNumber() {
  int number = 1;
  char filename[20];
  while (true) {
    sprintf(filename, "LOG%03d.TXT", number);
    if (!SD.exists(filename)) {
      break;
    }
    number++;
  }
  return number;
}

// Initialises the RTC sync, mounts the SD card, and opens the next free
// LOG###.TXT for append. Returns false on SD failure (caller decides whether
// to continue without a log).
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
      sd_ok = false;
      return false;
    } else {
      Serial.println("SD init succeeded");
    }

    SdFile::dateTimeCallback(dateTime);

    int logNumber = getNextLogFileNumber();
    snprintf(log_filename, sizeof(log_filename), "LOG%03d.TXT", logNumber);
    Serial.print("Opening log file: ");
    Serial.println(log_filename);
    dataFile = SD.open(log_filename, FILE_WRITE);
    if (!dataFile) {
      Serial.print("Failed to open file: ");
      Serial.println(log_filename);
      sd_ok = false;
      return false;
    }

    Serial.print("Logging to ");
    Serial.println(log_filename);
    sd_ok = true;
  }

  return true;
}

// Pre-flight checklist hook. Writes a marker line, force-flushes, and
// returns true only if both the write and the file handle are still healthy.
// Catches a card that opened OK at boot but was then pulled or filled up.
bool Logging::selfTest() {
  if (!logToSD) return false;
  if (!dataFile) {
    sd_ok = false;
    return false;
  }
  size_t n = dataFile.println("# CHECK_SD_MARKER");
  dataFile.flush();
  bool ok = (n > 0) && dataFile;
  sd_ok = ok;
  return ok;
}

// Writes one line. Mirrors to comm only when debug is on AND a transport is
// currently connected — without the connection check, a dropped USB cable
// fills the TX buffer and eventually blocks the main loop. SD writes always
// run so every tick lands in the launch log regardless of comm state.
void Logging::log(const char *message, bool newline) {
  if (debug && comm.connected()) {
    if (newline)
      comm.println(message);
    else
      comm.print(message);
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

// Builds one CSV telemetry row and writes it via log(). Auto-flushes the SD
// file once per second so a sudden power loss only loses the last <1 s of
// data. Schema is append-only: new fields go on the end so old dashboards
// parsing the first 29 columns keep working.
void Logging::logTelemetry(float altitude, float velocity, const SensorData_t &sens, const BrakeState_t &brake, const I2CControl_t &i2c, States st, bool armed) {
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
  buf.field(velocity);
  // Predictor diagnostics — let the dashboard cross-check the Teensy
  // predictor against the Python port.
  buf.field(i2c.target_cd_raw, 4);
  buf.field(i2c.apo_no_brakes, 1);
  buf.field(i2c.apo_max_brakes, 1);
  buf.field(armed ? 1 : 0);
  // Post-flight maxima — surface peak alt / speed / g and the launch +
  // apogee timestamps so a late-reconnecting dashboard sees the summary
  // immediately.
  buf.field(FlightState.max_altitude, 1);
  buf.field(FlightState.max_velocity, 1);
  buf.field(FlightState.max_accel_g, 2);
  buf.field((int)FlightState.ignition_time);
  buf.field((int)FlightState.fire_time);
  log(buf.str());

  if (millis() - lastFlush > 1000) {
    flush();
    lastFlush = millis();
  }
}
