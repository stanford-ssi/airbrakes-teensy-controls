#include "Logging.h"
#include <SD.h>

Logging::Logging(int SD_CS) {
  this->SD_CS = SD_CS;
}

int Logging::getNextLogFileNumber() {
  int number = 1;
  char filename[20];
  while (true) {
    sprintf(filename, "LOG%03d.TXT", number);
    if (!SD.exists(filename)) break;
    number++;
  }
  return number;
}

bool Logging::begin() {
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed");
    return false;
  }
  Serial.println("SD init OK");

  int logNumber = getNextLogFileNumber();
  char logFileName[20];
  sprintf(logFileName, "LOG%03d.TXT", logNumber);
  Serial.print("Logging to ");
  Serial.println(logFileName);
  dataFile = SD.open(logFileName, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Failed to open log file");
    return false;
  }

  return true;
}

void Logging::log(const char *message, bool newline) {
  if (newline) {
    Serial.println(message);
    dataFile.println(message);
  } else {
    Serial.print(message);
    dataFile.print(message);
  }
}

void Logging::flush() {
  if (dataFile) {
    dataFile.flush();
  }
}
