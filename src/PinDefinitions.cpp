#include "PinDefinitions.h"

#include <Arduino.h>
#include <SD.h>

PinDefinitions::PinDefinitions()
    : STATUS_LED_RED(5),
      STATUS_LED_GREEN(6),
      STATUS_LED_BLUE(7),

      IGNITER_0(21),
      IGNITER_1(18),

      IGNITER_SENSE_0(16),
      IGNITER_SENSE_1(19),

      SERVO(10),
      SERVO_2(9),

      BUZZER(11),

      ESP32_CS(12),
      SD_CS(BUILTIN_SDCARD),

      SCK(35),
      SDI(13),
      SDO(14),

      MCP_CS(33),
      MCP_INT(21),

      SDA(40),
      SCL(41),

      ARM(22) {
  // Teensy 4.1 pin mapping (matches board schematic v4.1)
}

void PinDefinitions::setupPins() {
  pinMode(STATUS_LED_RED, OUTPUT);
  pinMode(STATUS_LED_GREEN, OUTPUT);
  pinMode(STATUS_LED_BLUE, OUTPUT);

  pinMode(IGNITER_0, OUTPUT);
  pinMode(IGNITER_1, OUTPUT);

  pinMode(IGNITER_SENSE_0, INPUT);
  pinMode(IGNITER_SENSE_1, INPUT);

  pinMode(SERVO, OUTPUT);
  pinMode(SERVO_2, OUTPUT);

  pinMode(BUZZER, OUTPUT);

  pinMode(ESP32_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  pinMode(SCK, OUTPUT);
  pinMode(SDI, OUTPUT);
  pinMode(SDO, INPUT);

  pinMode(MCP_CS, OUTPUT);
  pinMode(MCP_INT, INPUT);

  pinMode(ARM, INPUT_PULLUP);
}

PinDefinitions PinDefs;