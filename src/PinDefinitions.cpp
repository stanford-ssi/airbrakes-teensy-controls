#include "PinDefinitions.h"

#include <Arduino.h>
#include <SD.h>

PinDefinitions::PinDefinitions()
    : STATUS_LED_RED(3),
      STATUS_LED_GREEN(4),
      STATUS_LED_BLUE(5),

      IGNITER_0(29),
      IGNITER_1(26),

      IGNITER_SENSE_0(25),
      IGNITER_SENSE_1(27),

      SERVO(8),
      SERVO_2(7),

      BUZZER(9),

      ESP32_CS(10),
      SD_CS(BUILTIN_SDCARD),

      SCK(13),
      SDI(11),
      SDO(12),

      MCP_CS(20),
      MCP_INT(21),

      SDA(18),
      SCL(19),

      ARM(22) {
  // Teensy 4.1 pin mapping
}

void PinDefinitions::setupPins() {
  pinMode(STATUS_LED_RED, OUTPUT);
  pinMode(STATUS_LED_GREEN, OUTPUT);
  pinMode(STATUS_LED_BLUE, OUTPUT);

  pinMode(IGNITER_0, OUTPUT);
  pinMode(IGNITER_1, OUTPUT);

  pinMode(IGNITER_SENSE_0, INPUT);
  pinMode(IGNITER_SENSE_1, INPUT);

  // Servo pins configured by Servo.attach(), not pinMode

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