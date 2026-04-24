// ─────────────────────────────────────────────────────────────────────────────
// ESP32 AirLift UART passthrough for reflashing NINA-FW.
//
// On boot: holds GPIO0 LOW and pulses RESETN so the ESP32 enters UART
// download (bootloader) mode. Then bridges bytes between the host's USB serial
// and the ESP32's UART on Teensy Serial1 (pins 0=RX1, 1=TX1). Mirrors host
// baud rate changes to Serial1 so esptool can negotiate faster speeds.
//
// Wiring required for this sketch:
//   AirLift TX   → Teensy pin 0  (RX1)
//   AirLift RX   → Teensy pin 1  (TX1)
//   AirLift GP0  → Teensy pin 15
//   AirLift RST  → Teensy pin 16
//   AirLift VIN  → 5V
//   AirLift GND  → GND
//
// Flash workflow:
//   1) pio run -e esp32_passthrough -t upload     (ESP32 now in bootloader)
//   2) esptool --port <port> --before no_reset --after no_reset \
//              --baud 115200 write-flash 0x0 firmware/NINA_W102-1.7.7.bin
//   3) pio run -e wifi_bringup -t upload          (releases GPIO0, resets ESP32)
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>

constexpr int ESP32_RESETN_PIN = 16;
constexpr int ESP32_GPIO0_PIN  = 15;

void setup() {
  pinMode(ESP32_RESETN_PIN, OUTPUT);
  pinMode(ESP32_GPIO0_PIN,  OUTPUT);

  // Enter UART bootloader: GPIO0 LOW at the rising edge of RESETN.
  digitalWrite(ESP32_GPIO0_PIN,  LOW);
  digitalWrite(ESP32_RESETN_PIN, HIGH);
  delay(10);
  digitalWrite(ESP32_RESETN_PIN, LOW);
  delay(100);
  digitalWrite(ESP32_RESETN_PIN, HIGH);
  // GPIO0 stays LOW to keep the ESP32 in download mode until the next reset.

  Serial.begin(115200);
  Serial1.begin(115200);
}

uint32_t last_baud = 115200;

void loop() {
  // Teensy USB CDC exposes the host-side baud rate via Serial.baud(). Mirror
  // it to Serial1 so esptool's CHANGE_BAUDRATE works: when esptool bumps the
  // host port to 460800, we bump Serial1 to match.
  uint32_t host_baud = Serial.baud();
  if (host_baud && host_baud != last_baud) {
    Serial1.begin(host_baud);
    last_baud = host_baud;
  }

  while (Serial.available())  Serial1.write(Serial.read());
  while (Serial1.available()) Serial.write(Serial1.read());
}
