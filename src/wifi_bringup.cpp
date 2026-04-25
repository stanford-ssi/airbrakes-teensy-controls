// ─────────────────────────────────────────────────────────────────────────────
// AirLift WiFi hardware bringup smoke test.
//
// This is NOT part of the main firmware. It is a standalone sketch that boots
// the ESP32 AirLift as a WiFi Access Point, prints the AirLift firmware
// version (proving SPI + control pins work), and runs a TCP echo server.
//
// Why a separate sketch: the main firmware does a lot of init (SD, sensors,
// servos). If WiFi fails inside that, you can't tell whether the radio is dead
// or some other init step broke. This sketch only does WiFi, so any failure
// is necessarily a WiFi problem.
//
//   Build & flash:   pio run -e wifi_bringup -t upload
//   Watch serial:    pio device monitor -e wifi_bringup
//
// What success looks like on the serial console:
//   === AirLift WiFi bringup ===
//   AirLift firmware: 1.4.x
//   AP up. SSID="AirbrakesRocket" IP=192.168.4.1 TCP=4040
//   Listening for clients...
//
// Then, from your laptop:
//   1. Join the WiFi network "AirbrakesRocket" (password below).
//   2. Open a terminal:  nc 192.168.4.1 4040
//   3. Type something. The Teensy echoes it back AND mirrors it to USB serial.
//
// Failure modes and what they mean:
//   "AirLift not detected"  → wrong CS/BUSY/RESET pin, SPI not wired,
//                             or AirLift unpowered.
//   "Firmware: ?.?.?"       → SPI works but FW handshake failed, often a
//                             stale/corrupted NINA-FW on the ESP32.
//   "beginAP returned <n>"  → radio is up but couldn't start AP (rare on FW
//                             ≥1.4.x, sometimes a power-supply issue).
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>

#include "WifiPins.h"

// ─── Pin / WiFi config ───────────────────────────────────────────────────────
// All four AirLift control pins and the AP credentials live in WifiPins.h so
// this sketch and CommLink stay in lockstep. SPI lines (MOSI=11, MISO=12,
// SCK=13) are implicit — they're the default Teensy 4.1 SPI peripheral
// (LPSPI4 on the top edge); the AirLift breakout's MOSI/MISO/SCK pads must
// be wired to those Teensy pins for hardware SPI to work.
constexpr int ESP32_CS_PIN     = WifiPins::CS;
constexpr int ESP32_BUSY_PIN   = WifiPins::BUSY;
constexpr int ESP32_RESETN_PIN = WifiPins::RESETN;
constexpr int ESP32_GPIO0_PIN  = WifiPins::GPIO0;
const char* AP_SSID            = WifiAP::SSID;
const char* AP_PASS            = WifiAP::PASS;
constexpr uint16_t TCP_PORT    = WifiAP::PORT;

WiFiServer server(TCP_PORT);
WiFiClient client;

// Health/status counters — pushed to the connected client every second so the
// dashboard has something concrete to display while we're still in bringup.
static uint32_t bytes_in = 0;
static uint32_t bytes_out = 0;
static uint32_t last_status_ms = 0;
static const uint32_t STATUS_INTERVAL_MS = 1000;

static void haltBlinking() {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 4000) {}
  Serial.println(F("\n=== AirLift WiFi bringup ==="));
  Serial.print(F("Pins: CS="));    Serial.print(ESP32_CS_PIN);
  Serial.print(F(" BUSY="));       Serial.print(ESP32_BUSY_PIN);
  Serial.print(F(" RESETN="));     Serial.print(ESP32_RESETN_PIN);
  Serial.print(F(" GPIO0="));      Serial.println(ESP32_GPIO0_PIN);

  // Manual ESP32 reset + BUSY-line probe BEFORE handing control to WiFiNINA.
  // The library's internal reset is sometimes too fast on Teensy 4.1, leaving
  // the ESP32 in an indeterminate state. We pulse RESETN ourselves and watch
  // BUSY to see whether the ESP32 actually boots.
  pinMode(ESP32_RESETN_PIN, OUTPUT);
  pinMode(ESP32_BUSY_PIN,   INPUT);
  pinMode(ESP32_CS_PIN,     OUTPUT);
  pinMode(ESP32_GPIO0_PIN,  OUTPUT);
  digitalWrite(ESP32_GPIO0_PIN, HIGH); // strap for normal NINA boot (not UART download)
  digitalWrite(ESP32_CS_PIN, HIGH);    // deassert CS
  Serial.print(F("BUSY before reset: "));
  Serial.println(digitalRead(ESP32_BUSY_PIN));
  digitalWrite(ESP32_RESETN_PIN, LOW);
  // Read BUSY multiple times during the 100ms hold. If ESP32 is the sole
  // driver of BUSY, it should go LOW (or float) while held in reset.
  Serial.print(F("BUSY during RESETN=LOW (500ms): "));
  for (int i = 0; i < 10; i++) { Serial.print(digitalRead(ESP32_BUSY_PIN)); delay(50); }
  Serial.println();
  digitalWrite(ESP32_RESETN_PIN, HIGH);
  Serial.println(F("RESETN released, polling BUSY for 10s:"));
  uint32_t t_reset = millis();
  int last = -1;
  while (millis() - t_reset < 10000) {
    int b = digitalRead(ESP32_BUSY_PIN);
    if (b != last) {
      Serial.print(F("  t=")); Serial.print(millis() - t_reset);
      Serial.print(F("ms BUSY=")); Serial.println(b);
      last = b;
    }
    delay(5);
  }
  Serial.print(F("BUSY final: "));
  Serial.println(digitalRead(ESP32_BUSY_PIN));

  // ── Raw SPI probe (bypasses WiFiNINA) ──────────────────────────────────────
  // Try every SPI mode × multiple clock speeds. Send the NINA START byte
  // (0xE0 = CMD_FLAG) and read 16 bytes. A live NINA slave must pull MISO low
  // at some point (ACK byte, padding, etc.) — dead silence across every
  // combination = chip isn't running NINA-FW. Also watch BUSY around each
  // transaction to see if CS assert triggers any reaction.
  SPI.begin();
  const uint32_t clocks[] = {8000000, 4000000, 1000000, 200000};
  const uint8_t  modes[]  = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};
  bool any_life = false;
  for (uint8_t m = 0; m < 4; m++) {
    for (uint8_t c = 0; c < 4; c++) {
      SPI.beginTransaction(SPISettings(clocks[c], MSBFIRST, modes[m]));
      int b_pre = digitalRead(ESP32_BUSY_PIN);
      digitalWrite(ESP32_CS_PIN, LOW);
      delayMicroseconds(200);
      int b_cs_low = digitalRead(ESP32_BUSY_PIN);
      uint8_t rx[16];
      rx[0] = SPI.transfer(0xE0);  // CMD_START
      for (int i = 1; i < 16; i++) rx[i] = SPI.transfer(0xFF);
      int b_after = digitalRead(ESP32_BUSY_PIN);
      digitalWrite(ESP32_CS_PIN, HIGH);
      SPI.endTransaction();
      bool all_zero = true, all_ff = true;
      for (int i = 0; i < 16; i++) {
        if (rx[i] != 0x00) all_zero = false;
        if (rx[i] != 0xFF) all_ff = false;
      }
      Serial.print(F("  MODE")); Serial.print(modes[m]);
      Serial.print(F(" @")); Serial.print(clocks[c]);
      Serial.print(F(" BUSY[pre/csL/post]=")); Serial.print(b_pre);
      Serial.print('/'); Serial.print(b_cs_low);
      Serial.print('/'); Serial.print(b_after);
      Serial.print(F("  MISO:"));
      for (int i = 0; i < 16; i++) {
        Serial.print(' ');
        if (rx[i] < 0x10) Serial.print('0');
        Serial.print(rx[i], HEX);
      }
      if (all_zero)      Serial.println(F("  [stuck-LOW]"));
      else if (all_ff)   Serial.println(F("  [stuck-HIGH]"));
      else { Serial.println(F("  ← LIFE")); any_life = true; }
      delay(20);
    }
  }
  Serial.print(F("Probe summary: "));
  Serial.println(any_life ? F("ESP32 SPI slave responded in at least one config.")
                          : F("No response in any SPI mode/speed — chip is not running NINA-FW."));

  WiFi.setPins(ESP32_CS_PIN, ESP32_BUSY_PIN, ESP32_RESETN_PIN, ESP32_GPIO0_PIN);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("FAIL: AirLift not detected."));
    Serial.println(F("  Check: CS/BUSY/RESET pin assignments above match your wiring."));
    Serial.println(F("  Check: AirLift VIN is powered (3.3V or 5V depending on module)."));
    Serial.println(F("  Check: SPI MISO/MOSI/SCK are wired (Teensy 4.1 = pins 12/11/13)."));
    haltBlinking();
  }

  String fw = WiFi.firmwareVersion();
  Serial.print(F("AirLift firmware: "));
  Serial.println(fw);
  Serial.print(F("Library expects:  "));
  Serial.println(WIFI_FIRMWARE_LATEST_VERSION);

  Serial.print(F("Creating AP \""));
  Serial.print(AP_SSID);
  Serial.println(F("\"..."));
  int ap = WiFi.beginAP(AP_SSID, AP_PASS);
  if (ap != WL_AP_LISTENING) {
    Serial.print(F("FAIL: beginAP returned status="));
    Serial.println(ap);
    haltBlinking();
  }

  delay(500);  // let the AP settle before reading IP
  server.begin();

  IPAddress ip = WiFi.localIP();
  Serial.print(F("AP up. SSID=\""));
  Serial.print(AP_SSID);
  Serial.print(F("\" IP="));
  Serial.print(ip);
  Serial.print(F(" TCP="));
  Serial.println(TCP_PORT);
  Serial.println(F("Listening for clients..."));
  Serial.println(F("Test: join WiFi, then `nc <ip> 4040` and type."));
}

void loop() {
  // Accept one client at a time — this is a smoke test, not a server.
  if (!client || !client.connected()) {
    WiFiClient incoming = server.available();
    if (incoming) {
      client = incoming;
      Serial.print(F("Client connected from "));
      Serial.println(client.remoteIP());
    }
  }

  // Echo bytes both ways and mirror to USB serial so you can see traffic
  // even without the laptop monitor in view.
  if (client && client.connected()) {
    while (client.available()) {
      char c = client.read();
      client.write(c);
      Serial.write(c);
      bytes_in++;
      bytes_out++;  // echo doubles output
    }

    // Periodic status push — dashboard parses these to show RSSI/uptime/rate.
    // Format: STATUS,uptime_ms,bytes_in,bytes_out,rssi_dbm
    // RSSI in AP mode is the signal of whichever station is connected; the
    // AirLift returns 0 if it can't read it, which the dashboard renders as "—".
    uint32_t now = millis();
    if (now - last_status_ms >= STATUS_INTERVAL_MS) {
      last_status_ms = now;
      int32_t rssi = WiFi.RSSI();  // dBm; 0 = unknown
      char buf[80];
      snprintf(buf, sizeof(buf), "STATUS,%lu,%lu,%lu,%ld\n",
               (unsigned long)now, (unsigned long)bytes_in,
               (unsigned long)bytes_out, (long)rssi);
      size_t n = client.print(buf);
      bytes_out += n;
    }
  }

  delay(5);
}
