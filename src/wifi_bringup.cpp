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

// ─── Pin assignments ─────────────────────────────────────────────────────────
// SPI lines (MOSI=11, MISO=12, SCK=13) are implicit — they're whatever the
// default `SPI` peripheral uses on Teensy 4.1 (LPSPI4 on the top edge). The
// AirLift breakout's MOSI/MISO/SCK pads must be wired to those Teensy pins
// for hardware SPI to work. The pins below are the four control lines on top
// of that.
constexpr int ESP32_CS_PIN     = 40;  // any GPIO works for CS; PinDefinitions still says 10
                                      // — update there once main firmware uses WiFi.
constexpr int ESP32_BUSY_PIN   = 39;
constexpr int ESP32_RESETN_PIN = 38;
constexpr int ESP32_GPIO0_PIN  = -1;  // not connected; AirLift's internal pull-up
                                      // holds boot mode strap HIGH for normal run

// ─── WiFi config ─────────────────────────────────────────────────────────────
// AP mode: the rocket *is* the network. Anyone within ~30 m can see the SSID,
// so keep WPA2 enabled (8+ char password). Change these to whatever you like.
const char* AP_SSID = "AirbrakesRocket";
const char* AP_PASS = "irec202630k";  // WPA2 minimum 8 chars
constexpr uint16_t TCP_PORT = 4040;

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

  WiFi.setPins(ESP32_CS_PIN, ESP32_BUSY_PIN, ESP32_RESETN_PIN, ESP32_GPIO0_PIN);
  SPI.begin();

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
