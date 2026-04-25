#include "CommLink.h"

// AirLift ESP32 pin assignments. Must match the wiring verified by
// src/wifi_bringup.cpp — if any of these change the bringup sketch needs
// updating in lockstep.
static constexpr int ESP32_CS_PIN     = 10;
static constexpr int ESP32_BUSY_PIN   = 17;
static constexpr int ESP32_RESETN_PIN = 16;
static constexpr int ESP32_GPIO0_PIN  = 15;

CommLink comm;

void CommLink::beginSerial(uint32_t baud) {
  Serial.begin(baud);
  Serial.setTimeout(_timeout_ms);
}

// Brings up the AirLift WiFi AP and opens a TCP listen socket.
// Returns false if the AirLift module isn't responding or beginAP fails;
// callers fall back to USB-only on false.
bool CommLink::beginWiFi(const char* ssid, const char* pass, uint16_t port) {
  // GPIO0 HIGH selects NINA-FW boot. Without this strap the AirLift can wake
  // into esptool passthrough mode and the library reports WL_NO_MODULE.
  pinMode(ESP32_GPIO0_PIN, OUTPUT);
  digitalWrite(ESP32_GPIO0_PIN, HIGH);

  WiFi.setPins(ESP32_CS_PIN, ESP32_BUSY_PIN, ESP32_RESETN_PIN, ESP32_GPIO0_PIN);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println(F("CommLink: AirLift not detected, USB only"));
    return false;
  }

  Serial.print(F("CommLink: AirLift firmware "));
  Serial.println(WiFi.firmwareVersion());

  int ap = WiFi.beginAP(ssid, pass);
  if (ap != WL_AP_LISTENING) {
    Serial.print(F("CommLink: beginAP failed, status="));
    Serial.println(ap);
    return false;
  }

  delay(300);  // let AP settle before querying IP / opening listen socket
  _server = WiFiServer(port);
  _server.begin();

  IPAddress ip = WiFi.localIP();
  Serial.print(F("CommLink: AP \""));
  Serial.print(ssid);
  Serial.print(F("\" up at "));
  Serial.print(ip);
  Serial.print(F(":"));
  Serial.println(port);

  _wifiUp = true;
  return true;
}

// Accepts an incoming TCP client when no client is currently bound. Cheap.
void CommLink::poll() {
  if (!_wifiUp) return;
  if (!_client || !_client.connected()) {
    WiFiClient incoming = _server.available();
    if (incoming) {
      _client = incoming;
      Serial.print(F("CommLink: client connected from "));
      Serial.println(_client.remoteIP());
    }
  }
}

bool CommLink::connected() {
  return usbConnected() || tcpConnected();
}

// ── USB-side line buffering ─────────────────────────────────────────────────
//
// All USB writes accumulate in _usb_buf and only flush on println() / write().
// This collapses a multi-print line ("MODE_ACK," + "SHITL\r\n") into one
// Serial.write() so Teensy 4.1's USB CDC doesn't drop the first byte of a
// follow-up packet that arrives before the host has drained the previous one.
// Without this we see truncated dashboard frames like "ATAREQUEST" or
// "ODE_ACK,SHITL". TCP doesn't need this — it already does its own framing.

void CommLink::_usbAppend(const char* s, size_t len) {
  if (_usb_pos + len >= sizeof(_usb_buf)) {
    _usbFlush();
    if (len > sizeof(_usb_buf)) len = sizeof(_usb_buf);  // soft cap
  }
  memcpy(_usb_buf + _usb_pos, s, len);
  _usb_pos += len;
}

void CommLink::_usbFlush() {
  if (_usb_pos == 0) return;
  Serial.write(reinterpret_cast<const uint8_t*>(_usb_buf), _usb_pos);
  // send_now() forces the CDC packet out — the auto-flush only triggers on
  // a full packet, so without this short lines sit buffered indefinitely.
  Serial.send_now();
  _usb_pos = 0;
}

// ── print / println — only println() and write() flush ──────────────────────

size_t CommLink::print(const char* s) {
  size_t slen = strlen(s);
  if (usbConnected()) _usbAppend(s, slen);
  if (tcpConnected()) _client.print(s);
  return slen;
}

size_t CommLink::print(const __FlashStringHelper* s) {
  // Materialise PROGMEM string once so both transports see identical bytes.
  const char* p = reinterpret_cast<const char*>(s);
  char buf[160];
  size_t i = 0;
  while (i < sizeof(buf) - 1) {
    char c = pgm_read_byte(p + i);
    if (!c) break;
    buf[i++] = c;
  }
  buf[i] = '\0';
  if (usbConnected()) _usbAppend(buf, i);
  if (tcpConnected()) _client.print(buf);
  return i;
}

size_t CommLink::print(float v, int decimals) {
  // Stringify locally so we append to the buffer instead of letting
  // Serial.print fragment the line into many small writes.
  char buf[24];
  dtostrf(v, 0, decimals, buf);
  return print(buf);
}

size_t CommLink::print(int v) {
  char buf[16];
  itoa(v, buf, 10);
  return print(buf);
}

size_t CommLink::print(unsigned long v) {
  char buf[16];
  ultoa(v, buf, 10);
  return print(buf);
}

size_t CommLink::println(const char* s) {
  size_t slen = strlen(s);
  if (usbConnected()) {
    _usbAppend(s, slen);
    _usbAppend("\r\n", 2);
    _usbFlush();
  }
  if (tcpConnected()) _client.println(s);
  return slen + 2;
}

size_t CommLink::println(const __FlashStringHelper* s) {
  const char* p = reinterpret_cast<const char*>(s);
  char buf[160];
  size_t i = 0;
  while (i < sizeof(buf) - 1) {
    char c = pgm_read_byte(p + i);
    if (!c) break;
    buf[i++] = c;
  }
  buf[i] = '\0';
  if (usbConnected()) {
    _usbAppend(buf, i);
    _usbAppend("\r\n", 2);
    _usbFlush();
  }
  if (tcpConnected()) _client.println(buf);
  return i + 2;
}

size_t CommLink::println() {
  if (usbConnected()) {
    _usbAppend("\r\n", 2);
    _usbFlush();
  }
  if (tcpConnected()) _client.println();
  return 2;
}

size_t CommLink::write(uint8_t b) {
  if (usbConnected()) {
    _usbAppend(reinterpret_cast<const char*>(&b), 1);
    _usbFlush();  // single-byte writes flush immediately (raw protocol bytes)
  }
  if (tcpConnected()) _client.write(b);
  return 1;
}

size_t CommLink::write(const uint8_t* buf, size_t len) {
  if (usbConnected()) {
    _usbAppend(reinterpret_cast<const char*>(buf), len);
    _usbFlush();
  }
  if (tcpConnected()) _client.write(buf, len);
  return len;
}

int CommLink::available() {
  int n = 0;
  if (tcpConnected()) n += _client.available();
  if (usbConnected()) n += Serial.available();
  return n;
}

int CommLink::read() {
  // TCP wins when both have data: once a WiFi dashboard is attached, that
  // is the control channel.
  if (tcpConnected() && _client.available()) return _client.read();
  if (usbConnected() && Serial.available()) return Serial.read();
  return -1;
}

// Manual line-terminator read with a unified timeout across both transports.
// Can't use client.readBytesUntil directly because we want to interleave
// Serial reads with TCP reads on the same deadline.
size_t CommLink::readBytesUntil(char terminator, char* buf, size_t len) {
  size_t pos = 0;
  unsigned long start = millis();
  while (pos < len) {
    int c = read();
    if (c < 0) {
      if (millis() - start >= _timeout_ms) break;
      delay(0);  // tiny yield so we don't spin during the timeout window
      continue;
    }
    if (c == terminator) break;
    buf[pos++] = (char)c;
  }
  return pos;
}

void CommLink::setTimeout(unsigned long ms) {
  _timeout_ms = ms;
  Serial.setTimeout(ms);
  // WiFiClient has its own setTimeout but we run the read loop manually,
  // so _timeout_ms is the source of truth.
}

void CommLink::flush() {
  if (usbConnected()) Serial.flush();
  if (tcpConnected()) _client.flush();
}
