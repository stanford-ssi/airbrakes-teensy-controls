#include "CommLink.h"

#include "WifiPins.h"

CommLink comm;

void CommLink::beginSerial(uint32_t baud) {
  Serial.begin(baud);
  Serial.setTimeout(_timeout_ms);
}

// Brings up the AirLift WiFi AP and opens a TCP listen socket.
// Returns false if the AirLift module isn't responding or beginAP fails;
// callers fall back to USB-only on false.
bool CommLink::beginWiFi(const char* ssid, const char* pass, uint16_t port) {
  // Manual ESP32 reset BEFORE handing control to WiFiNINA. The library's
  // internal reset is sometimes too fast on Teensy 4.1 — the ESP32 wakes
  // into an indeterminate state and the next WiFi.status() call reports
  // WL_NO_MODULE even when the wiring is fine. Pulsing RESETN ourselves
  // and waiting ~750 ms for NINA-FW to boot makes bringup deterministic.
  // Same lesson is exercised (with extra diagnostics) in wifi_bringup.cpp.
  pinMode(WifiPins::RESETN, OUTPUT);
  pinMode(WifiPins::CS,     OUTPUT);
  pinMode(WifiPins::GPIO0,  OUTPUT);
  digitalWrite(WifiPins::GPIO0, HIGH);   // strap for NINA-FW boot (not UART download)
  digitalWrite(WifiPins::CS,    HIGH);   // deassert CS so SPI bus is idle
  digitalWrite(WifiPins::RESETN, LOW);
  delay(50);
  digitalWrite(WifiPins::RESETN, HIGH);
  delay(750);                            // ESP32 NINA-FW boot time + margin

  WiFi.setPins(WifiPins::CS, WifiPins::BUSY, WifiPins::RESETN, WifiPins::GPIO0);

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

// Decode wl_status_t → short tag for the diagnostic line. Names match the
// WiFiNINA enum so cross-referencing wl_definitions.h is one-step.
static const __FlashStringHelper* wifiStatusName(uint8_t s) {
  switch (s) {
    case 0:   return F("IDLE");
    case 1:   return F("NO_SSID");
    case 2:   return F("SCAN_DONE");
    case 3:   return F("CONNECTED");
    case 4:   return F("CONNECT_FAILED");
    case 5:   return F("LOST");
    case 6:   return F("DISCONNECTED");
    case 7:   return F("AP_LISTENING");   // AP is up, no station associated
    case 8:   return F("AP_CONNECTED");   // AP is up AND a station has joined
    case 9:   return F("AP_FAILED");
    case 255: return F("NO_MODULE");
    default:  return F("?");
  }
}

// Accepts an incoming TCP client when no client is currently bound, emits a
// periodic AP/TCP heartbeat to USB so the operator can see which stage of
// "join SSID → DHCP → TCP/4040 → telemetry" is failing, and pushes a 1 Hz
// STATUS frame to the connected client so the dashboard's WiFi panel
// renders live RSSI / uptime / byte counters.
void CommLink::poll() {
  if (!_wifiUp) return;

  bool tcpNow = _client && _client.connected();
  if (!tcpNow) {
    WiFiClient incoming = _server.available();
    if (incoming) {
      _client = incoming;
      tcpNow = true;
      // Reset per-session counters so the dashboard's rate calc starts
      // clean on each connect, and force a STATUS frame on the next tick
      // so the panel populates immediately rather than waiting 1 s.
      _tcp_bytes_in = 0;
      _tcp_bytes_out = 0;
      _lastStatusMs = 0;
      Serial.print(F("CommLink: client connected from "));
      Serial.println(_client.remoteIP());
    }
  }
  // TCP drop event — separate log so the dashboard going away is visible
  // (previously only the connect side was logged).
  if (_lastTcpConnected && !tcpNow) {
    Serial.println(F("CommLink: client disconnected"));
  }
  _lastTcpConnected = tcpNow;

  // Status heartbeat. Print immediately on a wl_status change (catches
  // AP_LISTENING → AP_CONNECTED when a station joins, or AP_LISTENING →
  // AP_FAILED if the radio dies). Otherwise tick every _diag_interval_ms
  // while no TCP client is bound, so the user has a "still alive, still
  // waiting" signal on the USB serial monitor. Once a TCP client is bound,
  // heartbeat suppresses to keep USB quiet during a real session.
  uint8_t st = WiFi.status();
  unsigned long now = millis();
  bool changed = (st != _lastWifiStatus);
  bool dueForHeartbeat = !tcpNow && (now - _lastDiagMs >= _diag_interval_ms);
  if (changed || dueForHeartbeat) {
    _logWifiState(st);
    _lastDiagMs = now;
    _lastWifiStatus = st;
  }

  // STATUS,uptime_ms,bytes_in,bytes_out,rssi_dbm — same format wifi_bringup
  // emits, parsed by WiFiTransport.parse_status in the dashboard. Sent only
  // when a client is bound; goes directly to _client (NOT through the
  // buffered print path) so it doesn't pollute USB with 1/s diagnostic
  // spam during a real session.
  if (tcpNow && (now - _lastStatusMs >= _status_interval_ms)) {
    _lastStatusMs = now;
    int32_t rssi = WiFi.RSSI();  // dBm; AirLift returns 0 in AP mode if unknown
    char buf[80];
    int n = snprintf(buf, sizeof(buf), "STATUS,%lu,%lu,%lu,%ld\r\n",
                     (unsigned long)now,
                     (unsigned long)_tcp_bytes_in,
                     (unsigned long)_tcp_bytes_out,
                     (long)rssi);
    if (n > 0 && (size_t)n < sizeof(buf)) {
      _client.write(reinterpret_cast<const uint8_t*>(buf), (size_t)n);
      _tcp_bytes_out += (uint32_t)n;
    }
  }
}

// Drop-in replacement for delay() that keeps the AP responsive. Without this,
// blocking waits in setup() (sensor settle, SD retry, buzzer confirm) leave
// incoming TCP clients stuck in NINA-FW's backlog — the application-side
// _server.available() doesn't get called until loop() starts, which can be
// 15+ s into boot. The launch dashboard's connect-then-wait flow times out
// before that. Polling at ~50 ms during the wait pulls clients out of the
// backlog within one tick and lets the STATUS heartbeat begin.
void CommLink::pollDelay(unsigned long ms) {
  unsigned long start = millis();
  while (true) {
    poll();
    unsigned long elapsed = millis() - start;
    if (elapsed >= ms) break;
    unsigned long remaining = ms - elapsed;
    delay(remaining < 50 ? remaining : 50);
  }
}

void CommLink::_logWifiState(uint8_t status) {
  Serial.print(F("CommLink: wifi status="));
  Serial.print(status);
  Serial.print(F("("));
  Serial.print(wifiStatusName(status));
  Serial.print(F(") ip="));
  Serial.print(WiFi.localIP());
  Serial.print(F(" srv="));
  Serial.print(_server.status());
  Serial.print(F(" tcp="));
  Serial.println(_lastTcpConnected ? F("yes") : F("no"));
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
  // Chunk-flush so a payload larger than _usb_buf still gets through fully.
  // The previous version silently capped oversized writes at sizeof(_usb_buf);
  // not currently triggered (telemetry rows fit), but the chunked form has
  // no bad case.
  while (len > 0) {
    if (_usb_pos == sizeof(_usb_buf)) _usbFlush();
    size_t room = sizeof(_usb_buf) - _usb_pos;
    size_t n = (len < room) ? len : room;
    memcpy(_usb_buf + _usb_pos, s, n);
    _usb_pos += n;
    s += n;
    len -= n;
  }
}

void CommLink::_usbFlush() {
  if (_usb_pos == 0) return;
  Serial.write(reinterpret_cast<const uint8_t*>(_usb_buf), _usb_pos);
  // send_now() forces the CDC packet out — the auto-flush only triggers on
  // a full packet, so without this short lines sit buffered indefinitely.
  Serial.send_now();
  _usb_pos = 0;
}

void CommLink::_tcpWrite(const char* s, size_t len) {
  _tcpWriteAll(reinterpret_cast<const uint8_t*>(s), len);
}

// Single-source-of-truth TCP write with retry on backpressure. Long bulk
// transfers (SD log downloads) hit a stall where _client.write() returns 0
// once NINA-FW's TCP send buffer is full — the dashboard hasn't drained the
// last segment yet. Without retry the bytes are silently dropped and the
// transfer halts mid-file. Retrying with a 1 ms yield gives NINA-FW time to
// flush the buffer over the air; a 2 s no-progress deadline catches a truly
// stuck link.
size_t CommLink::_tcpWriteAll(const uint8_t* buf, size_t len) {
  if (!tcpConnected() || len == 0) return 0;
  size_t total = 0;
  unsigned long last_progress = millis();
  while (total < len) {
    if (!_client.connected()) break;
    size_t n = _client.write(buf + total, len - total);
    if (n > 0) {
      total += n;
      _tcp_bytes_out += (uint32_t)n;
      last_progress = millis();
      continue;
    }
    if (millis() - last_progress > 2000) break;
    delay(1);
  }
  return total;
}

size_t CommLink::_materializeF(const __FlashStringHelper* s, char* buf, size_t cap) {
  if (cap == 0) return 0;
  const char* p = reinterpret_cast<const char*>(s);
  size_t i = 0;
  while (i < cap - 1) {
    char c = pgm_read_byte(p + i);
    if (!c) break;
    buf[i++] = c;
  }
  buf[i] = '\0';
  return i;
}

// ── print / println — only println() and write() flush ──────────────────────

size_t CommLink::print(const char* s) {
  size_t slen = strlen(s);
  if (usbConnected()) _usbAppend(s, slen);
  _tcpWrite(s, slen);
  return slen;
}

size_t CommLink::print(const __FlashStringHelper* s) {
  // Materialise PROGMEM string once so both transports see identical bytes.
  char buf[160];
  size_t n = _materializeF(s, buf, sizeof(buf));
  if (usbConnected()) _usbAppend(buf, n);
  _tcpWrite(buf, n);
  return n;
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

// Combined body + CRLF into one TCP write so the AirLift sends a single
// segment per logical line. Two back-to-back small writes can hit a stutter
// on the NINA-FW SPI path that costs the second write (manifests as
// telemetry rows reaching the dashboard with no terminator, so the line
// reader hangs forever waiting for '\n' that never arrives — exactly the
// "link connected but no telem frames" symptom).
size_t CommLink::println(const char* s) {
  size_t slen = strlen(s);
  if (usbConnected()) {
    _usbAppend(s, slen);
    _usbAppend("\r\n", 2);
    _usbFlush();
  }
  if (tcpConnected()) {
    char tmp[320];  // sized for the longest telemetry row (≤256) + CRLF + headroom
    if (slen + 2 <= sizeof(tmp)) {
      memcpy(tmp, s, slen);
      tmp[slen]     = '\r';
      tmp[slen + 1] = '\n';
      _tcpWriteAll(reinterpret_cast<const uint8_t*>(tmp), slen + 2);
    } else {
      // Oversized payload — fall back to split writes; TCP still reassembles.
      _tcpWriteAll(reinterpret_cast<const uint8_t*>(s), slen);
      _tcpWriteAll(reinterpret_cast<const uint8_t*>("\r\n"), 2);
    }
  }
  return slen + 2;
}

size_t CommLink::println(const __FlashStringHelper* s) {
  char buf[160];
  size_t n = _materializeF(s, buf, sizeof(buf));
  if (usbConnected()) {
    _usbAppend(buf, n);
    _usbAppend("\r\n", 2);
    _usbFlush();
  }
  if (tcpConnected()) {
    char tmp[164];
    memcpy(tmp, buf, n);
    tmp[n]     = '\r';
    tmp[n + 1] = '\n';
    _tcpWriteAll(reinterpret_cast<const uint8_t*>(tmp), n + 2);
  }
  return n + 2;
}

size_t CommLink::println() {
  if (usbConnected()) {
    _usbAppend("\r\n", 2);
    _usbFlush();
  }
  _tcpWrite("\r\n", 2);
  return 2;
}

size_t CommLink::write(uint8_t b) {
  if (usbConnected()) {
    _usbAppend(reinterpret_cast<const char*>(&b), 1);
    _usbFlush();  // single-byte writes flush immediately (raw protocol bytes)
  }
  _tcpWrite(reinterpret_cast<const char*>(&b), 1);
  return 1;
}

size_t CommLink::write(const uint8_t* buf, size_t len) {
  if (usbConnected()) {
    _usbAppend(reinterpret_cast<const char*>(buf), len);
    _usbFlush();
  }
  _tcpWrite(reinterpret_cast<const char*>(buf), len);
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
  if (tcpConnected() && _client.available()) {
    int b = _client.read();
    if (b >= 0) _tcp_bytes_in++;
    return b;
  }
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
  // Drain the per-line USB accumulator FIRST so a buffered print() before
  // flush() actually reaches the wire — otherwise the buffered text sits
  // in _usb_buf and gets eaten by the next reset / link teardown. Then
  // wait for Serial's underlying TX queue to empty.
  if (usbConnected()) {
    _usbFlush();
    Serial.flush();
  }
  if (tcpConnected()) _client.flush();
}
