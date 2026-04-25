#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>

// CommLink: dual-transport line protocol bridge.
//
// Wraps Serial (USB CDC) and a WiFiClient (TCP socket served by the on-board
// AirLift ESP32) behind a stream-like API so the rest of the firmware can
// pretend it's just talking to one Serial port. Either transport can deliver
// bytes in (whichever has data) and writes fan out to BOTH transports so USB
// mirroring works for bench debugging.
//
// Reads prefer TCP — once a WiFi dashboard connects, that is the control
// channel. Writes are buffered per logical line on the USB side because
// Teensy 4.1 USB CDC drops the first byte of consecutive sub-millisecond
// packets when the host hasn't drained the previous one yet (this manifests
// as garbled "ATAREQUEST" / "ODE_ACK" on the dashboard).
class CommLink {
 public:
  void beginSerial(uint32_t baud);

  // Brings up the AirLift in AP mode and opens a TCP listen socket. Returns
  // true on success. Safe to call when the AirLift is missing or flaky:
  // failures are logged to Serial and CommLink degrades to USB-only.
  bool beginWiFi(const char* ssid, const char* pass, uint16_t port);

  // Pump the TCP accept + client-alive bookkeeping + 1 Hz STATUS push.
  // Call once per main loop tick. Cheap when nothing is happening.
  void poll();

  // Same shape as Arduino delay(ms) but calls poll() at ~50 ms cadence
  // throughout. Use during long setup() waits so an early-connecting
  // dashboard gets bound and starts receiving data within a tick instead
  // of sitting in NINA-FW's TCP backlog until setup() returns.
  void pollDelay(unsigned long ms);

  // True if ANY transport can currently deliver a byte. Used as the gate on
  // debug-log writes so a dropped cable / dropped WiFi never blocks the loop.
  bool connected();
  bool tcpConnected() { return _client && _client.connected(); }
  bool usbConnected() { return (bool)Serial; }

  // Stream-like API matching the Arduino Serial / WiFiClient surface closely
  // enough that existing call sites compile after s/Serial/comm/.
  size_t print(const char* s);
  size_t print(const __FlashStringHelper* s);
  size_t print(float v, int decimals = 2);
  size_t print(int v);
  size_t print(unsigned long v);
  size_t println(const char* s);
  size_t println(const __FlashStringHelper* s);
  size_t println();
  size_t write(uint8_t b);
  size_t write(const uint8_t* buf, size_t n);

  int available();
  int read();
  // Reads until `terminator` or until setTimeout()'s deadline expires.
  // Reads prefer TCP when a byte is available there, otherwise Serial.
  size_t readBytesUntil(char terminator, char* buf, size_t len);

  void setTimeout(unsigned long ms);

  // Drains the per-line USB accumulator AND the underlying Serial TX queue,
  // and the TCP send queue. Call before anything that resets the MCU or
  // tears the link down — without the _usb_buf drain, a buffered print()
  // before flush() silently disappears.
  void flush();

 private:
  // Per-line USB-side accumulator. print() appends; println() / write() flush
  // it as one Serial.write() so a logical message goes out as a single USB
  // CDC packet. See class comment for why this matters.
  char _usb_buf[288];
  size_t _usb_pos = 0;
  void _usbAppend(const char* s, size_t len);
  void _usbFlush();

  // Mirrors a buffer to the connected TCP client and updates the byte
  // counter used by the STATUS heartbeat. No-op when no client is bound.
  void _tcpWrite(const char* s, size_t len);

  // Robust TCP write: retries on partial / 0-byte writes until either all
  // `len` bytes are accepted by the AirLift, the link drops, or 2 s elapses
  // without progress. WiFiNINA's _client.write() returns 0 when NINA-FW's
  // TCP send buffer is full; without retry that chunk gets silently dropped
  // and bulk transfers (SD downloads) stall partway through. Returns the
  // number of bytes actually written.
  size_t _tcpWriteAll(const uint8_t* buf, size_t len);

  // Materializes a PROGMEM string into a stack buffer and returns its
  // length (not counting the trailing NUL). Used by print(F)/println(F).
  size_t _materializeF(const __FlashStringHelper* s, char* buf, size_t cap);

  WiFiServer _server{0};   // re-assigned in beginWiFi()
  WiFiClient _client;
  bool _wifiUp = false;
  unsigned long _timeout_ms = 1000;

  // ── Diagnostics & status ────────────────────────────────────────────
  // Heartbeat to USB serial monitor every _diag_interval_ms when no TCP
  // client is bound, plus an immediate log on WiFi.status() change or TCP
  // connect/disconnect. Without these the firmware silently rides on
  // whatever state beginWiFi() saw at boot, so a dropped AP or a station
  // that joined but can't reach TCP/4040 looks identical to "all good".
  unsigned long _lastDiagMs = 0;
  uint8_t       _lastWifiStatus = 255;  // 255 = WL_NO_MODULE sentinel; first poll forces a log
  bool          _lastTcpConnected = false;
  static constexpr unsigned long _diag_interval_ms = 3000;
  void _logWifiState(uint8_t status);

  // STATUS,uptime_ms,bytes_in,bytes_out,rssi_dbm pushed to the connected
  // TCP client at _status_interval_ms cadence. The dashboard parses this
  // to populate its WiFi panel; without these frames RSSI / byte counters
  // / fw_uptime_ms render as blanks.
  uint32_t _tcp_bytes_in = 0;
  uint32_t _tcp_bytes_out = 0;
  unsigned long _lastStatusMs = 0;
  static constexpr unsigned long _status_interval_ms = 1000;
};

extern CommLink comm;
