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

  // Pump the TCP accept + client-alive bookkeeping. Call once per main loop
  // tick. Cheap when nothing is happening.
  void poll();

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
  void flush();

  WiFiClient& client() { return _client; }

 private:
  // Per-line USB-side accumulator. print() appends; println() / write() flush
  // it as one Serial.write() so a logical message goes out as a single USB
  // CDC packet. See class comment for why this matters.
  char _usb_buf[288];
  size_t _usb_pos = 0;
  void _usbAppend(const char* s, size_t len);
  void _usbFlush();

  WiFiServer _server{0};   // re-assigned in beginWiFi()
  WiFiClient _client;
  bool _wifiUp = false;
  unsigned long _timeout_ms = 1000;
};

extern CommLink comm;
