#pragma once

#include <stdint.h>

// AirLift ESP32 control pin assignments (Teensy 4.1).
//
// SPI bus pins (MOSI=11, MISO=12, SCK=13) are the default Teensy SPI peripheral
// (LPSPI4 on the top edge) and are wired implicitly by SPI.begin(); the four
// pins below are the additional control lines on top of the bus.
//
// Single source of truth for the production firmware (CommLink) AND the
// standalone wifi_bringup smoke test. If wiring ever changes, update here and
// both build envs pick it up — previously these were redefined in three
// places and would silently drift.
namespace WifiPins {
  constexpr int CS     = 10;
  constexpr int BUSY   = 17;
  constexpr int RESETN = 16;
  constexpr int GPIO0  = 15;
}

// Default AP credentials and TCP port for the rocket-side dashboard link.
// The Python dashboard (Airbrakes_SHITL/shitl_dashboard.py) defaults to
// 192.168.4.1:4040 — keep PORT in lockstep with WiFiTransport.DEFAULT_PORT
// over there.
namespace WifiAP {
  constexpr const char* SSID = "AirbrakesRocket";
  constexpr const char* PASS = "irec202630k";   // WPA2 minimum 8 chars
  constexpr uint16_t    PORT = 4040;
}
