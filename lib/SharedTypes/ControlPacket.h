#pragma once
#include <stdint.h>
#include <stddef.h>

// ControlPacket: STM32 → Teensy (26 bytes)
// Sent by STM32 master via I2C write to slave address 0x42
struct ControlPacket
{
    uint32_t time_ms;     // 4 bytes
    float pressure;       // 4 bytes (hPa)
    float temperature;    // 4 bytes (°C)
    float accel_z_low_g;  // 4 bytes (ADXL345, g's)
    float accel_z_high_g; // 4 bytes (ADXL375, g's)
    float baro_altitude;  // 4 bytes (meters AGL)
    uint8_t flight_state; // 1 byte
    uint8_t crc;          // 1 byte
} __attribute__((packed));

// CommandPacket: Teensy → STM32 (18 bytes → 16 bytes after potentiometer removal)
// Served by Teensy slave via I2C onRequest (STM32 calls requestFrom)
struct CommandPacket
{
    float servo_angle_1;          // 4 bytes (percentage 0-100)
    float servo_angle_2;          // 4 bytes (percentage 0-100)
    float cd_add_cmd;             // 4 bytes (additional Cd commanded)
    float predicted_apogee;       // 4 bytes (meters AGL)
    uint8_t controller_state;     // 1 byte (0=idle, 1=active, 2=retracted, 3=full)
    uint8_t crc;                  // 1 byte
} __attribute__((packed));

inline uint8_t computeCRC(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++)
        crc ^= data[i];
    return crc;
}
