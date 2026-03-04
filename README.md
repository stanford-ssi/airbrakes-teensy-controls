# Airbrakes Teensy Controller

Airbrake control co-processor for SSI/IREC rocket avionics. Runs on Teensy 4.1, receives sensor data from the STM32 flight controller over I2C, and returns servo commands to achieve target apogee altitude.

## Overview

This firmware implements a closed-loop airbrake controller that:

1. **Fuses** barometric altitude and dual accelerometer (ADXL345 / ADXL375) data via a 1D Unscented Kalman Filter
2. **Predicts** apogee using RK4 trajectory simulation with altitude-dependent atmosphere
3. **Commands** servo angles via a Cd lookup table to hit target altitude (20,000 ft AGL)

The controller runs as an I2C slave at address `0x42`. The STM32 master sends sensor packets and reads back servo commands each loop.

## Architecture

```
STM32 Flight Controller (Master)
├── Sensors: ADXL345, ADXL375, LPS22 (baro), BNO080
├── I2C master @ 0x42
└── Sends: ControlPacket (sensor data)
    Receives: CommandPacket (servo angles, predicted apogee)
         ↓
Teensy 4.1 (Slave)
├── UKF1D — state estimation [alt, vel, accel]
├── SensorWeighting — dynamic R based on flight phase / Mach
├── ApogeePredictor — RK4 coast simulation, binary search for Cd
├── AirbrakeController — rule-based state machine
├── CdLookup — Cd_add → servo angle mapping
└── Atmosphere — US Standard Atmosphere 1976
```

## Requirements

- [PlatformIO](https://platformio.org/)
- [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) (for Teensy platform support)

## Build & Upload

```bash
cd airbrakes-teensy-controls

# Build
pio run

# Build and upload to Teensy
pio run --target upload

# Serial monitor (115200 baud)
pio device monitor

# Clean build
pio run --target clean
```

Update `upload_port` in `platformio.ini` if your Teensy uses a different device path.

## I2C Protocol

### ControlPacket (STM32 → Teensy, 26 bytes)

| Field | Type | Description |
|-------|------|-------------|
| `time_ms` | uint32_t | Timestamp (ms) |
| `pressure` | float | Barometric pressure (hPa) |
| `temperature` | float | Temperature (°C) |
| `accel_z_low_g` | float | ADXL345 Z-axis (g's) |
| `accel_z_high_g` | float | ADXL375 Z-axis (g's) |
| `baro_altitude` | float | Baro altitude AGL (m) |
| `flight_state` | uint8_t | States enum |
| `crc` | uint8_t | XOR checksum |

### CommandPacket (Teensy → STM32, 18 bytes)

| Field | Type | Description |
|-------|------|-------------|
| `servo_angle_1` | float | Servo 1 position (0–100%) |
| `servo_angle_2` | float | Servo 2 position (0–100%) |
| `cd_add_cmd` | float | Commanded additional Cd |
| `predicted_apogee` | float | Predicted apogee AGL (m) |
| `controller_state` | uint8_t | 0=idle, 1=active, 2=retracted, 3=full |
| `crc` | uint8_t | XOR checksum |

## Library Structure

| Module | Description |
|--------|-------------|
| `lib/StateEstimation/UKF1D` | 1D Unscented Kalman Filter: state = [altitude, velocity, acceleration] |
| `lib/StateEstimation/SensorWeighting` | Sensor noise (R) based on flight state and Mach number |
| `lib/Controller/AirbrakeController` | Rule-based controller: retract during burn/supersonic, run predictor during coast |
| `lib/Controller/ApogeePredictor` | RK4 trajectory simulation with binary search for Cd_add |
| `lib/LookupTable/CdLookup` | Cd_add → servo angle interpolation (linear placeholder; replace with wind tunnel CFD data) |
| `lib/Atmosphere/Atmosphere` | US Standard Atmosphere 1976 (density, speed of sound) |
| `lib/Comms/I2CSlave` | I2C slave with double-buffered receive/transmit |
| `lib/RocketParams/RocketConfig.h` | Mass, Cd, target altitude, launch site, Mach limits |

## Controller Logic

| Condition | Action |
|-----------|--------|
| Motor burning (BOOT..IGNITION) | Retract |
| Supersonic (Mach > 1.0) | Retract (structural safety) |
| Descending, post-apogee delay elapsed | Retract |
| Above target, still ascending | Full brakes |
| Normal coast | Run predictor, binary search for Cd_add |
| Slew rate limit | Cd change capped at 1.5 Cd/s |

## Configuration

Edit `lib/RocketParams/RocketConfig.h` for:

- **Target altitude:** `TARGET_ALT_AGL_M` (default 6096 m / 20,000 ft)
- **Launch site:** `LAUNCH_SITE_ALT_MSL_M` (default 1400 m, Spaceport America)
- **Mass, Cd, reference area:** `MASS_KG`, `BODY_CD`, `MAX_CD`, `REF_AREA_M2`
- **Servo limits:** `SERVO_MIN_PCT`, `SERVO_MAX_PCT` (10–60%)

## Cd Lookup Table

`lib/LookupTable/CdLookup.cpp` currently uses a linear placeholder mapping. Replace `cd_table` and `angle_table` with wind tunnel or CFD data when available.

## Related Projects

- **analysis/** — Python offline validation of UKF against flight logs (`ukf_test.py`)
- **Flying-phoque-airbrakes/** — STM32 flight controller (sensors, state machine, I2C master)

## License

Internal use for SSI/IREC competition.
