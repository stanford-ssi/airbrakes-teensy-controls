#include <Arduino.h>
#include <Wire.h>

#include <ControlPacket.h>
#include <I2CSlave.h>
#include <UKF1D.h>
#include <SensorWeighting.h>
#include <AirbrakeController.h>
#include <CdLookup.h>
#include <Atmosphere.h>
#include <RocketConfig.h>

I2CSlave i2c(0x42);
UKF1D ukf;
AirbrakeController controller;

uint32_t last_time_ms = 0;
uint32_t loop_count = 0;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Airbrake Controller Teensy 4.1");
    Serial.println("Waiting for I2C packets from STM32...");

    Wire.begin(0x42);
    i2c.begin();

    // Set initial command to retracted
    CommandPacket cmd = {};
    cmd.servo_angle_1 = RocketConfig::SERVO_MIN_PCT;
    cmd.servo_angle_2 = RocketConfig::SERVO_MIN_PCT;
    cmd.cd_add_cmd = 0.0f;
    cmd.predicted_apogee = 0.0f;
    cmd.controller_state = AirbrakeController::CTRL_IDLE;
    cmd.crc = computeCRC((const uint8_t *)&cmd, sizeof(cmd) - 1);
    i2c.setCommand(cmd);
}

void loop() {
    if (!i2c.hasNewPacket()) return;

    ControlPacket pkt = i2c.getPacket();

    // Validate CRC
    uint8_t expected_crc = computeCRC((const uint8_t *)&pkt, sizeof(pkt) - 1);
    if (pkt.crc != expected_crc) {
        Serial.println("CRC mismatch, dropping packet");
        return;
    }

    // Compute dt
    float dt;
    if (last_time_ms == 0) {
        dt = 0.05f; // Assume 50ms on first packet
    } else {
        dt = (pkt.time_ms - last_time_ms) / 1000.0f;
        if (dt <= 0.0f || dt > 1.0f) dt = 0.05f; // Sanity check
    }
    last_time_ms = pkt.time_ms;

    // Initialize UKF on first valid packet
    if (!ukf.isInitialized()) {
        ukf.init(pkt.baro_altitude, 0.0f, 0.0f);
        Serial.print("UKF initialized at alt=");
        Serial.println(pkt.baro_altitude);
    }

    // UKF predict
    ukf.predict(dt);

    // Get sensor weights based on flight state and estimated velocity
    float alt_msl = ukf.altitude() + RocketConfig::LAUNCH_SITE_ALT_MSL_M;
    float sos = Atmosphere::speedOfSound(alt_msl);
    SensorWeights weights = SensorWeighting::getWeights(
        pkt.flight_state, ukf.velocity(), sos);

    // Convert accelerometer readings from g's to m/s²
    float accel_low_ms2 = pkt.accel_z_low_g * RocketConfig::G_TO_MS2;
    float accel_high_ms2 = pkt.accel_z_high_g * RocketConfig::G_TO_MS2;

    // UKF measurement updates
    ukf.updateAccel(accel_low_ms2, weights.R_accel_low);
    ukf.updateAccel(accel_high_ms2, weights.R_accel_high);
    ukf.updateBaro(pkt.baro_altitude, weights.R_baro);

    // Run airbrake controller
    float time_s = pkt.time_ms / 1000.0f;
    float cd_add = controller.update(
        ukf.altitude(), ukf.velocity(), ukf.acceleration(),
        pkt.flight_state, sos, time_s, dt);

    // Convert Cd to servo angles
    ServoAngles angles = CdLookup::cdToServoAngles(cd_add);

    // Build command packet
    CommandPacket cmd;
    cmd.servo_angle_1 = angles.angle_1;
    cmd.servo_angle_2 = angles.angle_2;
    cmd.cd_add_cmd = cd_add;
    cmd.predicted_apogee = controller.predictedApogee();
    cmd.controller_state = static_cast<uint8_t>(controller.controllerState());
    cmd.crc = computeCRC((const uint8_t *)&cmd, sizeof(cmd) - 1);

    i2c.setCommand(cmd);

    // Debug output every 10 packets (~500ms)
    if (++loop_count % 10 == 0) {
        Serial.print("t=");
        Serial.print(pkt.time_ms);
        Serial.print(" alt=");
        Serial.print(ukf.altitude(), 1);
        Serial.print(" vel=");
        Serial.print(ukf.velocity(), 1);
        Serial.print(" acc=");
        Serial.print(ukf.acceleration(), 1);
        Serial.print(" cd=");
        Serial.print(cd_add, 3);
        Serial.print(" apogee=");
        Serial.print(controller.predictedApogee(), 0);
        Serial.print(" s1=");
        Serial.print(angles.angle_1, 1);
        Serial.print(" st=");
        Serial.println(pkt.flight_state);
    }
}
