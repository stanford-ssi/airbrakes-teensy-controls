#include "I2CSlave.h"
#include <string.h>

I2CSlave *I2CSlave::instance = nullptr;

I2CSlave::I2CSlave(uint8_t address) : address_(address), newPacket_(false) {
    memset((void *)rxBuf_, 0, sizeof(rxBuf_));
    memset(txBuf_, 0, sizeof(txBuf_));
    instance = this;
}

void I2CSlave::begin() {
    Wire.begin(address_);
    Wire.onReceive(onReceiveISR);
    Wire.onRequest(onRequestISR);
}

bool I2CSlave::hasNewPacket() {
    return newPacket_;
}

ControlPacket I2CSlave::getPacket() {
    ControlPacket pkt;
    noInterrupts();
    memcpy(&pkt, (const void *)rxBuf_, sizeof(ControlPacket));
    newPacket_ = false;
    interrupts();
    return pkt;
}

void I2CSlave::setCommand(const CommandPacket &cmd) {
    noInterrupts();
    memcpy(txBuf_, &cmd, sizeof(CommandPacket));
    interrupts();
}

void I2CSlave::onReceiveISR(int numBytes) {
    if (!instance) return;
    if ((size_t)numBytes == sizeof(ControlPacket)) {
        for (size_t i = 0; i < sizeof(ControlPacket); i++) {
            ((volatile uint8_t *)instance->rxBuf_)[i] = Wire.read();
        }
        instance->newPacket_ = true;
    } else {
        // Drain unexpected bytes
        while (Wire.available()) Wire.read();
    }
}

void I2CSlave::onRequestISR() {
    if (!instance) return;
    Wire.write(instance->txBuf_, sizeof(CommandPacket));
}
