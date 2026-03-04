#pragma once
#include <Wire.h>
#include <ControlPacket.h>

class I2CSlave {
public:
    I2CSlave(uint8_t address = 0x42);

    void begin();
    bool hasNewPacket();
    ControlPacket getPacket();
    void setCommand(const CommandPacket &cmd);

    static I2CSlave *instance;

private:
    uint8_t address_;
    volatile bool newPacket_;

    // Double-buffered: ISR writes to rx_, main loop reads copy
    volatile uint8_t rxBuf_[sizeof(ControlPacket)];
    uint8_t txBuf_[sizeof(CommandPacket)];

    static void onReceiveISR(int numBytes);
    static void onRequestISR();
};
