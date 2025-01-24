#ifndef ESC_BASE_H
#define ESC_BASE_H

#include <Arduino.h>
#include "sp140/structs.h"
#include "EscData.h"

class EscBase {

public:
    virtual void initESC(int escPin) = 0;
    virtual void setupESCSerial() = 0;
    virtual void setESCThrottle(int throttlePWM) = 0;
    virtual void readESCTelemetry(BatteryData& batteryData, EscData& escData) = 0;
    virtual void handleESCSerialData(BatteryData& batteryData, EscData& escData, byte buffer[]) = 0;
    virtual void prepareESCSerialRead() = 0;
    virtual int checkFletcher16(byte byteBuffer[]) = 0;
};



#endif