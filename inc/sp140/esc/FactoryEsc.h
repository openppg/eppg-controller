#ifndef FACTORY_ESC_H
#define FACTORY_ESC_H

#include <Servo.h>
#include <CircularBuffer.hpp>
#include "EscBase.h"
#include "EscData.h"

// ESC communication parameters
#define ESC_BAUD_RATE         115200
#define ESC_DATA_V2_SIZE      22
#define READ_INTERVAL         0
#define ESC_TIMEOUT           15

class FactoryEsc : public EscBase {
public:
    static void printRawSentence(byte buffer[]);

    FactoryEsc(CircularBuffer<float, 50>* pVoltageBuffer);

    virtual void initESC(int escPin);
    virtual void setupESCSerial();
    virtual void setESCThrottle(int throttlePWM);
    virtual void readESCTelemetry(BatteryData& batteryData, EscData& escData);
    virtual void handleESCSerialData(BatteryData& batteryData, EscData& escData, byte buffer[]);
    virtual void prepareESCSerialRead();
    virtual int checkFletcher16(byte byteBuffer[]);
private: 

    Servo esc;
    CircularBuffer<float, 50>* pVoltageBuffer;
};


#endif