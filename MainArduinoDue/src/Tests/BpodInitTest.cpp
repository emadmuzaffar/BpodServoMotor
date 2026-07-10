/*
#include <Arduino.h>
#include "ArCom/ArCOM.h"

void returnModuleInfo();

constexpr uint8_t kLedPin = LED_BUILTIN;

unsigned long FirmwareVersion = 1;
char moduleName[] = "ServoMotor";

ArCOM Serial1COM(Serial1);

byte opCode = 0;

void blinkLed(uint8_t blinkCount, unsigned int onMs = 100, unsigned int offMs = 100) {
    for (uint8_t i = 0; i < blinkCount; i++) {
        digitalWrite(kLedPin, HIGH);
        delay(onMs);
        digitalWrite(kLedPin, LOW);
        delay(offMs);
    }
}

void setup() {
    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, LOW);

    Serial1.begin(1312500);

    blinkLed(3, 100, 100);
}

void loop() {
    if (Serial1COM.available()) {
        opCode = Serial1COM.readByte();

        switch (opCode) {
            case 255:
                returnModuleInfo();     // answer Bpod immediately
                Serial1.flush();        // wait until reply is sent
                blinkLed(5, 50, 50);    // short confirmation blink
                break;

            default:
                blinkLed(opCode == 0 ? 1 : opCode);
                break;
        }
    }
}

void returnModuleInfo() {
    Serial1COM.writeByte(65);                         // acknowledge
    Serial1COM.writeUint32(FirmwareVersion);          // firmware version
    Serial1COM.writeByte(sizeof(moduleName) - 1);     // name length
    Serial1COM.writeCharArray(moduleName, sizeof(moduleName) - 1);
    Serial1COM.writeByte(0);                          // no extra info
}
*/