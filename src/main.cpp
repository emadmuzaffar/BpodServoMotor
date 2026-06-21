//
// Created by Emad Muzaffar on 6/18/26.
//

#include <Arduino.h>
#include <ArCom/ArCOM.h>


class Motor {
private:
    //Internal field for motor to store data
    int directionPin;
    int pwmPin;
    int enablePin;
    float target = 0;

    float getTarget(int param1, int param2, int direction) const {
        if (direction == 0) {
            return target - (static_cast<float>(param1) * static_cast<float>(param2));
        }
        return target + (static_cast<float>(param1) * static_cast<float>(param2));
    }

    float countEncoder() {
        
    }

public:
    explicit Motor(int directionPin, int pwmPin, int enablePin) {
        this->directionPin = directionPin;
        this->pwmPin = pwmPin;
        this->enablePin = enablePin;
        pinMode(directionPin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
        pinMode(enablePin, OUTPUT);
    }

    void enable() const {
        digitalWrite(this->enablePin, HIGH);
    }

    void disable() const {
        digitalWrite(this->enablePin, LOW);
    }


};


/**
 * Initialize objects and module setup
 * @author Emad Muzaffar
 */
unsigned long FirmwareVersion = 1;
char moduleName[] = "ServoMotor"; // ModuleName sent to Bpod in returnModuleInfo()
ArCOM Serial1COM(Serial1); // NOLINT(*-interfaces-global-init)
Motor motor(2,0,3);
byte opCode = 0; //Code to specify instruction type

/**
 * From SanWorks example
 * Writes info about arduino system over serial to Bpod
 * Called in switch(opCode) when opCode == 255, when Bpod scans for modules
 * @author JSNeuroDev
 */
void returnModuleInfo() {
    Serial1COM.writeByte(65); // Acknowledge
    Serial1COM.writeUint32(FirmwareVersion); // 4-byte firmware version
    Serial1COM.writeByte(sizeof(moduleName)-1); // Length of module name
    Serial1COM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
    Serial1COM.writeByte(0); // 1 if more info follows, 0 if not
}


void setup() {
    Serial1.begin(1312500); //specific baud rate for Bpod
    analogWriteResolution(16);
}

/**
 * Main loop
 * @author Emad Muzaffar
*/
void loop() {
    if (Serial1COM.available()) {
        opCode = Serial1COM.readByte();
        if (opCode == 255) {
            returnModuleInfo();
        } else if (opCode == 9) {
            //Turn enable off
            motor.disable();
        } else if (opCode == 10) {
            //Turn enable on
            motor.enable();
        } else if (opCode == 1) {
            u_int8_t buffer[3];
            while (Serial1COM.available() < 4) {
                // wait for full array
            }
            Serial1COM.readByteArray(buffer, 3);
            uint8_t speed  = buffer[0];
            uint8_t time = buffer[1];
            uint8_t direction = buffer[2];
            // motor.update(speed, time, direction)
        }
    }
    // motor.update()
}
