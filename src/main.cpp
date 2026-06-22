//
// Created by Emad Muzaffar on 6/18/26.
//

#include <Arduino.h>
#include <ArCom/ArCOM.h>
#include <ArCom/ArCOM.cpp>


class Motor {
    //Constants
    float MaxMotorRPM = 30; //RPM used for conversion math, should be set in the clearPath application
    float MaxPWM = 65535;

    //Internal variables for motor to store data
    int directionPin;
    int pwmPin;
    int enablePin;
    float target = 0;
    unsigned long startTime = 0;
    uint8_t time = 0;

    /*
    float getTarget(const int velocity, const int time, const int direction) const {
        if (direction == 0) {
            return target - (static_cast<float>(velocity) * static_cast<float>(time));
        }
        return target + (static_cast<float>(velocity) * static_cast<float>(time));
    }
    */

    void setTimer(const uint8_t timerLength) {
        startTime = millis();
        this->time = timerLength;
    }

    boolean updateTimer() const {
        if (millis() > startTime + time) return true;
        return false;
    }

    void setPWM(const int velocity) {
        startTime = millis();
        u_int16_t tRPM = static_cast<u_int16_t>(velocity) / 6; //simplified conversion from deg/s to rpm
        if (tRPM > static_cast<u_int16_t>(MaxMotorRPM)) {
            tRPM = static_cast<u_int16_t>(MaxMotorRPM);
        }
        if (tRPM < 0) {
            tRPM = 0;
        }
        analogWrite(this->pwmPin, 65535 * (tRPM/static_cast<u_int16_t>(MaxMotorRPM)));
    }

    void setDirection(int direction) const {
        if (direction == 0) {
            digitalWrite(directionPin, HIGH);
        } else if (direction == 1) {
            digitalWrite(directionPin, LOW);
        }
    }

    void pwmStop(boolean stop = false) const {
        if (stop) {
            analogWrite(this->pwmPin, 0);
            return;
        } else return;
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

    void update() {
        pwmStop(updateTimer());
        //add safety update here
    }

    void update(uint8_t speed, uint8_t time, uint8_t direction) {
        setDirection(direction);
        setPWM(speed);
        setTimer(time);
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
            while (Serial1COM.available() < 3) {
                // wait for full array
            }
            Serial1COM.readByteArray(buffer, 3);
            uint8_t speed  = buffer[0];
            uint8_t time = buffer[1];
            uint8_t direction = buffer[2];
            motor.update(speed, time, direction);
        }
    }
    motor.update();
}
