//
// Created by Emad Muzaffar on 6/18/26.
//

#include <Arduino.h>
#include <chrono>
#include <ArCom/ArCOM.h>

constexpr uint8_t kDirectionPin = 2;
constexpr uint8_t kPwmPin = 6;
constexpr uint8_t kEnablePin = 3;
constexpr uint8_t kStatusLedPin = 13;
constexpr float kMaxMotorRPM = 30.0f; // Must match the ClearPath MSP setup.
constexpr uint16_t kMaxPwm = 65535;
constexpr byte kModuleInfoOpCode = 255;
constexpr byte kMotorCommandOpCode = 1;
constexpr byte kDisableMotorOpCode = 9;
constexpr byte kEnableMotorOpCode = 10;
constexpr uint8_t kMotorCommandLength = 3;

class Motor {
    //Internal variables for motor to store data.
    uint8_t directionPin;
    uint8_t pwmPin;
    uint8_t enablePin;
    unsigned long startTime = 0;
    uint32_t durationMs = 0;
    bool running = false;

    void setTimer(const uint8_t timerLength) {
        startTime = millis();
        durationMs = timerLength;
        running = true;
    }

    boolean updateTimer() const {
        return running && (millis() - startTime >= durationMs);
    }

    void setPWM(const uint8_t velocityDegPerSec) const {
        float targetRPM = static_cast<float>(velocityDegPerSec) / 6.0f;
        targetRPM = constrain(targetRPM, 0.0f, kMaxMotorRPM);
        const auto duty = static_cast<uint16_t>((targetRPM / kMaxMotorRPM) * kMaxPwm);
        analogWrite(this->pwmPin, duty);
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
        }
    }

public:
    explicit Motor(uint8_t directionPin, uint8_t pwmPin, uint8_t enablePin) {
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
        if (updateTimer()) {
            pwmStop(true);
            running = false;
        }
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
Motor motor(kDirectionPin, kPwmPin, kEnablePin);
byte opCode = 0; //Code to specify instruction type
uint8_t motorCommandBuffer[kMotorCommandLength] = {};
uint8_t motorCommandBytesRead = 0;
bool readingMotorCommand = false;

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

void runMotorCommand() {
    uint8_t speed  = motorCommandBuffer[0];
    uint8_t time = motorCommandBuffer[1];
    uint8_t direction = motorCommandBuffer[2];
    motor.update(speed, time, direction);

    SerialUSB.println(speed); SerialUSB.println(time); SerialUSB.println(direction);
}

void readMotorCommandByte() {
    if (!Serial1COM.available()) {
        return;
    }

    motorCommandBuffer[motorCommandBytesRead] = Serial1COM.readByte();
    motorCommandBytesRead++;

    if (motorCommandBytesRead == kMotorCommandLength) {
        runMotorCommand();
        motorCommandBytesRead = 0;
        readingMotorCommand = false;
    }
}


void setup() {
    Serial1.begin(1312500); //specific baud rate for Bpod
    analogWriteResolution(16);

    //debug
    pinMode(kStatusLedPin, OUTPUT);
    digitalWrite(kStatusLedPin, LOW);
    SerialUSB.begin(115200);
}

/**
 * Main loop
 * @author Emad Muzaffar
*/
void loop() {

    if (readingMotorCommand) {
        readMotorCommandByte();
    } else if (Serial1COM.available()) {
        opCode = Serial1COM.readByte();
        if (opCode == kModuleInfoOpCode) {
            returnModuleInfo();
        } else if (opCode == kDisableMotorOpCode) {
            //Turn enable off
            digitalWrite(kStatusLedPin, LOW);
            motor.disable();
        } else if (opCode == kEnableMotorOpCode) {
            //Turn enable on
            digitalWrite(kStatusLedPin, HIGH);
            motor.enable();
        } else if (opCode == kMotorCommandOpCode) {
            readingMotorCommand = true;
            motorCommandBytesRead = 0;
            readMotorCommandByte();
        }
    }
    motor.update();
}
