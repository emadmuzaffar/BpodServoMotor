//
// Created by Emad Muzaffar on 6/18/26.
//

#include <Arduino.h>

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#include <algorithm>
#include <vector>
#include <ArCom/ArCOM.h>

constexpr uint8_t kDirectionPin = 23;
constexpr uint8_t kPwmPin = 6;
constexpr uint8_t kEnablePin = 24;
constexpr uint8_t kHostTransmitPin = 52;
constexpr uint8_t kHostReceivePin = 50;
constexpr uint8_t kClientTransmitPin = 48;
constexpr uint8_t kClientReceivePin = 46;
float kMaxMotorRPM = 30.0f; // Must match the ClearPath MSP setup. //TODO: MAKE CONFIGURABLE
constexpr uint16_t kMaxPwm = 65535;
constexpr byte kModuleScanOpCode = 255;
constexpr byte kDisableMotorOpCode = 249;
constexpr byte kEnableMotorOpCode = 250;
constexpr byte kMotorHomeOpCode = 200;
constexpr byte kEncoderResetOpCode = 253;
constexpr byte kConfigOpCode = 254;
constexpr uint8_t kBytesPerUint16 = 2;
constexpr uint8_t kMotorInstructionFieldCount = 3;
constexpr uint8_t kMotorInstructionLength = kMotorInstructionFieldCount * kBytesPerUint16;
constexpr uint8_t kMotorHomeInstructionLength = kBytesPerUint16;
constexpr uint8_t kConfigFieldCount = 6;
constexpr uint8_t kConfigLength = kConfigFieldCount * kBytesPerUint16;
constexpr uint32_t kHostStartTolerance = 300;
constexpr uint32_t kCrossCheckMaxNoResponseTime = 5000;
constexpr uint32_t kCrossCheckDutyTime = 500;
int timeMultiplier = 100; //TODO: MAKE CONFIGURABLE


struct Instruction {
    uint16_t speed;
    uint16_t time;
    uint16_t direction;
    Instruction() : speed(0), time(0), direction(0) {}
    Instruction(const uint16_t speed, const uint16_t time, const uint16_t direction)
        : speed(speed), time(time), direction(direction) {}
};

struct Config {
    float maxMotorRPM;
    int timeMultiplier;
    int tolerance;
    int encoderPPR;
    float kP;
    float kD;
    Config() : maxMotorRPM(30.0f), timeMultiplier(1), tolerance(5000), encoderPPR(14400), kP(0.1f), kD(0.1f) {}
    Config(const float maxMotorRPM, const int timeMultiplier, const int tolerance, const int encoderPPR, const float kP, const float kD)
        : maxMotorRPM(maxMotorRPM),
          timeMultiplier(timeMultiplier),
          tolerance(tolerance),
          encoderPPR(encoderPPR),
          kP(kP),
          kD(kD) {}
};

class Safetynet {
    friend class Motor;

    int32_t tolerance = 5000; //TODO: MAKE CONFIGURABLE
    int32_t encoderPPR = 14400; //TODO: MAKE CONFIGURABLE and multiply by 4 on input
    const uint8_t enablePin;
    unsigned long startTime = 0;
    double tPosition = 0;
    Instruction cInstruction;
    int32_t tickOffset = 0;
    bool eStopped = false;

    static float calculateInstructionDistance(const Instruction &instruction) {
        float distance = static_cast<float>(instruction.speed) * (static_cast<float>(instruction.time) * static_cast<float>(timeMultiplier));
        if (instruction.direction == 1) {
            distance = -distance;
        }
        return distance;
    }

    static float calculateInstructionDistance(const Instruction &instruction, const unsigned long elapsedTime) {
        float distance = static_cast<float>(instruction.speed) * (static_cast<float>(elapsedTime));
        if (instruction.direction == 1) {
            distance = -distance;
        }
        return distance;
    }

    bool checkTolerance() const {
        const double ctPosition = tPosition + -calculateInstructionDistance(cInstruction) + calculateInstructionDistance(cInstruction, micros() - startTime);
        const int32_t error = abs(getTicks() - static_cast<int32_t>(ctPosition));
        return error >= tolerance;
    }

    bool checkPositionSafety() const {
        return abs(getTicks()) > 29000;
    }

    void applyConfig(const Config &config) {
        tolerance = config.tolerance;
        encoderPPR = config.encoderPPR;
    }

public:
    explicit Safetynet(const uint8_t enablePin) : enablePin(enablePin) {}

    static void setupEncoder() {
        pmc_enable_periph_clk(ID_TC0);

        // D2 = TIOA0, D13 = TIOB0
        PIOB->PIO_PDR = PIO_PB25;   // D2
        PIOB->PIO_ABSR &= ~PIO_PB25;

        PIOB->PIO_PDR = PIO_PB27;   // D13
        PIOB->PIO_ABSR &= ~PIO_PB27;

        // Enable quadrature decoder
        TC0->TC_BMR =
            TC_BMR_QDEN |
            TC_BMR_POSEN |
            TC_BMR_EDGPHA;

        TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_XC0;

        TC0->TC_CHANNEL[0].TC_CCR =
            TC_CCR_CLKEN |
            TC_CCR_SWTRG;
    }

    int32_t getTicks() const {
        return static_cast<int32_t>(TC0->TC_CHANNEL[0].TC_CV) - tickOffset;
    }

    static int32_t getRawTicks() {
        return static_cast<int32_t>(TC0->TC_CHANNEL[0].TC_CV);
    }

    void setTickOffset() {
        tickOffset = getRawTicks();
    }


    void reportOverride() {
        tPosition += -calculateInstructionDistance(cInstruction);
        tPosition += calculateInstructionDistance(cInstruction, micros() - startTime);
    }

    void reportInstruction(const Instruction &instruction) {
        startTime = micros();
        cInstruction = instruction;
        tPosition += calculateInstructionDistance(instruction);
    }

    double getTargetTicks() const {
        return tPosition / 360 * encoderPPR;
    }

    void enable() const {
        if (eStopped) {
            disable();
            return;
        }

        digitalWrite(this->enablePin, HIGH);
    }

    void disable() const {
        digitalWrite(this->enablePin, LOW);
    }

    void eStop() {
        eStopped = true;
        disable();
    }


    void update() {
        SerialUSB.println(getTicks()); //TODO: debug
        if (checkPositionSafety()) {
            disable();
            eStopped = true;
        }
        if (checkTolerance()) {
            disable();
            eStopped = true;
        }

    }
};


class Motor {
    enum motorState{
        pCORRECTING,
        CORRECTING,
        pINSTRUCTED,
        INSTRUCTED
    };

    //Internal variables for motor to store data.
    motorState motorState = pCORRECTING;
    const uint8_t directionPin;
    const uint8_t pwmPin;
    unsigned long startTime = 0;
    uint32_t durationMs = 0;
    size_t cInstruction = 0;
    std::vector<Instruction> instructions;
    bool running = false;
    double pidTarget = 0;
    double lastError = 0;
    double pidTime = 0;
    float kP = 0.1; //TODO: MAKE CONFIGURABLE
    float kD = 0.1; //TODO: MAKE CONFIGURABLE

    void setTimer(const uint16_t timerLength) {
        startTime = millis();
        durationMs = timerLength * timeMultiplier;
    }

    bool timerComplete() {
        if (millis() - startTime >= durationMs) {
            running = false;
            return true;
        }
        running = true;
        return false;
    }

    static float getVelocity(const uint16_t velocityDegPerSec) {
        const float targetRPM = static_cast<float>(velocityDegPerSec) / 6.0f;
        return constrain(targetRPM, 0.0f, kMaxMotorRPM);
    }

    void setPWM(const uint16_t velocityDegPerSec) const {
        const float targetRPM = getVelocity(velocityDegPerSec);
        const auto duty = static_cast<uint16_t>((targetRPM / kMaxMotorRPM) * kMaxPwm);
        analogWrite(this->pwmPin, duty);
    }

    void setDirection(const uint16_t direction) const {
        if (direction == 0) {
            digitalWrite(directionPin, HIGH);
        } else if (direction == 1) {
            digitalWrite(directionPin, LOW);
        }
    }

    void pid() {
        const double error = safetynet.getTicks() - pidTarget;
        if (lastError == 0) {
            lastError = error;
        }

        const double dt = micros() - pidTime;

        const double p = error;

        const double d = (error - lastError) / dt;

        const double targetRPM = p * kP + d * kD;
        const auto duty = static_cast<uint16_t>((targetRPM / kMaxMotorRPM) * kMaxPwm);
        analogWrite(this->pwmPin, duty);
        pidTime = micros();
        lastError = error;
    }

    void correctionPID() {
        pidTarget = safetynet.getTargetTicks();
        pid();
    }


    void pwmStop() {
        analogWrite(this->pwmPin, 0);
        pidTarget = 0;
        lastError = 0;
        pidTime = 0;
    }

    void internalRunInstruction(const Instruction &instruction) {
        setDirection(instruction.direction);
        setPWM(instruction.speed);
        setTimer(instruction.time);
        motorState = pINSTRUCTED;
        safetynet.reportInstruction(instruction);
    }

public:
    Safetynet safetynet;
    explicit Motor(const uint8_t directionPin, const uint8_t pwmPin, const uint8_t enablePin)
        : directionPin(directionPin), pwmPin(pwmPin), safetynet(enablePin) {
        Safetynet::setupEncoder();
        pinMode(directionPin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
        pinMode(enablePin, OUTPUT);
    }

    bool checkStatusAndUpdate() {
        safetynet.update();
        switch (motorState) {
            case pCORRECTING:
                correctionPID();
                motorState = CORRECTING;
                return false;

            case CORRECTING:
                pid();
                return false;

            case pINSTRUCTED:
                if (instructions.empty()) {
                    motorState = CORRECTING;
                } else {
                    motorState = INSTRUCTED;
                }
                return false;

            case INSTRUCTED:
                if (timerComplete()) {
                    cInstruction++;
                    if (cInstruction < instructions.size()) {
                        internalRunInstruction(instructions[cInstruction]);
                    } else {
                        instructions.clear();
                        motorState = pCORRECTING;
                        return true;
                    }
                }
                return false;

            default:
                // Safety fallback
                motorState = CORRECTING;
                return false;
        }
    }

    void setInstructions(const Instruction * const nInstructions, const size_t instructionCount) {
        cInstruction = 0;
        instructions.clear();

        if (instructionCount > 0) {
            instructions.assign(nInstructions, nInstructions + instructionCount);
        }

        if (instructions.empty()) {
            pwmStop();
        } else {
            if (running) {
                safetynet.reportOverride();
            }
            internalRunInstruction(instructions[cInstruction]);
        }
    }

    void home(const uint16_t speed) {
        const int32_t cPos = safetynet.getTicks();
        const double cPosDegrees = static_cast<double>(cPos) / 40;
        const double time = cPosDegrees / speed;
        Instruction instruction[1];
        instruction[0].speed = speed;
        instruction[0].time = static_cast<uint16_t>(time);
        if (cPos > 0) {
            instruction[0].direction = 1;
        } else if (cPos < 0) {
            instruction[0].direction = 0;
        }
        setInstructions(instruction, 1);
    }

    void applyConfig(const Config &config) {
        kP = config.kP;
        kD = config.kD;
        safetynet.applyConfig(config);
    }

};


/**
 * Initialize objects and module setup
 * @author Emad Muzaffar
 */
constexpr uint32_t FirmwareVersion = 1;
constexpr char moduleName[] = "ServoMotor"; // ModuleName sent to Bpod in returnModuleInfo()
ArCOM Serial1COM(Serial1); // NOLINT(*-interfaces-global-init)
Motor motor(kDirectionPin, kPwmPin, kEnablePin);
byte opCode = 0; // Control opcode, or number of motor instructions to read.
uint8_t motorInstructionByteBuffer[kMotorInstructionLength] = {};
uint8_t motorInstructionBytesRead = 0;
uint8_t motorInstructionsExpected = 0;
std::vector<Instruction> receivedInstructions;
uint8_t configByteBuffer[kConfigLength] = {};
uint8_t configBytesRead = 0;
Config receivedConfig;
uint32_t readingStartTime = 0;
enum ReadState {
    OFF,
    ON,
    MACRO,
    CONFIG
};
ReadState readState = OFF;
uint8_t macroInstructionLength = 0;
uint32_t hostTransmitTime = 0;

/**
 * From SanWorks example
 * Writes info about Arduino system over serial to Bpod
 * Called in loop in switch(opCode) when opCode == 255, what Bpod sends to scan for modules
 * @authors JSNeuroDev, Emad Muzaffar
 */
void returnModuleInfo() {
    Serial1COM.writeByte(65); // Acknowledge
    Serial1COM.writeUint32(FirmwareVersion); // 4-byte firmware version
    Serial1COM.writeByte(sizeof(moduleName) - 1); // Length of module name
    Serial1COM.writeCharArray(moduleName, sizeof(moduleName) - 1); // Module name
    Serial1COM.writeByte(1);                      // More info follows
    Serial1COM.writeByte('#');                    // Behavior event record
    Serial1COM.writeByte(2);                      // Number of events
    constexpr char eventName1[] = "Done";
    Serial1COM.writeByte(sizeof(eventName1) - 1);    // Length
    Serial1COM.writeCharArray(eventName1, sizeof(eventName1) - 1);
    constexpr char eventName2[] = "Received";
    Serial1COM.writeByte(sizeof(eventName2) - 1);    // Length
    Serial1COM.writeCharArray(eventName2, sizeof(eventName2) - 1);
    Serial1COM.writeByte(0);
}

void applyConfig(const Config &config) {
    kMaxMotorRPM = config.maxMotorRPM;
    timeMultiplier = config.timeMultiplier;
    motor.applyConfig(config);
}

void startReadingMotorInstructions(const uint8_t instructionCount) {
    motorInstructionBytesRead = 0;
    motorInstructionsExpected = instructionCount;
    receivedInstructions.clear();
    receivedInstructions.reserve(motorInstructionsExpected);

    if (motorInstructionsExpected == 0) {
        motor.setInstructions(receivedInstructions.data(), receivedInstructions.size());
        readState = OFF;
    } else {
        readState = ON;
    }
}

void startReadingMacroInstructions(const uint8_t macroCode) {
    if (macroCode == kMotorHomeOpCode) {
        readState = MACRO;
        motorInstructionBytesRead = 0;
        macroInstructionLength = kMotorHomeInstructionLength;
        motorInstructionsExpected = 1;
        receivedInstructions.clear();
        receivedInstructions.reserve(1);
    }
}

void startReadingConfig() {
    readState = CONFIG;
    configBytesRead = 0;
    receivedConfig = Config();
}

uint16_t uint16ValueAt(const uint8_t * const byteBuffer, const uint8_t valueIndex) {
    const uint8_t byteIndex = valueIndex * kBytesPerUint16;
    return static_cast<uint16_t>(byteBuffer[byteIndex]) |
        (static_cast<uint16_t>(byteBuffer[byteIndex + 1]) << 8);
}

void finishMotorInstructions() {
    if (readState == ON) {
        motor.setInstructions(receivedInstructions.data(), receivedInstructions.size());
    } else if (readState == MACRO) {
        motor.home(receivedInstructions.front().speed);
    }
    //reset system
    std::fill_n(motorInstructionByteBuffer, kMotorInstructionLength, 0);
    motorInstructionBytesRead = 0;
    motorInstructionsExpected = 0;
    macroInstructionLength = 0;
    Serial1COM.writeByte(2);
    readState = OFF;
}

void finishConfig() {
    applyConfig(receivedConfig);
    //reset system
    receivedConfig = Config();
    std::fill_n(configByteBuffer, kConfigLength, 0);
    configBytesRead = 0;
    Serial1COM.writeByte(2);
    readState = OFF;
}

void readConfigByte() {
    if (!Serial1COM.available()) {
        return;
    }
    configByteBuffer[configBytesRead] = Serial1COM.readByte();
    configBytesRead++;

    if (configBytesRead == kConfigLength) {
        const uint16_t maxMotorRPM = uint16ValueAt(configByteBuffer, 0);
        const uint16_t configuredTimeMultiplier = uint16ValueAt(configByteBuffer, 1);
        const uint16_t tolerance = uint16ValueAt(configByteBuffer, 2);
        const uint16_t encoderPPR = uint16ValueAt(configByteBuffer, 3);
        const uint16_t kP = uint16ValueAt(configByteBuffer, 4);
        const uint16_t kD = uint16ValueAt(configByteBuffer, 5);
        receivedConfig = Config(maxMotorRPM,
            configuredTimeMultiplier,
            tolerance,
            encoderPPR,
            kP,
            kD);
        configBytesRead = 0;

        //TODO: debug
        SerialUSB.println(maxMotorRPM);
        SerialUSB.println(configuredTimeMultiplier);
        SerialUSB.println(tolerance);
        SerialUSB.println(encoderPPR);
        SerialUSB.println(kP);
        SerialUSB.println(kD);

        finishConfig();
    }
}



void readMotorInstructionByte(const uint8_t motorInstructionLength = kMotorInstructionLength) {
    if (!Serial1COM.available()) {
        return;
    }
    motorInstructionByteBuffer[motorInstructionBytesRead] = Serial1COM.readByte();
    motorInstructionBytesRead++;

    if (motorInstructionBytesRead == motorInstructionLength) {
        const uint16_t speed = uint16ValueAt(motorInstructionByteBuffer, 0);
        uint16_t time = 0;
        uint16_t direction = 0;
        if (motorInstructionLength >= 2 * kBytesPerUint16) {
            time = uint16ValueAt(motorInstructionByteBuffer, 1);
        }
        if (motorInstructionLength >= 3 * kBytesPerUint16) {
            direction = uint16ValueAt(motorInstructionByteBuffer, 2);
        }
        receivedInstructions.emplace_back(speed, time, direction);
        motorInstructionBytesRead = 0;
        Serial1COM.writeByte(2);
        //TODO: debug
        SerialUSB.println(speed); SerialUSB.println(time); SerialUSB.println(direction);

        if (receivedInstructions.size() == motorInstructionsExpected) {
            finishMotorInstructions();
        }
    }
}

void hostTransmit() {
    const uint32_t nowMicros = micros();

    if (nowMicros - hostTransmitTime > kCrossCheckDutyTime) {
        const uint8_t nextState = digitalRead(kHostTransmitPin) == HIGH ? LOW : HIGH;
        digitalWrite(kHostTransmitPin, nextState);
        hostTransmitTime = nowMicros;
    }
}

void hostUpdate() {
    if (millis() > kHostStartTolerance) {
        if (digitalRead(kHostReceivePin) != digitalRead(kHostTransmitPin)) {
            if (micros() - hostTransmitTime > kCrossCheckMaxNoResponseTime) {
                motor.safetynet.eStop();
            }
        } else {
            hostTransmit();
        }
    }
}

void clientUpdate() {
    digitalWrite(kClientTransmitPin, digitalRead(kClientReceivePin));
}

void crossCheckUpdate() {
    hostUpdate();
    clientUpdate();
}



/**
 * Setup
 * @author Emad Muzaffar
*/
void setup() {
    Serial1.begin(1312500); //specific baud rate for Bpod
    analogWriteResolution(16); //higher than default resolution for finer control,
    analogWrite(kPwmPin, 0); // ensure pwm is off
    pinMode(kHostTransmitPin, OUTPUT);
    pinMode(kHostReceivePin, INPUT);
    pinMode(kClientTransmitPin, OUTPUT);
    pinMode(kClientReceivePin, INPUT);
    //TODO: debug
    SerialUSB.begin(115200);
}

/**
 * Main loop
 * @author Emad Muzaffar
*/
//TODO: debug
uint32_t oldMicros = 0;

void loop() {

    //TODO: debug
    SerialUSB.println(micros() - oldMicros);
    oldMicros = micros();


    if (readState == ON) {
        readMotorInstructionByte();
    } else if (readState == MACRO) {
        readMotorInstructionByte(macroInstructionLength);
    } else if (readState == CONFIG) {
        readConfigByte();
    } else if (Serial1COM.available()) {
        opCode = Serial1COM.readByte();
        if (opCode == kModuleScanOpCode) {
            returnModuleInfo();
            motor.safetynet.setTickOffset();
        } else if (opCode == kDisableMotorOpCode) {
            //Turn enable off
            motor.safetynet.disable();
        } else if (opCode == kEnableMotorOpCode) {
            //Turn enable on
            motor.safetynet.enable();
        } else if (opCode == kMotorHomeOpCode) {
            startReadingMacroInstructions(opCode);
        } else if (opCode == kEncoderResetOpCode) {
            motor.safetynet.setTickOffset();
        } else if (opCode == kConfigOpCode) {
            startReadingConfig();
        } else {
            //Reads motor instructions
            startReadingMotorInstructions(opCode);
            readMotorInstructionByte();
        }
    }
    if (motor.checkStatusAndUpdate()) Serial1COM.writeByte(1); //updates motor, writes behavior state byte to Bpod if completed
    crossCheckUpdate();
}
