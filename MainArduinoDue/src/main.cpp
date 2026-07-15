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
#include <cmath>
#include <vector>
#include <ArCom/ArCOM.h>

///Declare Constants
constexpr uint8_t kDirectionPin = 7;
constexpr uint8_t kPwmPin = 6;
constexpr uint8_t kEnablePin = 53;
constexpr uint8_t kHostTransmitPin = 52;
constexpr uint8_t kHostReceivePin = 50;
constexpr uint8_t kClientTransmitPin = 48;
constexpr uint8_t kClientReceivePin = 46;
constexpr float kMaxMotorRPM = 120; // Must match the ClearPath MSP setup.
float MaxMotorRPM = kMaxMotorRPM;
constexpr uint16_t kMaxPwm = 65535;
constexpr double kPwmCalibration = 1.004; // Compensates for the measured 0.2-0.5% low PWM output.
constexpr byte kModuleScanOpCode = 255;
constexpr byte kDisableMotorOpCode = 249;
constexpr byte kEnableMotorOpCode = 250;
constexpr byte kMotorInstructionOpCode = 1;
constexpr byte kMotorHomeOpCode = 200;
constexpr byte kEncoderResetOpCode = 253;
constexpr byte kConfigOpCode = 254;
constexpr uint8_t kBytesPerUint16 = 2;
constexpr uint8_t kMotorInstructionLength = 2 * kBytesPerUint16 + sizeof(uint8_t);
constexpr uint8_t kMotorHomeInstructionLength = kBytesPerUint16;
constexpr uint8_t kConfigFieldCount = 8;
constexpr uint8_t kConfigLength = kConfigFieldCount * kBytesPerUint16;
constexpr uint32_t kHostStartTolerance = 300;
constexpr uint32_t kCrossCheckMaxNoResponseTime = 5000;
constexpr uint32_t kCrossCheckDutyTime = 500;
constexpr double kDegreesPerRotation = 360.0;
constexpr double kMillisPerSecond = 1000.0;
constexpr double kMicrosPerSecond = 1000000.0;
constexpr double kMaxInstructionTime = 65535.0;
constexpr float kConfigGainScale = 1000.0f;
constexpr int kTimeMultiplier = 1;
int timeMultiplier = kTimeMultiplier;
constexpr int kTolerance = 300;
constexpr int kEncoderPPR = 14400;
constexpr float defaultKP = 0.1f;
constexpr float defaultKD = 0.1f;
constexpr bool kCorrectionEnabled = true;
constexpr bool kUsbDebugEnabled = true; // Central switch for all USB debugging.
constexpr uint32_t kUsbDebugStartupDelayMs = 500;
constexpr int32_t kMaxRotation = kEncoderPPR * 2;

constexpr int32_t rotationLimitTicks(const uint16_t maxRotationDegrees, const uint16_t encoderPPR) {
    // The quadrature decoder counts four edges per encoder pulse. Add half a
    // degree-of-revolution denominator so integer division rounds to nearest.
    return static_cast<int32_t>(
        (static_cast<int64_t>(maxRotationDegrees) * encoderPPR * 4 + 180) / 360);
}

/**
 * USB debugging. Messages start with [FLOW][caller] so they are easy to
 * identify and filter in the serial monitor.
 */
void usbDebug(const __FlashStringHelper *message) {
    if (kUsbDebugEnabled) {
        Serial.println(message);
    }
}

void usbDebugValue(const __FlashStringHelper *message, const uint32_t value) {
    if (kUsbDebugEnabled) {
        Serial.print(message);
        Serial.println(value);
    }
}

/**
* Struct to hold the information required for a motor instruction
* Used to pass data into the motor class
* @author Emad Muzaffar
*/
struct Instruction {
    uint16_t speed;
    uint16_t time;
    uint8_t direction;
    Instruction() : speed(0), time(0), direction(0) {}
    Instruction(const uint16_t speed, const uint16_t time, const uint8_t direction)
        : speed(speed), time(time), direction(direction) {}
};

/**
*
* @author Emad Muzaffar
*/
struct Config {
    float maxMotorRPM;
    int timeMultiplier;
    int tolerance;
    int encoderPPR;
    int32_t maxRotation;
    float kP;
    float kD;
    bool correctionEnabled;
    Config() : maxMotorRPM(kMaxMotorRPM), timeMultiplier(kTimeMultiplier), tolerance(kTolerance), encoderPPR(kEncoderPPR), maxRotation(kMaxRotation), kP(defaultKP), kD(defaultKD), correctionEnabled(kCorrectionEnabled) {}
    Config(const float maxMotorRPM, const int timeMultiplier, const int tolerance, const int encoderPPR, const int maxRotation, const float kP, const float kD, const bool correctionEnabled)
        : maxMotorRPM(maxMotorRPM),
            timeMultiplier(timeMultiplier),
            tolerance(tolerance),
            encoderPPR(static_cast<int32_t>(encoderPPR) * 4),
            maxRotation(rotationLimitTicks(maxRotation, encoderPPR)),
            kP(kP),
            kD(kD),
            correctionEnabled(correctionEnabled) {}
};

/**
*
* @author Emad Muzaffar
*/
class Safetynet {
    friend class Motor; //Allow motor to access private

    //Internal variables for safetynet to store data.
    int32_t tolerance = kTolerance;
    int32_t encoderPPR = kEncoderPPR;
    int32_t maxRotation = kMaxRotation;
    const uint8_t enablePin;
    unsigned long startTime = 0;
    double tPosition = 0;
    Instruction cInstruction;
    int32_t tickOffset = 0;
    bool eStopped = false;

    /**
    *
    * @author Emad Muzaffar
    */
    static double absoluteValue(const double value) {
        return value < 0.0 ? -value : value;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    static double directionSign(const Instruction &instruction) {
        return instruction.direction == 1 ? -1.0 : 1.0;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    static double instructionDurationMs(const Instruction &instruction) {
        return static_cast<double>(instruction.time) * static_cast<double>(timeMultiplier);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    static double calculateInstructionDistance(const Instruction &instruction) {
        const double durationSeconds = instructionDurationMs(instruction) / kMillisPerSecond;
        return directionSign(instruction) * static_cast<double>(instruction.speed) * durationSeconds;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    static double calculateInstructionDistance(const Instruction &instruction, const unsigned long elapsedMicros) {
        const double maxElapsedMicros = instructionDurationMs(instruction) * kMillisPerSecond;
        const double clampedElapsedMicros = std::min(static_cast<double>(elapsedMicros), maxElapsedMicros);
        const double elapsedSeconds = clampedElapsedMicros / kMicrosPerSecond;
        return directionSign(instruction) * static_cast<double>(instruction.speed) * elapsedSeconds;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    double degreesToTicks(const double degrees) const {
        return degrees / kDegreesPerRotation * static_cast<double>(encoderPPR);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    double ticksToDegrees(const int32_t ticks) const {
        if (encoderPPR == 0) {
            return 0.0;
        }
        return static_cast<double>(ticks) * kDegreesPerRotation / static_cast<double>(encoderPPR);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    double currentTargetDegrees() const {
        return tPosition - calculateInstructionDistance(cInstruction) +
            calculateInstructionDistance(cInstruction, micros() - startTime);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    bool checkTolerance() const {
        const double targetTicks = degreesToTicks(currentTargetDegrees());
        const double error = absoluteValue(static_cast<double>(getTicks()) - targetTicks);
        return error >= tolerance;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    bool checkPositionSafety() const {
        return abs(getTicks()) > maxRotation;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void applyConfig(const Config &config) {
        tolerance = config.tolerance;
        encoderPPR = config.encoderPPR;
        maxRotation = config.maxRotation;
    }

public:
    /**
    *
    * @author Emad Muzaffar
    */
    explicit Safetynet(const uint8_t enablePin) : enablePin(enablePin) {}

    void begin() const {
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW);
        usbDebugValue(F("[FLOW][Safetynet::begin] enable pin readback="), digitalRead(enablePin));
    }

    /**
    *
    * @author Emad Muzaffar
    */
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

    /**
    *
    * @author Emad Muzaffar
    */
    int32_t getTicks() const {
        return -static_cast<int32_t>(TC0->TC_CHANNEL[0].TC_CV) - tickOffset;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    static int32_t getRawTicks() {
        return -static_cast<int32_t>(TC0->TC_CHANNEL[0].TC_CV);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void setTickOffset() {
        tickOffset = getRawTicks();
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void reportOverride() {
        tPosition += -calculateInstructionDistance(cInstruction);
        tPosition += calculateInstructionDistance(cInstruction, micros() - startTime);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void reportInstruction(const Instruction &instruction) {
        startTime = micros();
        cInstruction = instruction;
        tPosition += calculateInstructionDistance(instruction);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    double getTargetTicks() const {
        return degreesToTicks(tPosition);
    }

    /**
    * Enable the M
    * @author Emad Muzaffar
    */
    void enable() const {
        if (eStopped) {
            usbDebug(F("[FLOW][Safetynet::enable] blocked: emergency stop is active"));
            // eStop();
            return;
        }
        digitalWrite(this->enablePin, HIGH);
        usbDebugValue(F("[FLOW][Safetynet::enable] enable pin readback="), digitalRead(this->enablePin));
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void disable() const {
        digitalWrite(this->enablePin, LOW);
        usbDebugValue(F("[FLOW][Safetynet::disable] enable pin readback="), digitalRead(this->enablePin));
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void eStop() {
        eStopped = true;
        disable();
        usbDebug(F("[FLOW][Safetynet::eStop] emergency-stop path entered"));
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void update() {

        SerialUSB.println(getTicks());
        SerialUSB.println(getTargetTicks());

        if (eStopped == true) {
            return;
        }
        if (checkPositionSafety()) {
            usbDebug(F("[FLOW][Safetynet::update] eStop suppressed: position safety limit exceeded"));
            eStop();
        }
        if (checkTolerance()) {
            usbDebug(F("[FLOW][Safetynet::update] eStop suppressed: tracking tolerance exceeded"));
            // eStop();
        }

    }
};


class Motor {

    //Enum for motor update FSM
    enum motorState{
        INACTIVE,
        pCORRECTING,
        CORRECTING,
        pINSTRUCTED,
        INSTRUCTED
    };

    //Internal variables for motor to store data.
    motorState motorState = INACTIVE;
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
    float kP = defaultKP;
    float kD = defaultKD;
    bool correctionEnabled = kCorrectionEnabled;

    /**
    *
    * @author Emad Muzaffar
    */
    void setTimer(const uint16_t timerLength) {
        startTime = millis();
        durationMs = static_cast<uint32_t>(timerLength) * static_cast<uint32_t>(timeMultiplier);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    bool timerComplete() {
        if (millis() - startTime >= durationMs) {
            running = false;
            return true;
        }
        running = true;
        return false;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    static float getVelocity(const uint16_t velocityDegPerSec) {
        if (MaxMotorRPM <= 0.0f) {
            return 0.0f;
        }
        const float targetRPM = static_cast<float>(velocityDegPerSec) / 6.0f;
        return std::min(std::max(targetRPM, 0.0f), MaxMotorRPM);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void writePwmForRPM(const double rpm) const {
        if (MaxMotorRPM <= 0.0f) {
            analogWrite(this->pwmPin, 0);
            return;
        }

        const double clampedRPM = std::min(std::max(rpm, 0.0), static_cast<double>(MaxMotorRPM));

        const auto duty = static_cast<uint16_t>(std::lround(clampedRPM / static_cast<double>(MaxMotorRPM) * kMaxPwm));

        analogWrite(this->pwmPin, duty);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void setPWM(const uint16_t velocityDegPerSec) const {
        writePwmForRPM(getVelocity(velocityDegPerSec));
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void setDirection(const uint8_t direction) const {
        if (direction == 0) {
            digitalWrite(directionPin, HIGH);
        } else if (direction == 1) {
            digitalWrite(directionPin, LOW);
        }
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void pid() {
        const auto nowMicros = static_cast<double>(micros());
        const double error = static_cast<double>(safetynet.getTicks()) - pidTarget;
        const double dt = nowMicros - pidTime;
        const double p = error;
        const double d = pidTime > 0.0 && dt > 0.0 ? (error - lastError) / dt : 0.0;
        const double targetRPM = p * kP + d * kD;

        if (targetRPM > 0.0) {
            setDirection(1);
        } else if (targetRPM < 0.0) {
            setDirection(0);
        }
        writePwmForRPM(targetRPM < 0.0 ? -targetRPM : targetRPM);
        pidTime = nowMicros;
        lastError = error;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void correctionPID() {
        pidTarget = safetynet.getTargetTicks();
        pid();
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void pwmStop() {
        analogWrite(this->pwmPin, 0);
        running = false;
        pidTarget = 0;
        lastError = 0;
        pidTime = 0;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void internalRunInstruction(const Instruction &instruction) {
        usbDebugValue(F("[FLOW][Motor::internalRunInstruction] speed="), instruction.speed);
        usbDebugValue(F("[FLOW][Motor::internalRunInstruction] time="), instruction.time);
        usbDebugValue(F("[FLOW][Motor::internalRunInstruction] direction="), instruction.direction);
        setDirection(instruction.direction);
        setPWM(instruction.speed);
        setTimer(instruction.time);
        motorState = pINSTRUCTED;
        usbDebug(F("[FLOW][Motor::internalRunInstruction] state -> pINSTRUCTED"));
        safetynet.reportInstruction(instruction);
    }

public:

    /**
    *
    * @author Emad Muzaffar
    */
    Safetynet safetynet;
    explicit Motor(const uint8_t directionPin, const uint8_t pwmPin, const uint8_t enablePin)
        : directionPin(directionPin), pwmPin(pwmPin), safetynet(enablePin) {}

    void begin() const {
        Safetynet::setupEncoder();
        pinMode(directionPin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
        safetynet.begin();
        usbDebug(F("[FLOW][Motor::begin] motor hardware initialized"));
    }

    /**
    *
    * @author Emad Muzaffar
    */
    bool checkStatusAndUpdate() {
        safetynet.update();
        switch (motorState) {
            case INACTIVE:
                return false;

            case pCORRECTING:
                if (!correctionEnabled) {
                    motorState = INACTIVE;
                    usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] correction disabled; state -> INACTIVE"));
                    return false;
                }
                correctionPID();
                motorState = CORRECTING;
                usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] state -> CORRECTING"));
                return false;

            case CORRECTING:
                pid();
                return false;

            case pINSTRUCTED:
                if (instructions.empty()) {
                    motorState = CORRECTING;
                    usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] no queued instructions; state -> CORRECTING"));
                } else {
                    motorState = INSTRUCTED;
                    usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] state -> INSTRUCTED"));
                }
                return false;

            case INSTRUCTED:
                if (timerComplete()) {
                    cInstruction++;
                    if (cInstruction < instructions.size()) {
                        usbDebugValue(F("[FLOW][Motor::checkStatusAndUpdate] starting instruction index="), cInstruction);
                        internalRunInstruction(instructions[cInstruction]);
                    } else {
                        instructions.clear();
                        motorState = pCORRECTING;
                        usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] sequence complete; state -> pCORRECTING"));
                        return true;
                    }
                }
                return false;

            default:
                // Safety fallback
                motorState = CORRECTING;
                usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] invalid state; fallback -> CORRECTING"));
                return false;
        }
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void setInstructions(const Instruction * const nInstructions, const size_t instructionCount) {
        usbDebugValue(F("[FLOW][Motor::setInstructions] loading instruction count="), instructionCount);
        cInstruction = 0;
        instructions.clear();

        if (instructionCount > 0) {
            instructions.assign(nInstructions, nInstructions + instructionCount);
        }

        if (instructions.empty()) {
            usbDebug(F("[FLOW][Motor::setInstructions] instruction list empty; stopping PWM"));
            pwmStop();
        } else {
            if (running) {
                usbDebug(F("[FLOW][Motor::setInstructions] active instruction overridden"));
                safetynet.reportOverride();
            }
            usbDebug(F("[FLOW][Motor::setInstructions] starting instruction index=0"));
            internalRunInstruction(instructions[cInstruction]);
        }
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void home(const uint16_t speed) {
        const int32_t cPos = safetynet.getTicks();
        usbDebugValue(F("[FLOW][Motor::home] requested speed="), speed);
        if (speed == 0 || cPos == 0 || timeMultiplier <= 0) {
            usbDebug(F("[FLOW][Motor::home] skipped: invalid speed, position, or time multiplier"));
            pwmStop();
            return;
        }

        const double cPosDegrees = safetynet.ticksToDegrees(cPos);
        const double timeSeconds = Safetynet::absoluteValue(cPosDegrees) / static_cast<double>(speed);
        const double timeUnits = timeSeconds * kMillisPerSecond / static_cast<double>(timeMultiplier);
        Instruction instruction[1];
        instruction[0].speed = speed;
        instruction[0].time = static_cast<uint16_t>(std::min(timeUnits + 0.999, kMaxInstructionTime));
        if (cPos >= 0) {
            instruction[0].direction = 1;
        } else {
            instruction[0].direction = 0;
        }
        setInstructions(instruction, 1);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void applyConfig(const Config &config) {
        usbDebug(F("[FLOW][Motor::applyConfig] applying motor and safety configuration"));
        kP = config.kP;
        kD = config.kD;
        correctionEnabled = config.correctionEnabled;
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
    COUNT,
    ON,
    MACRO,
    CONFIG
};
ReadState readState = OFF;
uint8_t macroInstructionLength = 0;
uint32_t hostTransmitTime = 0;
uint32_t lastHostEchoTime = 0;
uint8_t lastHostEchoState = LOW;
bool hostTimeoutReported = false;

/**
 * From SanWorks example
 * Writes info about Arduino system over serial to Bpod
 * Called in loop in switch(opCode) when opCode == 255, what Bpod sends to scan for modules
 * @authors JSNeuroDev, Emad Muzaffar
 */
void returnModuleInfo() {
    usbDebug(F("[FLOW][returnModuleInfo] sending module information"));
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
    usbDebug(F("[FLOW][returnModuleInfo] response complete"));
}

/**
 *
 * @author Emad Muzaffar
*/
void applyConfig(const Config &config) {
    MaxMotorRPM = config.maxMotorRPM;
    timeMultiplier = config.timeMultiplier;
    motor.applyConfig(config);
}

/**
 *
 * @author Emad Muzaffar
*/
void startReadingMotorInstructions(const uint8_t instructionCount) {
    usbDebugValue(F("[FLOW][startReadingMotorInstructions] read state -> ON; instruction count="), instructionCount);
    motorInstructionBytesRead = 0;
    motorInstructionsExpected = instructionCount;
    receivedInstructions.clear();
    receivedInstructions.reserve(motorInstructionsExpected);

    if (motorInstructionsExpected == 0) {
        usbDebug(F("[FLOW][startReadingMotorInstructions] zero instructions; read state -> OFF"));
        motor.setInstructions(receivedInstructions.data(), receivedInstructions.size());
        readState = OFF;
    } else {
        readState = ON;
    }
}

void readMotorInstructionCountByte() {
    if (!Serial1COM.available()) {
        return;
    }
    startReadingMotorInstructions(Serial1COM.readByte());
}

/**
 *
 * @author Emad Muzaffar
*/
void startReadingMacroInstructions(const uint8_t macroCode) {
    usbDebugValue(F("[FLOW][startReadingMacroInstructions] macro opcode="), macroCode);
    if (macroCode == kMotorHomeOpCode) {
        readState = MACRO;
        usbDebug(F("[FLOW][startReadingMacroInstructions] read state -> MACRO (home)"));
        motorInstructionBytesRead = 0;
        macroInstructionLength = kMotorHomeInstructionLength;
        motorInstructionsExpected = 1;
        receivedInstructions.clear();
        receivedInstructions.reserve(1);
    }
}

/**
 *
 * @author Emad Muzaffar
*/
void startReadingConfig() {
    readState = CONFIG;
    usbDebug(F("[FLOW][startReadingConfig] read state -> CONFIG"));
    configBytesRead = 0;
    receivedConfig = Config();
}

/**
 *
 * @author Emad Muzaffar
*/
uint16_t uint16ValueAt(const uint8_t * const byteBuffer, const uint8_t valueIndex) {
    const uint8_t byteIndex = valueIndex * kBytesPerUint16;
    return static_cast<uint16_t>(byteBuffer[byteIndex]) |
        (static_cast<uint16_t>(byteBuffer[byteIndex + 1]) << 8);
}

/**
 *
 * @author Emad Muzaffar
*/
void finishMotorInstructions() {
    if (readState == ON) {
        usbDebug(F("[FLOW][finishMotorInstructions] instruction packet complete; sending to motor"));
        motor.setInstructions(receivedInstructions.data(), receivedInstructions.size());
    } else if (readState == MACRO) {
        usbDebug(F("[FLOW][finishMotorInstructions] home packet complete; sending to motor"));
        motor.home(receivedInstructions.front().speed);
    }
    //reset system
    std::fill_n(motorInstructionByteBuffer, kMotorInstructionLength, 0);
    motorInstructionBytesRead = 0;
    motorInstructionsExpected = 0;
    macroInstructionLength = 0;
    readState = OFF;
    usbDebug(F("[FLOW][finishMotorInstructions] read state -> OFF"));
}

/**
 *
 * @author Emad Muzaffar
*/
void finishConfig() {
    usbDebug(F("[FLOW][finishConfig] configuration packet complete"));
    applyConfig(receivedConfig);
    //reset system
    receivedConfig = Config();
    std::fill_n(configByteBuffer, kConfigLength, 0);
    configBytesRead = 0;
    readState = OFF;
    usbDebug(F("[FLOW][finishConfig] read state -> OFF"));
}

/**
 *
 * @author Emad Muzaffar
*/
void readConfigByte() {
    if (!Serial1COM.available()) {
        return;
    }
    configByteBuffer[configBytesRead] = Serial1COM.readByte();
    configBytesRead++;
    usbDebugValue(F("[FLOW][readConfigByte] bytes received="), configBytesRead);

    // MATLAB sends one uint16 field per config state, so acknowledge only
    // after both bytes arrive. This keeps its eight states aligned with the
    // firmware's eight config fields.
    if (configBytesRead % kBytesPerUint16 == 0) {
        Serial1COM.writeByte(2);
    }

    if (configBytesRead == kConfigLength) {
        const uint16_t maxMotorRPM = uint16ValueAt(configByteBuffer, 0);
        const uint16_t configuredTimeMultiplier = uint16ValueAt(configByteBuffer, 1);
        const uint16_t tolerance = uint16ValueAt(configByteBuffer, 2);
        const uint16_t encoderPPR = uint16ValueAt(configByteBuffer, 3);
        const uint16_t maxRotation = uint16ValueAt(configByteBuffer, 4);
        const float kP = static_cast<float>(uint16ValueAt(configByteBuffer, 5)) / kConfigGainScale;
        const float kD = static_cast<float>(uint16ValueAt(configByteBuffer, 6)) / kConfigGainScale;
        const uint16_t correctiveInt = uint16ValueAt(configByteBuffer, 7);
        bool correctionEnabled = true;
        if (correctiveInt == 0) {
            correctionEnabled = false;
        }

        receivedConfig = Config(maxMotorRPM,
            configuredTimeMultiplier,
            tolerance,
            encoderPPR,
            maxRotation,
            kP,
            kD,
            correctionEnabled);
        finishConfig();
    }
}


/**
 *
 * @author Emad Muzaffar
*/
void readMotorInstructionByte(const uint8_t motorInstructionLength = kMotorInstructionLength) {
    if (!Serial1COM.available()) {
        return;
    }
    motorInstructionByteBuffer[motorInstructionBytesRead] = Serial1COM.readByte();
    motorInstructionBytesRead++;

    if (motorInstructionBytesRead == motorInstructionLength) {
        const uint16_t speed = uint16ValueAt(motorInstructionByteBuffer, 0);
        uint16_t time = 0;
        uint8_t direction = 0;
        if (motorInstructionLength >= 2 * kBytesPerUint16) {
            time = uint16ValueAt(motorInstructionByteBuffer, 1);
        }
        if (motorInstructionLength >= 2 * kBytesPerUint16 + sizeof(uint8_t)) {
            direction = motorInstructionByteBuffer[2 * kBytesPerUint16];
        }
        receivedInstructions.emplace_back(speed, time, direction);
        Serial1COM.writeByte(2);
        usbDebugValue(F("[FLOW][readMotorInstructionByte] complete instructions received="), receivedInstructions.size());
        motorInstructionBytesRead = 0;

        if (receivedInstructions.size() == motorInstructionsExpected) {
            finishMotorInstructions();
        }
    }
}

/**
 *
 * @author Emad Muzaffar
*/
void hostTransmit() {
    const uint32_t nowMicros = micros();

    if (nowMicros - hostTransmitTime > kCrossCheckDutyTime) {
        const uint8_t nextState = digitalRead(kHostTransmitPin) == HIGH ? LOW : HIGH;
        digitalWrite(kHostTransmitPin, nextState);
        hostTransmitTime = nowMicros;
    }
}

/**
 *
 * @author Emad Muzaffar
*/
void hostUpdate() {
    const uint32_t nowMicros = micros();

    if (millis() <= kHostStartTolerance) {
        lastHostEchoState = digitalRead(kHostReceivePin);
        lastHostEchoTime = nowMicros;
        hostTimeoutReported = false;
        return;
    }

    hostTransmit();

    const uint8_t echoState = digitalRead(kHostReceivePin);
    if (echoState != lastHostEchoState) {
        lastHostEchoState = echoState;
        lastHostEchoTime = nowMicros;
        hostTimeoutReported = false;
        return;
    }

    if (nowMicros - lastHostEchoTime > kCrossCheckMaxNoResponseTime && !hostTimeoutReported) {
        hostTimeoutReported = true;
        usbDebug(F("[FLOW][hostUpdate] eStop suppressed: host echo timed out"));
        // motor.safetynet.eStop(); //TODO: ESTOPPED HERE
    }
}

/**
 *
 * @author Emad Muzaffar
*/
void clientUpdate() {
    digitalWrite(kClientTransmitPin, digitalRead(kClientReceivePin));
}

/**
 *
 * @author Emad Muzaffar
*/
void crossCheckUpdate() {
    hostUpdate();
    clientUpdate();
}



/**
 * Setup
 * @author Emad Muzaffar
*/
void setup() {
    if (kUsbDebugEnabled) {
        Serial.begin(115200);
        delay(kUsbDebugStartupDelayMs);
        Serial.println();
        Serial.println(F("[RESET] ServoMotor firmware starting"));
        Serial.flush();
    }
    usbDebug(F("[FLOW][setup] USB debugging started"));
    motor.begin();
    Serial1.begin(1312500); //specific baud rate for Bpod
    usbDebug(F("[FLOW][setup] Bpod Serial1 started"));
    analogWriteResolution(16); //higher than default resolution for finer control,
    analogWrite(kPwmPin, 0); // ensure pwm is off
    pinMode(kHostTransmitPin, OUTPUT);
    pinMode(kHostReceivePin, INPUT);
    pinMode(kClientTransmitPin, OUTPUT);
    pinMode(kClientReceivePin, INPUT);
    digitalWrite(kHostTransmitPin, LOW);
    digitalWrite(kClientTransmitPin, LOW);
    lastHostEchoState = digitalRead(kHostReceivePin);
    lastHostEchoTime = micros();
    usbDebug(F("[FLOW][setup] complete; entering loop"));
}

/**
 * Main loop
 * @author Emad Muzaffar
*/
void loop() {
    if (readState == COUNT) {
        readMotorInstructionCountByte();
    } else if (readState == ON) {
        readMotorInstructionByte();
    } else if (readState == MACRO) {
        readMotorInstructionByte(macroInstructionLength);
    } else if (readState == CONFIG) {
        readConfigByte();
    } else if (Serial1COM.available()) {
        opCode = Serial1COM.readByte();
        usbDebugValue(F("[FLOW][loop] opcode received="), opCode);
        if (opCode == kModuleScanOpCode) {
            usbDebug(F("[FLOW][loop] route -> returnModuleInfo"));
            returnModuleInfo();
            motor.safetynet.setTickOffset();
        } else if (opCode == kDisableMotorOpCode) {
            //Turn enable off
            usbDebug(F("[FLOW][loop] route -> Safetynet::disable"));
            motor.safetynet.disable();
        } else if (opCode == kEnableMotorOpCode) {
            //Turn enable on
            usbDebug(F("[FLOW][loop] route -> Safetynet::enable"));
            motor.safetynet.enable();
        } else if (opCode == kMotorInstructionOpCode) {
            usbDebug(F("[FLOW][loop] route -> read instruction count"));
            readState = COUNT;
            readMotorInstructionCountByte();
        } else if (opCode == kMotorHomeOpCode) {
            usbDebug(F("[FLOW][loop] route -> startReadingMacroInstructions"));
            startReadingMacroInstructions(opCode);
        } else if (opCode == kEncoderResetOpCode) {
            usbDebug(F("[FLOW][loop] route -> Safetynet::setTickOffset"));
            motor.safetynet.setTickOffset();
        } else if (opCode == kConfigOpCode) {
            usbDebug(F("[FLOW][loop] route -> startReadingConfig"));
            startReadingConfig();
        } else {
            //Reads motor instructions
            usbDebug(F("[FLOW][loop] route -> startReadingMotorInstructions"));
            startReadingMotorInstructions(opCode);
            readMotorInstructionByte();
        }
    }
    if (motor.checkStatusAndUpdate()) {
        usbDebug(F("[FLOW][loop] motor complete; sending behavior byte 1 to Bpod"));
        Serial1COM.writeByte(1);
    }
    crossCheckUpdate();
}
