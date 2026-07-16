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
constexpr uint8_t kFrequencyOutputPin = 6; // ClearPath Input B; D6 is the Due's PWML7 output.
constexpr uint8_t kEnablePin = 53;
constexpr uint8_t kHostTransmitPin = 52;
constexpr uint8_t kHostReceivePin = 50;
constexpr uint8_t kClientTransmitPin = 48;
constexpr uint8_t kClientReceivePin = 46;
constexpr float kMaxMotorRPM = 120; // Must match the ClearPath MSP setup.
float MaxMotorRPM = kMaxMotorRPM;
constexpr byte kModuleScanOpCode = 255;
constexpr byte kDisableMotorOpCode = 249;
constexpr byte kEnableMotorOpCode = 250;
constexpr byte kMotorInstructionOpCode = 1;
constexpr byte kMotorHomeOpCode = 200;
constexpr byte kEncoderResetOpCode = 253;
constexpr byte kConfigOpCode = 254;
constexpr byte kDoneEvent = 1;
constexpr byte kReceivedEvent = 2;
constexpr byte kCorrectionDoneEvent = 3;
constexpr uint8_t kBytesPerUint16 = 2;
constexpr uint8_t kMotorInstructionLength = 2 * kBytesPerUint16 + sizeof(uint8_t);
constexpr uint8_t kMotorHomeInstructionLength = kBytesPerUint16;
constexpr uint8_t kConfigFieldCount = 9;
constexpr uint8_t kConfigLength = kConfigFieldCount * kBytesPerUint16;
constexpr uint32_t kHostStartTolerance = 1000;
constexpr uint32_t kCrossCheckMaxNoResponseTime = 5000;
constexpr uint32_t kCrossCheckDutyTime = 500;
constexpr uint32_t kSerialPacketTimeoutMs = 1000;
constexpr uint8_t kMaxMotorInstructions = 199;
constexpr double kDegreesPerRotation = 360.0;
constexpr double kMillisPerSecond = 1000.0;
constexpr double kMicrosPerSecond = 1000000.0;
constexpr double kMaxInstructionTime = 65535.0;
constexpr float kConfigGainScale = 1000.0f;
constexpr int kTimeMultiplier = 1;
int timeMultiplier = kTimeMultiplier;
constexpr int kEncoderPPR = 14400;
constexpr int kTolerance = 300;
constexpr int kDisableTolerance = kEncoderPPR / 2;
constexpr uint32_t kToleranceTripDelayMs = 2000;
constexpr float defaultKP = 0.1f;
constexpr float defaultKD = 0.1f;
constexpr bool kCorrectionEnabled = true;
constexpr uint32_t kCorrectionStartDelayMs = 0;
constexpr bool kUsbDebugEnabled = true; // Central switch for all USB debugging.
constexpr uint32_t kUsbDebugStartupDelayMs = 500;
constexpr int32_t kMaxRotation = kEncoderPPR * 2;
constexpr uint32_t kFrequencyPwmChannel = PWM_CH7;
constexpr uint32_t kWaveformClockHz = VARIANT_MCK; // Run the waveform generator directly from the 84 MHz master clock.
constexpr uint32_t kMinFrequency = 2000;
constexpr uint32_t kMaxFrequency = 4000;
constexpr uint32_t kFrequencyRange = kMaxFrequency - kMinFrequency;

static_assert(kMinFrequency >= 20, "ClearPath MCPV frequency input must be at least 20 Hz");
static_assert(kMaxFrequency <= 700000, "ClearPath MCPV frequency input must not exceed 700 kHz");

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
    uint32_t correctionStartDelayMs;
    Config() : maxMotorRPM(kMaxMotorRPM), timeMultiplier(kTimeMultiplier), tolerance(kTolerance), encoderPPR(kEncoderPPR), maxRotation(kMaxRotation), kP(defaultKP), kD(defaultKD), correctionEnabled(kCorrectionEnabled), correctionStartDelayMs(kCorrectionStartDelayMs) {}
    Config(const float maxMotorRPM, const int timeMultiplier, const int tolerance, const int encoderPPR, const int maxRotation, const float kP, const float kD, const bool correctionEnabled, const uint16_t correctionStartDelayMs)
        : maxMotorRPM(maxMotorRPM),
            timeMultiplier(timeMultiplier),
            tolerance(tolerance),
            encoderPPR(static_cast<int32_t>(encoderPPR) * 4),
            maxRotation(rotationLimitTicks(maxRotation, encoderPPR)),
            kP(kP),
            kD(kD),
            correctionEnabled(correctionEnabled),
            correctionStartDelayMs(correctionStartDelayMs) {}
};

/**
*
* @author Emad Muzaffar
*/
class Safetynet {
    friend class Motor; //Allow motor to access private

    //Internal variables for safetynet to store data.
    int32_t tolerance = kTolerance;
    int32_t disableTolerance = kDisableTolerance;
    int32_t encoderPPR = kEncoderPPR;
    int32_t maxRotation = kMaxRotation;
    const uint8_t enablePin;
    unsigned long startTime = 0;
    double tPosition = 0;
    Instruction cInstruction;
    int32_t tickOffset = 0;
    bool eStopped = false;
    bool toleranceTimerActive = false;
    unsigned long toleranceExceededSinceMs = 0;

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
    bool checkTolerance() {
        const double targetTicks = degreesToTicks(currentTargetDegrees());
        const double error = absoluteValue(static_cast<double>(getTicks()) - targetTicks);
        if (error < disableTolerance) {
            toleranceTimerActive = false;
            return false;
        }

        const unsigned long nowMs = millis();
        if (!toleranceTimerActive) {
            toleranceTimerActive = true;
            toleranceExceededSinceMs = nowMs;
            return false;
        }

        return nowMs - toleranceExceededSinceMs >= kToleranceTripDelayMs;
    }

    bool checkCorrective() const {
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
        disableTolerance = config.encoderPPR / 2;
        encoderPPR = config.encoderPPR;
        maxRotation = config.maxRotation;
        toleranceTimerActive = false;
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

        // D2/PB25 = TIOA0 and D13/PB27 = TIOB0 on peripheral B.
        PIO_Configure(
            PIOB,
            PIO_PERIPH_B,
            PIO_PB25B_TIOA0 | PIO_PB27B_TIOB0,
            PIO_DEFAULT);

        // Count both edges of PHA and PHB for x4 quadrature decoding.
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

    void resetPositionReference() {
        setTickOffset();
        tPosition = 0.0;
        cInstruction = Instruction();
        startTime = micros();
        toleranceTimerActive = false;
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
    * Enable the Motor
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
        static unsigned long lastDebugPrintMs = 0;
        const unsigned long nowMs = millis();

        if (kUsbDebugEnabled && nowMs - lastDebugPrintMs >= 1000) {
            lastDebugPrintMs = nowMs;
            const int32_t measuredTicks = getTicks();
            const double targetTicks = degreesToTicks(currentTargetDegrees());
            Serial.print(F("[FLOW][Safetynet::telemetry] measuredTicks="));
            Serial.print(measuredTicks);
            Serial.print(F(" targetTicks="));
            Serial.print(targetTicks);
            Serial.print(F(" errorTicks="));
            Serial.print(static_cast<double>(measuredTicks) - targetTicks);
            Serial.print(F(" enable="));
            Serial.print(digitalRead(enablePin));
            Serial.print(F(" eStopped="));
            Serial.println(eStopped);
        }

        if (eStopped == true) {
            return;
        }

        if (checkPositionSafety()) {
            usbDebug(F("[FLOW][Safetynet::update] eStop: position safety limit exceeded"));
            eStop();
            return;
        }
        if (checkTolerance()) {
            usbDebug(F("[FLOW][Safetynet::update] eStop: tracking tolerance exceeded for 2 seconds"));
            eStop();
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
    uint32_t correctionStartDelayMs = kCorrectionStartDelayMs;
    uint32_t correctionDelayStartedAtMs = 0;

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

    static void setFrequencyOutput(const uint32_t requestedFrequencyHz) {
        const uint32_t frequencyHz = std::min(
            std::max(requestedFrequencyHz, kMinFrequency),
            kMaxFrequency);
        // Round to the nearest whole waveform period. At the configured endpoints,
        // 84 MHz / 42,000 = 2 kHz and 84 MHz / 21,000 = 4 kHz exactly.
        const auto period = static_cast<uint16_t>((kWaveformClockHz + frequencyHz / 2) / frequencyHz);

        PWMC_SetPeriod(PWM_INTERFACE, kFrequencyPwmChannel, period);
        PWMC_SetDutyCycle(PWM_INTERFACE, kFrequencyPwmChannel, period / 2);
    }

    static void setupFrequencyOutput() {
        // The SAM3X PWM peripheral is used only as a hardware waveform
        // generator here; frequency, rather than duty cycle, commands velocity.
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk(PWM_INTERFACE_ID);
        // Connect Arduino pin 6 to its PWML7 peripheral.
        PIO_Configure(
            g_APinDescription[kFrequencyOutputPin].pPort,
            g_APinDescription[kFrequencyOutputPin].ulPinType,
            g_APinDescription[kFrequencyOutputPin].ulPin,
            g_APinDescription[kFrequencyOutputPin].ulPinConfiguration
        );
        PWMC_ConfigureChannel(
            PWM_INTERFACE,
            kFrequencyPwmChannel,
            PWM_CMR_CPRE_MCK,
            0,
            0);
        setFrequencyOutput(kMinFrequency);
        PWMC_EnableChannel(PWM_INTERFACE, kFrequencyPwmChannel);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void writeFrequencyForRPM(const double rpm) const {
        if (MaxMotorRPM <= 0.0f) {
            setFrequencyOutput(kMinFrequency);
            return;
        }

        const double clampedRPM = std::min(std::max(rpm, 0.0), static_cast<double>(MaxMotorRPM));
        const auto frequencyHz = static_cast<uint32_t>(std::lround(
            static_cast<double>(kMinFrequency) +
            clampedRPM / static_cast<double>(MaxMotorRPM) * static_cast<double>(kFrequencyRange)));
        setFrequencyOutput(frequencyHz);
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void setVelocityCommand(const uint16_t velocityDegPerSec) const {
        writeFrequencyForRPM(getVelocity(velocityDegPerSec));
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
        const double rawError = static_cast<double>(safetynet.getTicks()) - pidTarget;
        const double error = rawError > safetynet.tolerance
            ? rawError - safetynet.tolerance
            : (rawError < -safetynet.tolerance ? rawError + safetynet.tolerance : 0.0);
        const double dtSeconds = (nowMicros - pidTime) / kMicrosPerSecond;
        const double p = error;
        const double d = pidTime > 0.0 && dtSeconds > 0.0
            ? (error - lastError) / dtSeconds
            : 0.0;
        const double targetRPM = p * kP + d * kD;

        if (targetRPM > 0.0) {
            setDirection(1);
        } else if (targetRPM < 0.0) {
            setDirection(0);
        }
        writeFrequencyForRPM(targetRPM < 0.0 ? -targetRPM : targetRPM);
        pidTime = nowMicros;
        lastError = error;
    }

    bool updateCorrection() {
        if (safetynet.checkCorrective()) {
            pid();
            return false;
        }

        writeFrequencyForRPM(0.0);
        lastError = 0;
        pidTime = 0;
        return true;
    }

    /**
    *
    * @author Emad Muzaffar
    */
    bool correctionPID() {
        pidTarget = safetynet.getTargetTicks();
        return updateCorrection();
    }

    /**
    *
    * @author Emad Muzaffar
    */
    void commandZeroVelocity() {
        setFrequencyOutput(kMinFrequency);
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
        setVelocityCommand(instruction.speed);
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
    explicit Motor(const uint8_t directionPin, const uint8_t enablePin)
        : directionPin(directionPin), safetynet(enablePin) {}

    void begin() const {
        Safetynet::setupEncoder();
        pinMode(directionPin, OUTPUT);
        safetynet.begin();
        setupFrequencyOutput();
        usbDebug(F("[FLOW][Motor::begin] motor hardware initialized"));
    }

    /**
    *
    * @author Emad Muzaffar
    */
    uint8_t checkStatusAndUpdate() {
        safetynet.update();
        if (safetynet.eStopped) {
            if (motorState != INACTIVE || running) {
                instructions.clear();
                motorState = INACTIVE;
                commandZeroVelocity();
                usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] emergency stop aborted active motion"));
            }
            return 0;
        }

        switch (motorState) {
            case INACTIVE:
                return 0;

            case pCORRECTING:
                if (!correctionEnabled) {
                    motorState = INACTIVE;
                    safetynet.disable();
                    commandZeroVelocity();
                    usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] correction disabled; sequence complete; state -> INACTIVE"));
                    return kCorrectionDoneEvent;
                }
                if (millis() - correctionDelayStartedAtMs < correctionStartDelayMs) {
                    return 0;
                }
                if (correctionPID()) {
                    motorState = INACTIVE;
                    usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] correction already within tolerance; state -> INACTIVE"));
                    return kCorrectionDoneEvent;
                }
                motorState = CORRECTING;
                usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] state -> CORRECTING"));
                return 0;

            case CORRECTING:
                if (updateCorrection()) {
                    motorState = INACTIVE;
                    usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] correction complete; state -> INACTIVE"));
                    return kCorrectionDoneEvent;
                }
                return 0;

            case pINSTRUCTED:
                if (instructions.empty()) {
                    motorState = CORRECTING;
                    usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] no queued instructions; state -> CORRECTING"));
                } else {
                    motorState = INSTRUCTED;
                    usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] state -> INSTRUCTED"));
                }
                return 0;

            case INSTRUCTED:
                if (timerComplete()) {
                    cInstruction++;
                    if (cInstruction < instructions.size()) {
                        usbDebugValue(F("[FLOW][Motor::checkStatusAndUpdate] starting instruction index="), cInstruction);
                        internalRunInstruction(instructions[cInstruction]);
                    } else {
                        instructions.clear();
                        commandZeroVelocity();
                        correctionDelayStartedAtMs = millis();
                        motorState = pCORRECTING;
                        usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] timed sequence complete; Done sent; correction delay started"));
                        return kDoneEvent;
                    }
                }
                return 0;

            default:
                // Safety fallback
                motorState = CORRECTING;
                usbDebug(F("[FLOW][Motor::checkStatusAndUpdate] invalid state; fallback -> CORRECTING"));
                return 0;
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
            usbDebug(F("[FLOW][Motor::setInstructions] instruction list empty; commanding zero velocity"));
            commandZeroVelocity();
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
            commandZeroVelocity();
            return;
        }

        const double cPosDegrees = safetynet.ticksToDegrees(cPos);
        const double timeSeconds = Safetynet::absoluteValue(cPosDegrees) / static_cast<double>(speed);
        const double timeUnits = timeSeconds * kMillisPerSecond / static_cast<double>(timeMultiplier);
        Instruction instruction[1];
        instruction[0].speed = speed;
        instruction[0].time = static_cast<uint16_t>(std::min(
            std::max(std::ceil(timeUnits), 1.0),
            kMaxInstructionTime));
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
        correctionStartDelayMs = config.correctionStartDelayMs;
        safetynet.applyConfig(config);
    }

    void resetPositionReference() {
        instructions.clear();
        motorState = INACTIVE;
        commandZeroVelocity();
        safetynet.disable();
        safetynet.resetPositionReference();
        usbDebug(F("[FLOW][Motor::resetPositionReference] motion stopped and measured/target position reset"));
    }

};


/**
 * Initialize objects and module setup
 * @author Emad Muzaffar
 */
constexpr uint32_t FirmwareVersion = 1;
constexpr char moduleName[] = "ServoMotor"; // ModuleName sent to Bpod in returnModuleInfo()
ArCOM Serial1COM(Serial1); // NOLINT(*-interfaces-global-init)
Motor motor(kDirectionPin, kEnablePin);
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

void resetSerialReadState() {
    std::fill_n(motorInstructionByteBuffer, kMotorInstructionLength, 0);
    std::fill_n(configByteBuffer, kConfigLength, 0);
    motorInstructionBytesRead = 0;
    motorInstructionsExpected = 0;
    configBytesRead = 0;
    macroInstructionLength = 0;
    receivedInstructions.clear();
    receivedConfig = Config();
    readState = OFF;
    readingStartTime = 0;
}

void markSerialReadProgress() {
    readingStartTime = millis();
}

void recoverTimedOutSerialPacket() {
    if (readState != OFF && millis() - readingStartTime >= kSerialPacketTimeoutMs) {
        usbDebug(F("[FLOW][recoverTimedOutSerialPacket] incomplete packet discarded; read state -> OFF"));
        resetSerialReadState();
    }
}

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
    Serial1COM.writeByte(3);                      // Number of events
    constexpr char eventName1[] = "Done";
    Serial1COM.writeByte(sizeof(eventName1) - 1);    // Length
    Serial1COM.writeCharArray(eventName1, sizeof(eventName1) - 1);
    constexpr char eventName2[] = "Received";
    Serial1COM.writeByte(sizeof(eventName2) - 1);    // Length
    Serial1COM.writeCharArray(eventName2, sizeof(eventName2) - 1);
    constexpr char eventName3[] = "CorrectionDone";
    Serial1COM.writeByte(sizeof(eventName3) - 1);    // Length
    Serial1COM.writeCharArray(eventName3, sizeof(eventName3) - 1);
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
    if (instructionCount > kMaxMotorInstructions) {
        usbDebug(F("[FLOW][startReadingMotorInstructions] rejected: instruction count exceeds protocol maximum"));
        resetSerialReadState();
        return;
    }

    motorInstructionBytesRead = 0;
    motorInstructionsExpected = instructionCount;
    receivedInstructions.clear();
    receivedInstructions.reserve(motorInstructionsExpected);
    markSerialReadProgress();

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
    markSerialReadProgress();
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
        markSerialReadProgress();
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
    markSerialReadProgress();
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
    resetSerialReadState();
    usbDebug(F("[FLOW][finishMotorInstructions] read state -> OFF"));
}

/**
 *
 * @author Emad Muzaffar
*/
void finishConfig() {
    usbDebug(F("[FLOW][finishConfig] configuration packet complete"));
    applyConfig(receivedConfig);
    resetSerialReadState();
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
    markSerialReadProgress();
    usbDebugValue(F("[FLOW][readConfigByte] bytes received="), configBytesRead);

    // MATLAB sends one uint16 field per config state, so acknowledge only
    // after both bytes arrive. This keeps its nine states aligned with the
    // firmware's nine config fields.
    if (configBytesRead % kBytesPerUint16 == 0) {
        Serial1COM.writeByte(kReceivedEvent);
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
        const uint16_t correctionStartDelayMs = uint16ValueAt(configByteBuffer, 8);
        if (maxMotorRPM == 0 || configuredTimeMultiplier == 0 || encoderPPR == 0 ||
            maxRotation == 0 || correctiveInt > 1) {
            usbDebug(F("[FLOW][readConfigByte] rejected invalid configuration; emergency stop latched"));
            motor.safetynet.eStop();
            resetSerialReadState();
            return;
        }
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
            correctionEnabled,
            correctionStartDelayMs);
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
    markSerialReadProgress();

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
        if (direction > 1) {
            usbDebug(F("[FLOW][readMotorInstructionByte] rejected: direction must be 0 or 1"));
            motor.safetynet.eStop();
            resetSerialReadState();
            return;
        }
        receivedInstructions.emplace_back(speed, time, direction);
        Serial1COM.writeByte(kReceivedEvent);
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
        // usbDebug(F("[FLOW][hostUpdate] eStop: host echo timed out"));
        // motor.safetynet.eStop();
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
    pinMode(kHostTransmitPin, OUTPUT);
    pinMode(kHostReceivePin, INPUT_PULLUP);
    pinMode(kClientTransmitPin, OUTPUT);
    pinMode(kClientReceivePin, INPUT_PULLUP);
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
    recoverTimedOutSerialPacket();

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
            motor.resetPositionReference();
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
            markSerialReadProgress();
            readMotorInstructionCountByte();
        } else if (opCode == kMotorHomeOpCode) {
            usbDebug(F("[FLOW][loop] route -> startReadingMacroInstructions"));
            startReadingMacroInstructions(opCode);
        } else if (opCode == kEncoderResetOpCode) {
            usbDebug(F("[FLOW][loop] route -> Motor::resetPositionReference"));
            motor.resetPositionReference();
        } else if (opCode == kConfigOpCode) {
            usbDebug(F("[FLOW][loop] route -> startReadingConfig"));
            startReadingConfig();
        } else {
            usbDebug(F("[FLOW][loop] ignored unknown opcode"));
        }
    }
    const uint8_t motorEvent = motor.checkStatusAndUpdate();
    if (motorEvent != 0) {
        usbDebugValue(F("[FLOW][loop] sending behavior event to Bpod="), motorEvent);
        Serial1COM.writeByte(motorEvent);
    }
    crossCheckUpdate();
}
