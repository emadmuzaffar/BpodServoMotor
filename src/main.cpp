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

#include <vector>
#include <ArCom/ArCOM.h>

constexpr uint8_t kDirectionPin = 23;
constexpr uint8_t kPwmPin = 6;
constexpr uint8_t kEnablePin = 24;
constexpr uint8_t kStatusLedPin = 13;
constexpr float kMaxMotorRPM = 30.0f; // Must match the ClearPath MSP setup.
constexpr uint16_t kMaxPwm = 65535;
constexpr byte kModuleScanOpCode = 255;
constexpr byte kDisableMotorOpCode = 249;
constexpr byte kEnableMotorOpCode = 250;
constexpr byte kMotorHomeOpCode = 251;
constexpr uint8_t kMotorInstructionLength = 3;
constexpr uint32_t maxInstructionReadTime = 500;
constexpr int timeMultiplier = 100;


struct Instruction {
    uint8_t speed;
    uint8_t time;
    uint8_t direction;
    Instruction() : speed(0), time(0), direction(0) {}
    Instruction(uint8_t speed, uint8_t time, uint8_t direction)
        : speed(speed), time(time), direction(direction) {}
};

class Safetynet {
    const int32_t tolerance = 5000;
    uint8_t enablePin;
    unsigned long startTime = 0;
    float tPosition = 0;
    Instruction cInstruction;
    int32_t tickOffset = 0;
    bool eStopped = false;

    static float calculateInstructionDistance(Instruction instruction) {
        float distance = static_cast<float>(instruction.speed) * (static_cast<float>(instruction.time) * timeMultiplier);
        if (instruction.direction == 1) {
            distance = -distance;
        }
        return distance;
    }

    static float calculateInstructionDistance(Instruction instruction, unsigned long elapsedTime) {
        float distance = static_cast<float>(instruction.speed) * (static_cast<float>(elapsedTime));
        if (instruction.direction == 1) {
            distance = -distance;
        }
        return distance;
    }

    bool checkTolerance() const {
        float ctPosition = tPosition + -calculateInstructionDistance(cInstruction) + calculateInstructionDistance(cInstruction, micros() - startTime);
        int32_t error = abs(getTicks() - static_cast<int32_t>(ctPosition));
        if (error >= tolerance) {
            return true;
        }
        return false;
    }

    bool checkPositionSafety() const {
        return abs(getTicks()) > 29000;
    }

public:
     Safetynet(int enablePin) {
        this->enablePin = enablePin;
    }

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

    void reportInstruction(Instruction instruction) {
        startTime = millis();
        cInstruction = instruction;
        tPosition += calculateInstructionDistance(instruction);
    }

    double getTargetTicks() const {
        return tPosition/360 * 14400;
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
        pHOME,
        pCORRECTING,
        CORRECTING,
        pINSTRUCTED,
        INSTRUCTED
    };

    //Internal variables for motor to store data.
    motorState motorState = CORRECTING;
    Safetynet safetynet;
    uint8_t directionPin;
    uint8_t pwmPin;
    unsigned long startTime = 0;
    uint32_t durationMs = 0;
    size_t cInstruction = 0;
    std::vector<Instruction> instructions;
    bool running = false;
    double pidTarget = 0;
    double lastError = 0;
    double pidTime = 0;
    const float kP = 0.1;
    const float kD = 0.1;

    void setTimer(const uint8_t timerLength) {
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

    static float getVelocity(const uint8_t velocityDegPerSec) {
        float targetRPM = static_cast<float>(velocityDegPerSec) / 6.0f;
        return constrain(targetRPM, 0.0f, kMaxMotorRPM);
    }

    void setPWM(const uint8_t velocityDegPerSec) const {
        float targetRPM = getVelocity(velocityDegPerSec);
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

    void pid() {
        double error = safetynet.getTicks() - pidTarget;
        if (lastError == 0) {
            lastError = error;
        }

        double dt = micros() - pidTime;

        double p = error;

        double d = (error - lastError) / dt;

        double targetRPM = p * kP + d * kD;
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

    void internalRunInstruction(Instruction instruction) {
        setDirection(instruction.direction);
        setPWM(instruction.speed);
        setTimer(instruction.time);
        safetynet.reportInstruction(instruction);
    }

public:
    explicit Motor(uint8_t directionPin, uint8_t pwmPin, uint8_t enablePin) : safetynet(enablePin) {
        Safetynet::setupEncoder();
        this->directionPin = directionPin;
        this->pwmPin = pwmPin;
        pinMode(directionPin, OUTPUT);
        pinMode(pwmPin, OUTPUT);
        pinMode(enablePin, OUTPUT);
    }

    void enable() const {
        safetynet.enable();
    }

    void disable() const {
        safetynet.disable();
    }

    void setTickOffset() {
        safetynet.setTickOffset();
    }

    bool checkStatusAndUpdate() {
        safetynet.update();
        switch(motorState) {
            case CORRECTING:
                pid();
                return false;
                break;

            case INSTRUCTED:
                if (instructions.empty()) {
                    motorState = CORRECTING;
                }

                if (timerComplete()) {
                    cInstruction++;
                    if (cInstruction < instructions.size()) {
                        internalRunInstruction(instructions[cInstruction]);
                    } else {
                        instructions.clear();
                        motorState = CORRECTING;
                        return true;
                    }
                }
                break;

            default:
                // Safety fallback
                motorState = CORRECTING;
                break;
        }


        return false;
    }

    void setInstructions(const Instruction nInstructions[], size_t instructionCount) {
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

    void home(const u_int8_t speed) {
        int32_t cPos = safetynet.getTicks();
        double cPosDegrees = static_cast<double>(cPos) / 40;
        double time = cPosDegrees / speed;
        Instruction instruction[1];
        instruction[0].speed = speed;
        instruction[0].time = time;
        if (cPos > 0) {
            instruction[0].direction = 1;
        } else if (cPos < 0) {
            instruction[0].direction = 0;
        }
        setInstructions(instruction, 1);
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
byte opCode = 0; // Control opcode, or number of motor instructions to read.
uint8_t motorInstructionBuffer[kMotorInstructionLength] = {};
uint8_t motorInstructionBytesRead = 0;
uint8_t motorInstructionsExpected = 0;
std::vector<Instruction> receivedInstructions;
uint32_t readingStartTime = 0;
enum ReadState {
    OFF,
    ON,
    MACRO
};
ReadState readState = OFF;
int macroInstructionLength = 0;

/**
 * From SanWorks example
 * Writes info about Arduino system over serial to Bpod
 * Called in loop in switch(opCode) when opCode == 255, what Bpod sends to scan for modules
 * @authors JSNeuroDev, Emad Muzaffar
 */
void returnModuleInfo() {
    Serial1COM.writeByte(65); // Acknowledge
    Serial1COM.writeUint32(FirmwareVersion); // 4-byte firmware version
    Serial1COM.writeByte(sizeof(moduleName)-1); // Length of module name
    Serial1COM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
    Serial1COM.writeByte(1);                      // More info follows
    Serial1COM.writeByte('#');                    // Behavior event record
    Serial1COM.writeByte(2);                      // Number of events
    char eventName1[5] = "Done";
    Serial1COM.writeByte(sizeof(eventName1)-1);    // Length
    Serial1COM.writeCharArray(eventName1, sizeof(eventName1)-1);
    char eventName2[9] = "Received";
    Serial1COM.writeByte(sizeof(eventName2)-1);    // Length
    Serial1COM.writeCharArray(eventName2, sizeof(eventName2)-1);
    Serial1COM.writeByte(0);
}

void startReadingMotorInstructions(const uint8_t instructionCount) {
    motorInstructionBytesRead = 0;
    motorInstructionsExpected = instructionCount;
    receivedInstructions.clear();
    receivedInstructions.reserve(instructionCount);

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
        motorInstructionsExpected = 1;
        receivedInstructions.clear();
        receivedInstructions.reserve(1);
    }
}

void finishMotorInstructions() {
    if (readState == ON) {
        motor.setInstructions(receivedInstructions.data(), receivedInstructions.size());
    }
    else if (readState == MACRO) {
        motor.home(receivedInstructions.data()->speed);
    }
    motorInstructionBytesRead = 0;
    motorInstructionsExpected = 0;
    std::fill_n(motorInstructionBuffer, 2, 0);
    macroInstructionLength = 0;
    Serial1COM.writeByte(2);
    readState = OFF;
}

void cancelMotorInstructions() {
    motorInstructionBytesRead = 0;
    motorInstructionsExpected = 0;
    std::fill_n(motorInstructionBuffer, 2, 0);
    readState = OFF;
}

void readMotorInstructionByte(uint8_t motorInstructionLength = kMotorInstructionLength) {
    if (!Serial1COM.available()) {
        return;
    }
    motorInstructionBuffer[motorInstructionBytesRead] = Serial1COM.readByte();
    motorInstructionBytesRead++;

    if (motorInstructionBytesRead == motorInstructionLength) {
        uint8_t speed  = motorInstructionBuffer[0];
        uint8_t time = motorInstructionBuffer[1];
        uint8_t direction = motorInstructionBuffer[2];
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

/**
 * Setup
 * @author Emad Muzaffar
*/
void setup() {
    Serial1.begin(1312500); //specific baud rate for Bpod
    analogWriteResolution(16); //higher than default resolution for finer control,
    analogWrite(kPwmPin, 0); // ensure pwm is off

    //TODO: debug
    SerialUSB.begin(115200);
}

/**
 * Main loop
 * @author Emad Muzaffar
*/
//TODO: debug
double cMicros = 0;

void loop() {

    //TODO: debug
    SerialUSB.println(cMicros - micros());
    cMicros = micros();


    if (readState == ON) {
        readMotorInstructionByte();
    } else if (readState == MACRO) {
        readMotorInstructionByte(macroInstructionLength);
    } else if (Serial1COM.available()) {
        opCode = Serial1COM.readByte();
        if (opCode == kModuleScanOpCode) {
            returnModuleInfo();
            motor.setTickOffset();
        } else if (opCode == kDisableMotorOpCode) {
            //Turn enable off
            motor.disable();
        } else if (opCode == kEnableMotorOpCode) {
            //Turn enable on
            motor.enable();
        } else if (opCode == kMotorHomeOpCode) {
            startReadingMacroInstructions(opCode);
        } else {
            //reads motor instructions
            startReadingMotorInstructions(opCode);
            readMotorInstructionByte();
        }
    }
    if (motor.checkStatusAndUpdate()) Serial1COM.writeByte(1); //updates motor, writes behavior state byte to Bpod if completed
}
