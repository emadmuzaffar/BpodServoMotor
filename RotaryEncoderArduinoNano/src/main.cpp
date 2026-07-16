
#include <Arduino.h>
#include "ArCom/ArCOM.h"

constexpr uint8_t kEncoderAPin = D2;
constexpr uint8_t kEncoderBPin = D3;
constexpr uint32_t kBpodBaudRate = 1312500;
constexpr uint32_t FirmwareVersion = 1;
constexpr char moduleName[] = "ServoMotor"; // ModuleName sent to Bpod in returnModuleInfo()

ArCOM Serial1COM(Serial1); // NOLINT(*-interfaces-global-init)
byte opCode = 0;
bool readingConfig = false;


volatile int32_t encoderTicks = 0;
volatile uint8_t previousEncoderState = 0;

void usbDebug(const __FlashStringHelper *message) {
    Serial.println(message);
}

static void IRAM_ATTR encoderISR() {
    const uint8_t currentState =
        (static_cast<uint8_t>(digitalRead(kEncoderAPin)) << 1) |
        static_cast<uint8_t>(digitalRead(kEncoderBPin));

    const uint8_t transition =
        static_cast<uint8_t>((previousEncoderState << 2) | currentState);

    switch (transition) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            encoderTicks++;
            break;

        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            encoderTicks--;
            break;

        default:
            break;
    }

    previousEncoderState = currentState;
}

static void setupEncoder() {
    pinMode(kEncoderAPin, INPUT_PULLUP);
    pinMode(kEncoderBPin, INPUT_PULLUP);

    previousEncoderState =
        (static_cast<uint8_t>(digitalRead(kEncoderAPin)) << 1) |
        static_cast<uint8_t>(digitalRead(kEncoderBPin));

    attachInterrupt(digitalPinToInterrupt(kEncoderAPin), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncoderBPin), encoderISR, CHANGE);
}

static float readEncoderSpeedTicksPerSecond() {
    static int32_t previousTicks = 0;
    static uint32_t previousTimeMicros = micros();

    const uint32_t currentTimeMicros = micros();
    noInterrupts();
    const int32_t currentTicks = encoderTicks;
    interrupts();
    const uint32_t elapsedMicros = currentTimeMicros - previousTimeMicros;

    if (elapsedMicros == 0) {
        return 0.0f;
    }

    const int32_t tickChange = currentTicks - previousTicks;
    const float speedTicksPerSecond =
        static_cast<float>(tickChange) * 1000000.0f /
        static_cast<float>(elapsedMicros);

    previousTicks = currentTicks;
    previousTimeMicros = currentTimeMicros;

    return speedTicksPerSecond;
}

void returnModuleInfo() {
    usbDebug(F("[FLOW][returnModuleInfo] sending module information"));
    Serial1COM.writeByte(65); // Acknowledge
    Serial1COM.writeUint32(FirmwareVersion); // 4-byte firmware version
    Serial1COM.writeByte(sizeof(moduleName) - 1); // Length of module name
    Serial1COM.writeCharArray(moduleName, sizeof(moduleName) - 1); // Module name
    Serial1COM.writeByte(1);                      // More info follows
    Serial1COM.writeByte('#');                    // Behavior event record
    Serial1COM.writeByte(3);                      // Number of events
    constexpr char eventName1[] = "SpeedAchieved";
    Serial1COM.writeByte(sizeof(eventName1) - 1);    // Length
    Serial1COM.writeCharArray(eventName1, sizeof(eventName1) - 1);
    Serial1COM.writeByte(0);
    usbDebug(F("[FLOW][returnModuleInfo] response complete"));
}

void startReadingConfig() {
    readingConfig = true;
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(kBpodBaudRate);
    setupEncoder();
}

void loop() {
    static uint32_t lastSpeedSampleMs = 0;

    if (millis() - lastSpeedSampleMs >= 100) {
        lastSpeedSampleMs = millis();

        const float speedTicksPerSecond = readEncoderSpeedTicksPerSecond();
        Serial.println(speedTicksPerSecond);
    }

    if (readingConfig) {
        
    } else if (Serial1COM.available()) {
        opCode = Serial1COM.readByte();

        if (opCode == 255) {
            usbDebug(F("[FLOW][returnModuleInfo] sending module information"));
        } else if (opCode == 1) {

        }
    }

}