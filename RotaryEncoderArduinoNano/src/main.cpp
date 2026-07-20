#include <Arduino.h>

namespace {

constexpr uint8_t kBpodRxPin = D0;
constexpr uint8_t kBpodTxPin = D1;
constexpr uint8_t kEncoderAPin = D2;
constexpr uint8_t kEncoderBPin = D3;

constexpr uint32_t kBpodBaudRate = 1312500;
constexpr uint32_t kFirmwareVersion = 1;
constexpr char kModuleName[] = "EmadRotaryEncoder";

constexpr uint8_t kModuleInfoOpCode = 255;
constexpr uint8_t kConfigureSpeedTriggerOpCode = 1;
constexpr uint8_t kSpeedAchievedEvent = 1;
constexpr uint8_t kConfigLength = 4;
constexpr uint32_t kConfigReceiveTimeoutMs = 100;

// Enable this while testing over USB. Bpod communication always uses Serial1.
constexpr bool kUsbDebugEnabled = true;

volatile int64_t encoderTicks = 0;
volatile uint8_t previousEncoderState = 0;

uint8_t configBuffer[kConfigLength] = {};
uint8_t configBytesRead = 0;
bool readingConfig = false;
uint32_t lastConfigByteTimeMs = 0;

uint16_t requiredSpeedTicksPerSecond = 0;
uint16_t integrationTimeMs = 0;
bool speedTriggerArmed = false;
int64_t integrationStartTicks = 0;
uint32_t integrationStartTimeMs = 0;

void usbDebug(const __FlashStringHelper *message) {
    if (kUsbDebugEnabled) {
        Serial.println(message);
    }
}

int64_t getEncoderTicks() {
    noInterrupts();
    const int64_t ticks = encoderTicks;
    interrupts();
    return ticks;
}

void IRAM_ATTR encoderISR() {
    const uint8_t currentState =
        (static_cast<uint8_t>(digitalRead(kEncoderAPin)) << 1U) |
        static_cast<uint8_t>(digitalRead(kEncoderBPin));

    const auto transition =
        static_cast<uint8_t>((previousEncoderState << 2U) | currentState);

    switch (transition) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            ++encoderTicks;
            break;

        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            --encoderTicks;
            break;

        default:
            break;
    }

    previousEncoderState = currentState;
}

void setupEncoder() {
    pinMode(kEncoderAPin, INPUT_PULLUP);
    pinMode(kEncoderBPin, INPUT_PULLUP);

    previousEncoderState =
        (static_cast<uint8_t>(digitalRead(kEncoderAPin)) << 1U) |
        static_cast<uint8_t>(digitalRead(kEncoderBPin));

    attachInterrupt(digitalPinToInterrupt(kEncoderAPin), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncoderBPin), encoderISR, CHANGE);
}

void writeUint32LittleEndian(const uint32_t value) {
    Serial1.write(static_cast<uint8_t>(value));
    Serial1.write(static_cast<uint8_t>(value >> 8U));
    Serial1.write(static_cast<uint8_t>(value >> 16U));
    Serial1.write(static_cast<uint8_t>(value >> 24U));
}

uint16_t readUint16LittleEndian(const uint8_t *bytes) {
    return static_cast<uint16_t>(bytes[0]) |
           (static_cast<uint16_t>(bytes[1]) << 8U);
}

void returnModuleInfo() {
    usbDebug(F("[FLOW] Sending module information"));

    Serial1.write('A');
    writeUint32LittleEndian(kFirmwareVersion);
    Serial1.write(sizeof(kModuleName) - 1U);
    Serial1.write(reinterpret_cast<const uint8_t *>(kModuleName),
                  sizeof(kModuleName) - 1U);

    Serial1.write(1);   // One additional information record follows.
    Serial1.write('#'); // Behavior-event-name record.
    Serial1.write(1);   // One behavior event.

    constexpr char kEventName[] = "SpeedAchieved";
    Serial1.write(sizeof(kEventName) - 1U);
    Serial1.write(reinterpret_cast<const uint8_t *>(kEventName),
                  sizeof(kEventName) - 1U);

    Serial1.write(0); // End of additional information.
    Serial1.flush();
}

void startReadingConfig() {
    readingConfig = true;
    configBytesRead = 0;
    lastConfigByteTimeMs = millis();

    // Do not allow the old configuration to fire while a replacement packet
    // is only partially received.
    speedTriggerArmed = false;
}

void applyReceivedConfig() {
    requiredSpeedTicksPerSecond = readUint16LittleEndian(configBuffer) * 360/1024;
    integrationTimeMs = readUint16LittleEndian(configBuffer + 2);

    readingConfig = false;
    configBytesRead = 0;

    if (integrationTimeMs == 0) {
        speedTriggerArmed = false;
        usbDebug(F("[FLOW] Ignored config with a zero integration time"));
        return;
    }

    integrationStartTicks = getEncoderTicks();
    integrationStartTimeMs = millis();
    speedTriggerArmed = true;

    if (kUsbDebugEnabled) {
        Serial.print(F("[FLOW] Armed: threshold="));
        Serial.print(requiredSpeedTicksPerSecond);
        Serial.print(F(" ticks/s, window="));
        Serial.print(integrationTimeMs);
        Serial.println(F(" ms"));
    }
}

void recoverTimedOutConfig() {
    if (readingConfig &&
        millis() - lastConfigByteTimeMs >= kConfigReceiveTimeoutMs) {
        readingConfig = false;
        configBytesRead = 0;
        usbDebug(F("[FLOW] Incomplete opcode 1 packet discarded"));
    }
}

void processBpodSerial() {
    recoverTimedOutConfig();

    while (Serial1.available() > 0) {
        if (readingConfig) {
            configBuffer[configBytesRead++] =
                static_cast<uint8_t>(Serial1.read());
            lastConfigByteTimeMs = millis();

            if (configBytesRead == kConfigLength) {
                applyReceivedConfig();
            }
            continue;
        }

        const uint8_t opCode = static_cast<uint8_t>(Serial1.read());

        if (opCode == kModuleInfoOpCode) {
            returnModuleInfo();
        } else if (opCode == kConfigureSpeedTriggerOpCode) {
            startReadingConfig();
        }
    }
}

void checkSpeedTrigger() {
    if (!speedTriggerArmed) {
        return;
    }

    const uint32_t currentTimeMs = millis();
    const uint32_t elapsedMs = currentTimeMs - integrationStartTimeMs;

    if (elapsedMs < integrationTimeMs) {
        return;
    }

    const int64_t currentTicks = getEncoderTicks();
    const int64_t signedTickChange = currentTicks - integrationStartTicks;
    const uint64_t absoluteTickChange = signedTickChange < 0
                                            ? static_cast<uint64_t>(-signedTickChange)
                                            : static_cast<uint64_t>(signedTickChange);

    // Compare |ticks| / elapsed time against the requested ticks/second
    // without rounding the measured speed down to an integer.
    const uint64_t measuredScaled = absoluteTickChange * 1000ULL;
    const uint64_t requiredScaled =
        static_cast<uint64_t>(requiredSpeedTicksPerSecond) * elapsedMs;
    const bool speedExceeded = measuredScaled > requiredScaled;

    if (kUsbDebugEnabled) {
        const float measuredSpeed =
            static_cast<float>(measuredScaled) / static_cast<float>(elapsedMs);
        Serial.print(F("[FLOW] Average speed="));
        Serial.print(measuredSpeed);
        Serial.println(F(" ticks/s"));
    }

    if (speedExceeded) {
        Serial1.write(kSpeedAchievedEvent);
        usbDebug(F("[FLOW] SpeedAchieved event sent to Bpod"));
    }

    // Continue monitoring in consecutive, non-overlapping windows. Every
    // window above the threshold produces a SpeedAchieved event.
    integrationStartTicks = currentTicks;
    integrationStartTimeMs = currentTimeMs;
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println(F("EmadRotaryEncoder firmware started"));
    Serial1.begin(kBpodBaudRate, SERIAL_8N1, kBpodRxPin, kBpodTxPin);
    setupEncoder();
}

void loop() {
    processBpodSerial();
    checkSpeedTrigger();
}
