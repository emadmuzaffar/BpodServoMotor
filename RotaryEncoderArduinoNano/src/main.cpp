#include <Arduino.h>

namespace {

constexpr uint8_t kBpodRxPin = D0;
constexpr uint8_t kBpodTxPin = D1;
constexpr uint8_t kTreadmillSpeedPin = D2;

constexpr uint32_t kBpodBaudRate = 1312500;
constexpr uint32_t kFirmwareVersion = 1;
constexpr char kModuleName[] = "EmadRotaryEncoder";

constexpr uint16_t kTreadmillOutputFullScaleMv = 2500;
constexpr uint16_t kTreadmillMaxSpeedMmPerSecond = 1000;

constexpr uint8_t kModuleInfoOpCode = 255;
constexpr uint8_t kConfigureSpeedTriggerOpCode = 1;
constexpr uint8_t kSpeedAchievedEvent = 1;
constexpr uint8_t kConfigLength = 4;
constexpr uint32_t kConfigReceiveTimeoutMs = 100;

// testing over USB
constexpr bool kUsbDebugEnabled = true;

uint8_t configBuffer[kConfigLength] = {};
uint8_t configBytesRead = 0;
bool readingConfig = false;
uint32_t lastConfigByteTimeMs = 0;

uint16_t requiredSpeedMmPerSecond = 0;
uint16_t integrationTimeMs = 0;
bool speedTriggerArmed = false;
uint32_t integrationStartTimeMs = 0;
uint32_t lastSpeedIntegralUpdateTimeMs = 0;
uint16_t lastSpeedMmPerSecond = 0;
uint64_t integratedSpeedMmPerSecondMilliseconds = 0;

void usbDebug(const __FlashStringHelper *message) {
    if (kUsbDebugEnabled) {
        Serial.println(message);
    }
}

void setupTreadmillSpeedInput() {
    pinMode(kTreadmillSpeedPin, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(kTreadmillSpeedPin, ADC_11db);
}

uint16_t readTreadmillSpeedMmPerSecond() {
    const uint32_t speedOutputMv = analogReadMilliVolts(kTreadmillSpeedPin);
    uint32_t speedMmPerSecond =
        (speedOutputMv * kTreadmillMaxSpeedMmPerSecond +
         kTreadmillOutputFullScaleMv / 2U) /
        kTreadmillOutputFullScaleMv;

    if (speedMmPerSecond > kTreadmillMaxSpeedMmPerSecond) {
        speedMmPerSecond = kTreadmillMaxSpeedMmPerSecond;
    }

    return static_cast<uint16_t>(speedMmPerSecond);
}

void startSpeedIntegrationWindow() {
    const uint32_t currentTimeMs = millis();
    integrationStartTimeMs = currentTimeMs;
    lastSpeedIntegralUpdateTimeMs = currentTimeMs;
    lastSpeedMmPerSecond = readTreadmillSpeedMmPerSecond();
    integratedSpeedMmPerSecondMilliseconds = 0;
}

uint32_t sampleAndUpdateSpeedIntegral() {
    const uint16_t newSpeedMmPerSecond =
        readTreadmillSpeedMmPerSecond();
    const uint32_t currentTimeMs = millis();
    const uint32_t elapsedSinceIntegralUpdateMs =
        currentTimeMs - lastSpeedIntegralUpdateTimeMs;

    if (speedTriggerArmed) {
        integratedSpeedMmPerSecondMilliseconds +=
            static_cast<uint64_t>(lastSpeedMmPerSecond) *
            elapsedSinceIntegralUpdateMs;
    }

    lastSpeedIntegralUpdateTimeMs = currentTimeMs;
    lastSpeedMmPerSecond = newSpeedMmPerSecond;

    return currentTimeMs;
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

    // Do not allow the old configuration to fire while a replacement packet is only partially received.
    speedTriggerArmed = false;
}

void applyReceivedConfig() {
    requiredSpeedMmPerSecond = readUint16LittleEndian(configBuffer);
    integrationTimeMs = readUint16LittleEndian(configBuffer + 2);

    readingConfig = false;
    configBytesRead = 0;

    if (integrationTimeMs == 0) {
        speedTriggerArmed = false;
        usbDebug(F("[FLOW] Ignored config with a zero integration time"));
        return;
    }

    startSpeedIntegrationWindow();
    speedTriggerArmed = true;

    if (kUsbDebugEnabled) {
        Serial.print(F("[FLOW] Armed: threshold="));
        Serial.print(requiredSpeedMmPerSecond);
        Serial.print(F(" mm/s, window="));
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

void checkSpeedTrigger(const uint32_t currentTimeMs) {
    if (!speedTriggerArmed) {
        return;
    }

    const uint32_t elapsedMs = currentTimeMs - integrationStartTimeMs;

    if (elapsedMs < integrationTimeMs) {
        return;
    }

    // Compare the time-integrated analog speed against the requested average
    // without rounding the measured average down to an integer.
    const uint64_t measuredScaled =
        integratedSpeedMmPerSecondMilliseconds;
    const uint64_t requiredScaled =
        static_cast<uint64_t>(requiredSpeedMmPerSecond) * elapsedMs;
    const bool speedExceeded = measuredScaled > requiredScaled;

    if (kUsbDebugEnabled) {
        const float measuredSpeed =
            static_cast<float>(measuredScaled) / static_cast<float>(elapsedMs);
        Serial.print(F("[FLOW] Average speed="));
        Serial.print(measuredSpeed);
        Serial.println(F(" mm/s"));
    }

    if (speedExceeded) {
        Serial1.write(kSpeedAchievedEvent);
        usbDebug(F("[FLOW] SpeedAchieved event sent to Bpod"));
    }

    // Continue monitoring in consecutive, non-overlapping windows. Every
    // window above the threshold produces a SpeedAchieved event.
    integrationStartTimeMs = currentTimeMs;
    lastSpeedIntegralUpdateTimeMs = currentTimeMs;
    integratedSpeedMmPerSecondMilliseconds = 0;
}

} // namespace

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println(F("EmadTreadmillSpeed firmware started"));
    Serial1.begin(kBpodBaudRate, SERIAL_8N1, kBpodRxPin, kBpodTxPin);
    setupTreadmillSpeedInput();
}

void loop() {
    processBpodSerial();
    const uint32_t currentTimeMs = sampleAndUpdateSpeedIntegral();
    checkSpeedTrigger(currentTimeMs);
}
