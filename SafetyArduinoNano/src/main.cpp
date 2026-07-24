//
// Created by Emad Muzaffar on 7/10/26.
//

#include <Arduino.h>

// Cross-check pin setup
constexpr uint8_t kEnablePin = D9;
constexpr uint8_t kHostTransmitPin = D5;
constexpr uint8_t kHostReceivePin = D4;
constexpr uint8_t kClientTransmitPin = D3;
constexpr uint8_t kClientReceivePin = D2;

// Timing setup
constexpr uint32_t kHostStartTolerance = 1000;       // milliseconds before safety check starts
constexpr uint32_t kCrossCheckMaxNoResponseTime = 5000; // microseconds before e-stop
constexpr uint32_t kCrossCheckDutyTime = 500;        // microseconds between host toggles

static_assert(kCrossCheckMaxNoResponseTime > 2 * kCrossCheckDutyTime,
    "The cross-check timeout must allow at least two heartbeat transitions");

// State tracking
uint32_t hostTransmitTime = 0;
uint32_t lastHostEchoTime = 0;
uint8_t lastHostEchoState = LOW;

//eStop bool
bool eStopBool = false;

void eStop() {
    if (eStopBool) {
        return;
    }

    digitalWrite(kEnablePin, LOW);
    eStopBool = true;
    // Stop both communication directions. The Due will see its heartbeat
    // disappear and latch its own emergency stop.
    digitalWrite(kHostTransmitPin, LOW);
    digitalWrite(kClientTransmitPin, LOW);
    Serial.println("eStop: Due heartbeat timed out");
}

void hostTransmit() {
    const uint32_t nowMicros = micros();

    if (nowMicros - hostTransmitTime > kCrossCheckDutyTime) {
        const uint8_t currentState = digitalRead(kHostTransmitPin);

        if (currentState == HIGH) {
            digitalWrite(kHostTransmitPin, LOW);
        } else {
            digitalWrite(kHostTransmitPin, HIGH);
        }

        hostTransmitTime = nowMicros;
    }
}

void hostUpdate() {
    const uint32_t nowMicros = micros();

    // Give the other Arduino time to boot before checking
    if (millis() <= kHostStartTolerance) {
        lastHostEchoState = digitalRead(kHostReceivePin);
        lastHostEchoTime = nowMicros;
        return;
    }

    // Host toggles independently
    hostTransmit();

    // Check if the echoed signal is still changing
    const uint8_t echoState = digitalRead(kHostReceivePin);

    if (echoState != lastHostEchoState) {
        lastHostEchoState = echoState;
        lastHostEchoTime = nowMicros;
        digitalWrite(kEnablePin, HIGH);
        return;
    }

    // If no echo change has happened recently, stop
    if (nowMicros - lastHostEchoTime > kCrossCheckMaxNoResponseTime) {
        eStop();
    }
}

void clientUpdate() {
    static uint8_t lastClientReceiveState = LOW;

    const uint8_t receiveState = digitalRead(kClientReceivePin);

    // Only write when the input changes
    if (receiveState != lastClientReceiveState) {
        lastClientReceiveState = receiveState;
        digitalWrite(kClientTransmitPin, receiveState);
    }
}

void crossCheckUpdate() {
    clientUpdate();
    hostUpdate();
}

void setup() {
    Serial.begin(115200);

    pinMode (kEnablePin, OUTPUT);
    digitalWrite(kEnablePin, LOW);

    pinMode(kHostTransmitPin, OUTPUT);
    pinMode(kHostReceivePin, INPUT_PULLUP);

    pinMode(kClientTransmitPin, OUTPUT);
    pinMode(kClientReceivePin, INPUT_PULLUP);

    digitalWrite(kHostTransmitPin, LOW);
    digitalWrite(kClientTransmitPin, LOW);

    lastHostEchoState = digitalRead(kHostReceivePin);
    lastHostEchoTime = micros();
}

void loop() {
    if (eStopBool) {
        // Keep every fail-safe output low until this controller is reset.
        digitalWrite(kEnablePin, LOW);
        digitalWrite(kHostTransmitPin, LOW);
        digitalWrite(kClientTransmitPin, LOW);
        return;
    }
    crossCheckUpdate();
}
