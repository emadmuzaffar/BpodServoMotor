//
// Created by Emad Muzaffar on 7/10/26.
//

#include <Arduino.h>

// Cross-check pin setup
constexpr uint8_t kHostTransmitPin = 52;
constexpr uint8_t kHostReceivePin = 50;
constexpr uint8_t kClientTransmitPin = 48;
constexpr uint8_t kClientReceivePin = 46;

// Timing setup
constexpr uint32_t kHostStartTolerance = 300;        // milliseconds before safety check starts
constexpr uint32_t kCrossCheckMaxNoResponseTime = 5000; // microseconds before e-stop
constexpr uint32_t kCrossCheckDutyTime = 500;        // microseconds between host toggles

// State tracking
uint32_t hostTransmitTime = 0;
uint32_t lastHostEchoTime = 0;
uint8_t lastHostEchoState = LOW;

// Replace this with your real e-stop function
void eStop() {

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

    pinMode(kHostTransmitPin, OUTPUT);
    pinMode(kHostReceivePin, INPUT);

    pinMode(kClientTransmitPin, OUTPUT);
    pinMode(kClientReceivePin, INPUT);

    digitalWrite(kHostTransmitPin, LOW);
    digitalWrite(kClientTransmitPin, LOW);

    lastHostEchoState = digitalRead(kHostReceivePin);
    lastHostEchoTime = micros();
}

void loop() {
    crossCheckUpdate();
}