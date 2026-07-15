//
// Created by Emad Muzaffar on 7/13/26.
//

#include <Arduino.h>

constexpr uint8_t kDirectionPin = 7;
constexpr uint8_t kPwmPin = 6;
constexpr uint8_t kEnablePin = 53;

void setup() {
    Serial.begin(115200);
    pinMode(kPwmPin, OUTPUT);
    pinMode(kDirectionPin, OUTPUT);
    pinMode(kEnablePin, OUTPUT);
    analogWrite(kPwmPin, 200);
    digitalWrite(kEnablePin, HIGH);
    digitalWrite(kDirectionPin, LOW);
}

void loop() {
    while (Serial.available() > 0) {
        Serial.write(Serial.read());
    }
}