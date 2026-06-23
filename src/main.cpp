// //
// // Created by Emad Muzaffar on 6/18/26.
// //
//
// #include <Arduino.h>
// #include <ArCom/ArCOM.h>
// #include <ArCom/ArCOM.cpp>
//
// constexpr uint8_t kDirectionPin = 2;
// constexpr uint8_t kPwmPin = 6;
// constexpr uint8_t kEnablePin = 3;
// constexpr uint8_t kStatusLedPin = 13;
// constexpr float kMaxMotorRPM = 30.0f; // Must match the ClearPath MSP setup.
// constexpr uint16_t kMaxPwm = 65535;
//
// class Motor {
//     //Internal variables for motor to store data.
//     uint8_t directionPin;
//     uint8_t pwmPin;
//     uint8_t enablePin;
//     unsigned long startTime = 0;
//     uint32_t durationMs = 0;
//     bool running = false;
//
//     void setTimer(const uint8_t timerLength) {
//         startTime = millis();
//         durationMs = timerLength;
//         running = true;
//     }
//
//     boolean updateTimer() const {
//         return running && (millis() - startTime >= durationMs);
//     }
//
//     void setPWM(const uint8_t velocityDegPerSec) const {
//         float targetRPM = static_cast<float>(velocityDegPerSec) / 6.0f;
//         targetRPM = constrain(targetRPM, 0.0f, kMaxMotorRPM);
//         uint16_t duty = static_cast<uint16_t>((targetRPM / kMaxMotorRPM) * kMaxPwm);
//         analogWrite(this->pwmPin, duty);
//     }
//
//     void setDirection(int direction) const {
//         if (direction == 0) {
//             digitalWrite(directionPin, HIGH);
//         } else if (direction == 1) {
//             digitalWrite(directionPin, LOW);
//         }
//     }
//
//     void pwmStop(boolean stop = false) const {
//         if (stop) {
//             analogWrite(this->pwmPin, 0);
//         }
//     }
//
// public:
//     explicit Motor(uint8_t directionPin, uint8_t pwmPin, uint8_t enablePin) {
//         this->directionPin = directionPin;
//         this->pwmPin = pwmPin;
//         this->enablePin = enablePin;
//         pinMode(directionPin, OUTPUT);
//         pinMode(pwmPin, OUTPUT);
//         pinMode(enablePin, OUTPUT);
//     }
//
//     void enable() const {
//         digitalWrite(this->enablePin, HIGH);
//     }
//
//     void disable() const {
//         digitalWrite(this->enablePin, LOW);
//     }
//
//     void update() {
//         if (updateTimer()) {
//             pwmStop(true);
//             running = false;
//         }
//         //add safety update here
//     }
//
//     void update(uint8_t speed, uint8_t time, uint8_t direction) {
//         setDirection(direction);
//         setPWM(speed);
//         setTimer(time);
//     }
//
// };
//
//
// /**
//  * Initialize objects and module setup
//  * @author Emad Muzaffar
//  */
// unsigned long FirmwareVersion = 1;
// char moduleName[] = "ServoMotor"; // ModuleName sent to Bpod in returnModuleInfo()
// ArCOM Serial1COM(Serial1); // NOLINT(*-interfaces-global-init)
// Motor motor(kDirectionPin, kPwmPin, kEnablePin);
// byte opCode = 0; //Code to specify instruction type
//
// /**
//  * From SanWorks example
//  * Writes info about arduino system over serial to Bpod
//  * Called in switch(opCode) when opCode == 255, when Bpod scans for modules
//  * @author JSNeuroDev
//  */
// void returnModuleInfo() {
//     Serial1COM.writeByte(65); // Acknowledge
//     Serial1COM.writeUint32(FirmwareVersion); // 4-byte firmware version
//     Serial1COM.writeByte(sizeof(moduleName)-1); // Length of module name
//     Serial1COM.writeCharArray(moduleName, sizeof(moduleName)-1); // Module name
//     Serial1COM.writeByte(0); // 1 if more info follows, 0 if not
// }
//
//
// void setup() {
//     Serial1.begin(1312500); //specific baud rate for Bpod
//     analogWriteResolution(16);
//     pinMode(kStatusLedPin, OUTPUT);
//     digitalWrite(kStatusLedPin, LOW);
// }
//
// /**
//  * Main loop
//  * @author Emad Muzaffar
// */
// void loop() {
//     if (Serial1COM.available()) {
//         opCode = Serial1COM.readByte();
//         if (opCode == 255) {
//             returnModuleInfo();
//         } else if (opCode == 9) {
//             //Turn enable off
//             digitalWrite(kStatusLedPin, LOW);
//             motor.disable();
//         } else if (opCode == 10) {
//             //Turn enable on
//             digitalWrite(kStatusLedPin, HIGH);
//             motor.enable();
//         } else if (opCode == 1) {
//             uint8_t buffer[3];
//             while (Serial1COM.available() < 3) {
//                 // wait for full array
//             }
//             Serial1COM.readByteArray(buffer, 3);
//             uint8_t speed  = buffer[0];
//             uint8_t time = buffer[1];
//             uint8_t direction = buffer[2];
//             motor.update(speed, time, direction);
//         }
//     }
//     motor.update();
// }
