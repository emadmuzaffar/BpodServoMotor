/*
  Breadboard Arduino Nano ESP32 port of the Sanworks Bpod Rotary Encoder
  Module v1 firmware.

  The original firmware is Copyright (C) 2022 Sanworks LLC and licensed
  under GPL-3.0. This derived port is distributed under the same license.
  Original source:
  https://github.com/sanworks/Bpod_RotaryEncoder_Firmware
*/

#include <Arduino.h>

namespace {

constexpr uint32_t kFirmwareVersion = 6;
constexpr uint8_t kHardwareVersion = 1;
constexpr uint8_t kHardwareRevision = 0;
constexpr char kModuleName[] = "RotaryEncoder";
constexpr uint32_t kBpodBaud = 1312500;

// Nano ESP32 breadboard pin assignment. Use Arduino Dx labels, not raw GPIOs.
constexpr uint8_t kStateMachineRxPin = D0;
constexpr uint8_t kStateMachineTxPin = D1;
constexpr uint8_t kEncoderPinA = D2;
constexpr uint8_t kEncoderPinB = D3;
constexpr uint8_t kEncoderPinZ = D4;  // Reserved for the encoder index signal.
constexpr uint8_t kOutputStreamRxPin = D5;
constexpr uint8_t kOutputStreamTxPin = D6;

constexpr uint8_t kMaxThresholds = 8;
constexpr uint16_t kSampleBufferSize = 256;
constexpr uint16_t kSampleBufferMask = kSampleBufferSize - 1;

static_assert((kSampleBufferSize & kSampleBufferMask) == 0,
              "Sample buffer size must be a power of two");

enum class CommandSource : uint8_t {
  usb,
  stateMachine,
  outputStream,
};

struct EncoderSample {
  int16_t position;
  int32_t wraps;
  uint32_t timestamp;
};

volatile int16_t encoderPosition = 0;
volatile int32_t encoderWraps = 0;
volatile bool lastEncoderA = false;
volatile uint8_t currentDirection = 0;
volatile bool wrappingEnabled = true;
volatile uint8_t wrapMode = 0;
volatile int16_t wrapPoint = 512;

volatile EncoderSample sampleBuffer[kSampleBufferSize]{};
volatile uint16_t sampleHead = 0;
volatile uint16_t sampleTail = 0;
volatile uint32_t droppedSamples = 0;

int16_t thresholds[kMaxThresholds]{};
bool thresholdActive[kMaxThresholds] = {
    true, true, true, true, true, true, true, true};
uint8_t thresholdCount = kMaxThresholds;

bool usbStreaming = false;
bool sendEvents = true;
bool moduleStreaming = false;
char moduleStreamPrefix = 'M';
uint32_t streamStartMicros = 0;

Stream& streamFor(const CommandSource source) {
  switch (source) {
    case CommandSource::usb:
      return Serial;
    case CommandSource::stateMachine:
      return Serial1;
    case CommandSource::outputStream:
      return Serial2;
  }
  return Serial;
}

uint8_t readByte(Stream& stream) {
  while (stream.available() == 0) {
    yield();
  }
  return static_cast<uint8_t>(stream.read());
}

uint16_t readUint16(Stream& stream) {
  const uint16_t low = readByte(stream);
  return low | (static_cast<uint16_t>(readByte(stream)) << 8U);
}

int16_t readInt16(Stream& stream) {
  return static_cast<int16_t>(readUint16(stream));
}

void writeUint16(Stream& stream, const uint16_t value) {
  stream.write(static_cast<uint8_t>(value));
  stream.write(static_cast<uint8_t>(value >> 8U));
}

void writeInt16(Stream& stream, const int16_t value) {
  writeUint16(stream, static_cast<uint16_t>(value));
}

void writeUint32(Stream& stream, const uint32_t value) {
  stream.write(static_cast<uint8_t>(value));
  stream.write(static_cast<uint8_t>(value >> 8U));
  stream.write(static_cast<uint8_t>(value >> 16U));
  stream.write(static_cast<uint8_t>(value >> 24U));
}

void clearSamples() {
  noInterrupts();
  sampleTail = sampleHead;
  interrupts();
}

void setPosition(const int16_t position) {
  noInterrupts();
  encoderPosition = position;
  encoderWraps = 0;
  sampleTail = sampleHead;
  interrupts();
}

int16_t getPosition() {
  noInterrupts();
  const int16_t position = encoderPosition;
  interrupts();
  return position;
}

void resetThresholds() {
  for (uint8_t i = 0; i < thresholdCount; ++i) {
    thresholdActive[i] = true;
  }
}

void IRAM_ATTR recordEncoderSample() {
  int16_t position = encoderPosition;
  int32_t wraps = encoderWraps;

  if (wrappingEnabled) {
    if (wrapMode == 0) {
      if (position <= -wrapPoint) {
        position = wrapPoint;
        --wraps;
      } else if (position >= wrapPoint) {
        position = -wrapPoint;
        ++wraps;
      }
    } else {
      if (position > wrapPoint) {
        position = 0;
        ++wraps;
      } else if (position < 0) {
        position = wrapPoint;
        --wraps;
      }
    }
  }

  encoderPosition = position;
  encoderWraps = wraps;

  const uint16_t head = sampleHead;
  const uint16_t next = (head + 1U) & kSampleBufferMask;
  if (next == sampleTail) {
    ++droppedSamples;
    return;
  }

  sampleBuffer[head].position = position;
  sampleBuffer[head].wraps = wraps;
  sampleBuffer[head].timestamp = micros();
  sampleHead = next;
}

void IRAM_ATTR updatePosition() {
  // Preserve the v1 firmware's X1 quadrature decoding behavior.
  const bool encoderA = digitalRead(kEncoderPinA);
  const bool encoderB = digitalRead(kEncoderPinB);

  if (encoderA && !lastEncoderA) {
    if (encoderB) {
      if (currentDirection == 0) {
        ++encoderPosition;
        recordEncoderSample();
      }
      currentDirection = 0;
    } else {
      currentDirection = 1;
    }
  } else {
    if (encoderB) {
      if (currentDirection == 1) {
        --encoderPosition;
        recordEncoderSample();
      }
      currentDirection = 1;
    } else {
      currentDirection = 0;
    }
  }
  lastEncoderA = encoderA;
}

bool popSample(EncoderSample& sample) {
  noInterrupts();
  const uint16_t tail = sampleTail;
  if (tail == sampleHead) {
    interrupts();
    return false;
  }
  sample.position = sampleBuffer[tail].position;
  sample.wraps = sampleBuffer[tail].wraps;
  sample.timestamp = sampleBuffer[tail].timestamp;
  sampleTail = (tail + 1U) & kSampleBufferMask;
  interrupts();
  return true;
}

void writeUsbPosition(const EncoderSample& sample) {
  Serial.write('P');
  writeInt16(Serial, sample.position);
  writeUint32(Serial, sample.timestamp - streamStartMicros);
}

void checkThresholds(const EncoderSample& sample) {
  if (!sendEvents || sample.wraps != 0) {
    return;
  }

  for (uint8_t i = 0; i < thresholdCount; ++i) {
    if (!thresholdActive[i]) {
      continue;
    }

    const bool crossed = thresholds[i] < 0
                             ? sample.position <= thresholds[i]
                             : sample.position >= thresholds[i];
    if (crossed) {
      thresholdActive[i] = false;
      Serial1.write(i + 1U);
    }
  }
}

void processEncoderSamples() {
  EncoderSample sample{};
  while (popSample(sample)) {
    if (usbStreaming) {
      writeUsbPosition(sample);
    }
    if (moduleStreaming) {
      Serial2.write(moduleStreamPrefix);
      const uint16_t streamedPosition = wrapMode == 0
                                            ? sample.position + wrapPoint
                                            : sample.position;
      writeUint16(Serial2, streamedPosition);
    }
    checkThresholds(sample);
  }
}

void returnModuleInfo() {
  bool stateMachineSupportsHardwareInfo = false;
  delayMicroseconds(100);
  if (Serial1.available() == 1 && readByte(Serial1) == 255) {
    stateMachineSupportsHardwareInfo = true;
  }

  Serial1.write('A');
  writeUint32(Serial1, kFirmwareVersion);
  Serial1.write(sizeof(kModuleName) - 1U);
  Serial1.write(reinterpret_cast<const uint8_t*>(kModuleName),
                sizeof(kModuleName) - 1U);

  if (stateMachineSupportsHardwareInfo) {
    Serial1.write(1);
    Serial1.write('V');
    Serial1.write(kHardwareVersion);
    Serial1.write(1);
    Serial1.write('v');
    Serial1.write(kHardwareRevision);
  }
  Serial1.write(0);
  Serial1.flush();
}

void emitLoggedEvent(const uint8_t eventCode) {
  if (!usbStreaming) {
    return;
  }
  Serial.write('E');
  Serial.write(0);  // State-machine event source.
  Serial.write(eventCode);
  writeUint32(Serial, micros() - streamStartMicros);
}

void handleCommand(const CommandSource source, const uint8_t opCode) {
  Stream& input = streamFor(source);

  switch (opCode) {
    case 255:
      if (source == CommandSource::stateMachine) {
        returnModuleInfo();
      }
      break;

    case 254:
      if (source == CommandSource::usb) {
        Serial2.write(254);
      } else if (source == CommandSource::outputStream) {
        Serial.write(254);
      }
      break;

    case 'C':
      if (source == CommandSource::usb) {
        Serial.write(217);
      }
      break;

    case 'S':
      if (source == CommandSource::usb) {
        usbStreaming = readByte(input) != 0;
        if (usbStreaming) {
          setPosition(0);
          streamStartMicros = micros();
        }
      }
      break;

    case 'O':
      moduleStreaming = readByte(input) != 0;
      if (source == CommandSource::usb) {
        Serial.write(1);
      }
      break;

    case 'V':
      if (source == CommandSource::usb) {
        sendEvents = readByte(input) != 0;
        Serial.write(1);
      }
      break;

    case '#':
      if (source == CommandSource::stateMachine) {
        emitLoggedEvent(readByte(input));
      }
      break;

    case 'W':
      if (source == CommandSource::usb) {
        const int16_t newWrapPoint = readInt16(input);
        noInterrupts();
        wrapPoint = newWrapPoint;
        wrappingEnabled = newWrapPoint != 0;
        encoderWraps = 0;
        interrupts();
        Serial.write(1);
      }
      break;

    case 'M':
      if (source == CommandSource::usb) {
        wrapMode = readByte(input);
        Serial.write(1);
      }
      break;

    case 'T':
      if (source == CommandSource::usb) {
        const uint8_t count = readByte(input);
        if (count <= kMaxThresholds) {
          thresholdCount = count;
          for (uint8_t i = 0; i < thresholdCount; ++i) {
            thresholds[i] = readInt16(input);
            thresholdActive[i] = true;
          }
          Serial.write(1);
        } else {
          Serial.write(0);
        }
      }
      break;

    case 'I':
      if (source == CommandSource::usb) {
        moduleStreamPrefix = static_cast<char>(readByte(input));
        Serial.write(1);
      }
      break;

    case ';': {
      const uint8_t enabledMask = readByte(input);
      for (uint8_t i = 0; i < thresholdCount; ++i) {
        thresholdActive[i] = bitRead(enabledMask, i);
      }
      noInterrupts();
      encoderWraps = 0;
      interrupts();
      break;
    }

    case 'Z':
      setPosition(0);
      if (usbStreaming) {
        EncoderSample zeroSample{0, 0, micros()};
        writeUsbPosition(zeroSample);
      }
      break;

    case 'E':
      resetThresholds();
      noInterrupts();
      encoderWraps = 0;
      interrupts();
      break;

    case 'R':
      // The breadboard port has no microSD. Report zero logged positions so
      // the official client completes the request instead of waiting.
      if (source == CommandSource::usb) {
        writeUint32(Serial, 0);
      }
      break;

    case 'Q':
      if (source == CommandSource::usb) {
        writeInt16(Serial, getPosition());
      }
      break;

    case 'P':
      if (source == CommandSource::usb) {
        setPosition(readInt16(input));
        Serial.write(1);
      }
      break;

    case 'X':
      usbStreaming = false;
      moduleStreaming = false;
      setPosition(0);
      streamStartMicros = micros();
      break;

    // 'L' and 'F' are accepted as no-ops because this breadboard build has
    // no microSD storage. Advanced v2-only commands are intentionally absent.
    case 'L':
    case 'F':
    default:
      break;
  }
}

void processCommands() {
  if (Serial.available() > 0) {
    handleCommand(CommandSource::usb, readByte(Serial));
  } else if (Serial1.available() > 0) {
    handleCommand(CommandSource::stateMachine, readByte(Serial1));
  } else if (Serial2.available() > 0) {
    handleCommand(CommandSource::outputStream, readByte(Serial2));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial1.begin(kBpodBaud, SERIAL_8N1, kStateMachineRxPin,
                kStateMachineTxPin);
  Serial2.begin(kBpodBaud, SERIAL_8N1, kOutputStreamRxPin,
                kOutputStreamTxPin);

  // The v1 module board supplied input conditioning. On a breadboard, the
  // Nano's 3.3 V pull-ups provide defined levels for open-collector encoders.
  pinMode(kEncoderPinA, INPUT_PULLUP);
  pinMode(kEncoderPinB, INPUT_PULLUP);
  pinMode(kEncoderPinZ, INPUT_PULLUP);
  lastEncoderA = digitalRead(kEncoderPinA);
  attachInterrupt(digitalPinToInterrupt(kEncoderPinA), updatePosition, CHANGE);

  streamStartMicros = micros();
}

void loop() {
  processCommands();
  processEncoderSamples();
}
