# Bpod Rotary Encoder v1 - breadboard Nano ESP32

This is a breadboard-oriented port of the official Sanworks Rotary Encoder
Module v1 firmware to an **Arduino Nano ESP32**. It is kept in a separate
PlatformIO environment so it cannot be linked into either existing controller.

## Build and upload

```sh
/opt/homebrew/bin/pio run -e rotaryEncoderNanoESP32
/opt/homebrew/bin/pio run -e rotaryEncoderNanoESP32 -t upload
```

## Pin map

| Nano ESP32 pin | Purpose | Connect to |
|---|---|---|
| D0 / RX | Bpod UART receive | UART TX of the full-duplex RS-485 converter |
| D1 / TX | Bpod UART transmit | UART RX of the full-duplex RS-485 converter |
| D2 | Encoder channel A | Encoder A/open-collector output |
| D3 | Encoder channel B | Encoder B/open-collector output |
| D4 | Encoder index Z | Reserved; currently not used by the v1 protocol |
| D5 / RX2 | Output-stream receive | Optional second module UART TX |
| D6 / TX2 | Output-stream transmit | Optional second module UART RX |
| GND | Common reference | Encoder and converter ground |

The Nano ESP32 is a **3.3 V device**. Do not drive D0-D6 with 5 V logic.
The firmware enables the Nano's 3.3 V pull-ups on the encoder inputs; external
pull-ups to 3.3 V are preferable for long/noisy encoder wiring.

The Bpod module port is RS-485, not TTL UART. Keep the same full-duplex
RS-485-to-UART converter between Bpod and the Nano that is used for the Due.
The serial link is configured for the Bpod module rate of 1,312,500 baud.

## Compatibility and intentional differences

Preserved from Rotary Encoder Module v1:

- Bpod module discovery (opcode 255, module name `RotaryEncoder`)
- X1 quadrature position counting and bipolar/unipolar wrapping
- eight position thresholds and threshold event bytes 1-8
- USB control, position/time streaming, and event markers
- optional second-UART position stream

Not present on the breadboard build:

- microSD logging (`L` and `F` are no-ops; `R` returns zero samples)
- the Teensy 3.5 circuit-revision sensing pins
- v2-only advanced time thresholds and DAC output

The interrupt writes encoder changes into a 256-sample ring buffer. The main
loop performs serial writes and threshold checks, keeping those slower actions
out of the encoder interrupt. If the loop cannot drain that buffer, new samples
are dropped rather than corrupting memory.
