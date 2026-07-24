# Bpod Treadmill Interface speed trigger - breadboard Nano ESP32

This firmware turns a breadboard-mounted **Arduino Nano ESP32** into a Bpod
module that reports when the analog speed output from the
[Janelia Treadmill Interface](https://github.com/janelia-experimental-technology/Treadmill-Interface)
exceeds a configured threshold. The Nano no longer decodes the encoder's A/B
quadrature signals. It is kept in a separate PlatformIO environment so it
cannot be linked into either existing controller.

## Build and upload

This environment uses the ESP32-S3 ROM downloader. Before every upload:

1. Connect **B1 to GND**.
2. Press and release **RST** once.
3. Disconnect **B1 from GND**.
4. Confirm that `pio device list` shows the Espressif bootloader. On the
   current Mac it is `/dev/cu.usbmodem11401`, with USB VID:PID `303A:1001`.
5. Run the upload command below.

```sh
/opt/homebrew/bin/pio run -e rotaryEncoderNanoESP32
/opt/homebrew/bin/pio run -e rotaryEncoderNanoESP32 -t upload
```

Do not upload while the port is `/dev/cu.usbmodem206EF1324A442` with VID:PID
`2341:0070`; that is the running Arduino application, not the ROM downloader.
The configured `no_reset` option intentionally preserves an already-open ROM
download port because a second automatic reset makes macOS disconnect it.

## Pin map

| Nano ESP32 pin | Purpose | Connect to |
|---|---|---|
| D0 / RX | Bpod UART receive | UART TX of the full-duplex RS-485 converter |
| D1 / TX | Bpod UART transmit | UART RX of the full-duplex RS-485 converter |
| D2 | Analog speed input | Treadmill Interface speed-output SMA center pin |
| GND | Common reference | Treadmill Interface SMA shield and converter ground |

The Nano ESP32 is a **3.3 V device**. Do not drive D0-D2 above 3.3 V. The
Treadmill Interface firmware limits the speed output to 2.5 V, so it can be
connected directly to D2. The grounds must be connected for the analog reading
to be valid. D3 is no longer used.

This firmware expects the older absolute-speed Treadmill Interface output:

| Speed | Analog output |
|---|---|
| 0 mm/s | 0 V |
| +1000 mm/s | 2.5 V |

The direction output is not connected or used.

The Bpod module port is RS-485, not TTL UART. Keep the same full-duplex
RS-485-to-UART converter between Bpod and the Nano that is used for the Due.
The serial link is configured for the Bpod module rate of 1,312,500 baud.

## Bpod command

Send one 5-byte message. Multi-byte values are unsigned and little-endian.

| Byte(s) | Type | Meaning |
|---|---|---|
| 1 | `uint8` | Opcode `1` |
| 2-3 | `uint16` | Required treadmill speed in mm/s |
| 4-5 | `uint16` | Integration window in milliseconds |

The Nano samples the analog speed on every pass through `loop()`. For each
integration window, the firmware calculates:

```text
integral of speed over the window
--------------------------------- = average speed in mm/s
       elapsed time (ms)
```

The windows are consecutive and do not overlap. When the average speed is
strictly greater than the requested speed, the module sends behavior event byte
`1` (`SpeedAchieved`) to Bpod. The trigger remains active, so every integration
window above the threshold sends another event. Send another opcode `1` packet
only when changing the speed or integration time. An integration time of `0`
is invalid and stops speed measurements until a valid configuration is sent.

Example MATLAB packet construction:

```matlab
requiredSpeedMmPerSecond = uint16(100);
integrationTimeMs = uint16(100);

speedBytes = typecast(requiredSpeedMmPerSecond, 'uint8');
timeBytes = typecast(integrationTimeMs, 'uint8');

command = [
    uint8(1), ...
    speedBytes, ...
    timeBytes
];
```

Opcode `255` returns module name `EmadRotaryEncoder` and advertises the single
behavior event `SpeedAchieved` during Bpod module discovery.
