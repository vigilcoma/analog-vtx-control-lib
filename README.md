# Analog VTX Control Library

Reusable ESP32 Arduino/PlatformIO controller for analog VTXs that use **SmartAudio** or **Tramp**.

It is extracted from the proven `vtx-control` firmware without changing the transport implementation: SmartAudio’s dummy-byte write, explicit TX/RX half-duplex direction changes, and `flush()` behavior are retained.

## Install with PlatformIO

```ini
lib_deps =
  https://github.com/vigilcoma/analog-vtx-control-lib.git
```

The library currently targets ESP32 Arduino projects. It uses one half-duplex UART bus that shares one ESP32 GPIO with the VTX control line.

## Quick start

```cpp
#include <Arduino.h>

HardwareSerial VTXSER(1);  // Required: define once in your application.
#include <VTXControl.h>

constexpr int VTX_LINE_PIN = 5;
VTXControl* vtx = nullptr;

void setup() {
  Serial.begin(115200);

  // SmartAudio: 4800 baud, 8N2. Use VTXMode::Tramp for Tramp VTXs.
  vtx = new VTXControl(VTXMode::SmartAudio, VTX_LINE_PIN);

  // Query VTX first so its protocol version and current values are known.
  if (vtx->updateParameters()) {
    vtx->setFrequency(5800);  // MHz; use your VTX's own frequency table.
    vtx->flush();             // Wait until the command has left the UART.
  }
}

void loop() {}
```

For a Tramp VTX, use `VTXMode::Tramp`. Before setting a Tramp power level by index, pass the VTX-specific control values with `setTrampPowerTable()`; `setPowerInmW()` is generally clearer.

## API

- `setFrequency(uint16_t mhz)`
- `setChannel(int standard_5_8ghz_index)`
- `setPower(int level)`
- `setPowerInmW(uint16_t milliwatts)`
- `setSmartAudioPowerRaw(uint8_t raw_value)` for VTX-specific SmartAudio tables
- `updateParameters()`
- `flush()`

The built-in `setChannel()` map is the standard 5.8 GHz channel map. For 1.2 GHz, 3.3 GHz, or non-standard VTX bands, use `setFrequency()` with the selected VTX table.

## Wiring

- Connect the selected ESP32 GPIO to the VTX SmartAudio or Tramp control wire.
- Connect VTX and ESP32 ground.
- SmartAudio VTX control must be compatible with 3.3 V logic.
- The library deliberately releases the line in RX mode after each transmit command so the VTX can reply.

See [the example](examples/esp32_half_duplex/esp32_half_duplex.ino) for a complete minimal sketch.

## Proven source

This package contains the protocol/controller core from [vigilcoma/vtx-control](https://github.com/vigilcoma/vtx-control). The application UI, Wi-Fi portal, CRSF integration, VTX-table parser, and firmware binaries are intentionally not included.
