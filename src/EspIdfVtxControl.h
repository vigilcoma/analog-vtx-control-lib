#pragma once

#include <cstddef>
#include <cstdint>

enum class EspIdfVtxMode : uint8_t {
  SmartAudio = 1,
  Tramp = 2,
};

enum class EspIdfSmartAudioVersion : uint8_t {
  V1 = 0,
  V2 = 1,
  V21 = 2,
};

// Native ESP-IDF backend for the same one-wire VTX protocol used by VTXControl.
// UART1 is recreated TX-only for each frame, flushed, then recreated RX-only,
// so the shared VTX DATA pin is always released for a response.
class EspIdfVtxControl {
public:
  EspIdfVtxControl(EspIdfVtxMode mode, int data_pin, int response_timeout_ms = 100,
                   bool invert = false, int num_tries = 1);
  ~EspIdfVtxControl();

  void flush();
  void end();
  bool setChannel(int channel_index);
  bool setFrequency(uint16_t frequency_mhz);
  bool setSmartAudioPowerRaw(uint8_t raw_power);
  bool setPowerInmW(uint16_t milliwatts);
  bool updateParameters();

  bool sa_setProtocolVersion(EspIdfSmartAudioVersion version);
  EspIdfSmartAudioVersion getSmartAudioProtocolVersion() const { return smart_audio_version_; }

private:
  EspIdfVtxMode mode_;
  int data_pin_;
  bool invert_;
  bool uart_ready_ = false;
  EspIdfSmartAudioVersion smart_audio_version_ = EspIdfSmartAudioVersion::V1;

  void configure_rx();
  void release_line();
  bool send(const uint8_t *bytes, size_t size);
  uint8_t smart_audio_crc(const uint8_t *bytes, size_t size) const;
  uint8_t tramp_checksum(const uint8_t *frame) const;
};
