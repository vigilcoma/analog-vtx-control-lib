#include "EspIdfVtxControl.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr uart_port_t kVtxUart = UART_NUM_1;
constexpr int kRxBufferSize = 512;

uart_config_t uart_config_for(const EspIdfVtxMode mode) {
  uart_config_t config = {};
  config.baud_rate = mode == EspIdfVtxMode::SmartAudio ? 4800 : 9600;
  config.data_bits = UART_DATA_8_BITS;
  config.parity = UART_PARITY_DISABLE;
  config.stop_bits = mode == EspIdfVtxMode::SmartAudio ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
  config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if ESP_IDF_VERSION_MAJOR >= 5
  config.source_clk = UART_SCLK_DEFAULT;
#else
  config.source_clk = UART_SCLK_APB;
#endif
  return config;
}
}  // namespace

EspIdfVtxControl::EspIdfVtxControl(const EspIdfVtxMode mode, const int data_pin,
                                   const int, const bool invert, const int)
    : mode_(mode), data_pin_(data_pin), invert_(invert) {
  configure_rx();
}

EspIdfVtxControl::~EspIdfVtxControl() {
  end();
}

void EspIdfVtxControl::end() {
  if (uart_ready_) {
    uart_driver_delete(kVtxUart);
    uart_ready_ = false;
  }
  release_line();
}

void EspIdfVtxControl::release_line() {
  if (data_pin_ < 0) return;
  gpio_reset_pin(static_cast<gpio_num_t>(data_pin_));
  gpio_set_pull_mode(static_cast<gpio_num_t>(data_pin_), GPIO_PULLUP_ONLY);
  gpio_set_direction(static_cast<gpio_num_t>(data_pin_), GPIO_MODE_INPUT);
}

void EspIdfVtxControl::configure_rx() {
  if (uart_ready_) {
    ESP_ERROR_CHECK(uart_driver_delete(kVtxUart));
    uart_ready_ = false;
  }
  release_line();
  const uart_config_t config = uart_config_for(mode_);
  ESP_ERROR_CHECK(uart_driver_install(kVtxUart, kRxBufferSize, 0, 0, nullptr, 0));
  uart_ready_ = true;
  ESP_ERROR_CHECK(uart_param_config(kVtxUart, &config));
  ESP_ERROR_CHECK(uart_set_pin(kVtxUart, UART_PIN_NO_CHANGE, data_pin_,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void EspIdfVtxControl::flush() {
  if (uart_ready_) (void)uart_wait_tx_done(kVtxUart, pdMS_TO_TICKS(40));
}

uint8_t EspIdfVtxControl::smart_audio_crc(const uint8_t *bytes, const size_t size) const {
  uint8_t crc = 0;
  for (size_t i = 0; i < size; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                         : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

uint8_t EspIdfVtxControl::tramp_checksum(const uint8_t *frame) const {
  uint8_t checksum = 0;
  for (size_t i = 1; i < 14; ++i) checksum = static_cast<uint8_t>(checksum + frame[i]);
  return checksum;
}

bool EspIdfVtxControl::send(const uint8_t *bytes, const size_t size) {
  if (data_pin_ < 0 || size == 0) return false;
  if (uart_ready_) {
    ESP_ERROR_CHECK(uart_driver_delete(kVtxUart));
    uart_ready_ = false;
  }
  release_line();

  const uart_config_t config = uart_config_for(mode_);
  ESP_ERROR_CHECK(uart_driver_install(kVtxUart, kRxBufferSize, 0, 0, nullptr, 0));
  uart_ready_ = true;
  ESP_ERROR_CHECK(uart_param_config(kVtxUart, &config));
  ESP_ERROR_CHECK(uart_set_pin(kVtxUart, data_pin_, UART_PIN_NO_CHANGE,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(gpio_set_direction(static_cast<gpio_num_t>(data_pin_), GPIO_MODE_OUTPUT));

  const int queued = uart_write_bytes(kVtxUart, bytes, size);
  const esp_err_t tx_done = uart_wait_tx_done(kVtxUart, pdMS_TO_TICKS(40));
  configure_rx();
  esp_rom_delay_us(500);
  return queued == static_cast<int>(size) && tx_done == ESP_OK;
}

bool EspIdfVtxControl::setChannel(const int channel_index) {
  if (mode_ != EspIdfVtxMode::SmartAudio || channel_index < 0 || channel_index > 255) return false;
  uint8_t frame[] = {0x00, 0xAA, 0x55, 0x07, 0x01, static_cast<uint8_t>(channel_index), 0};
  frame[6] = smart_audio_crc(frame + 1, 5);
  return send(frame, sizeof(frame));
}

bool EspIdfVtxControl::setFrequency(const uint16_t frequency_mhz) {
  if (mode_ == EspIdfVtxMode::Tramp) {
    uint8_t frame[16] = {0x0F, 'F', static_cast<uint8_t>(frequency_mhz),
                         static_cast<uint8_t>(frequency_mhz >> 8)};
    frame[14] = tramp_checksum(frame);
    return send(frame, sizeof(frame));
  }
  uint8_t frame[] = {0x00, 0xAA, 0x55, 0x09, 0x02,
                     static_cast<uint8_t>(frequency_mhz >> 8),
                     static_cast<uint8_t>(frequency_mhz), 0};
  frame[7] = smart_audio_crc(frame + 1, 6);
  return send(frame, sizeof(frame));
}

bool EspIdfVtxControl::setSmartAudioPowerRaw(const uint8_t raw_power) {
  if (mode_ != EspIdfVtxMode::SmartAudio) return false;
  uint8_t frame[] = {0x00, 0xAA, 0x55, 0x05, 0x01, raw_power, 0};
  frame[6] = smart_audio_crc(frame + 1, 5);
  return send(frame, sizeof(frame));
}

bool EspIdfVtxControl::setPowerInmW(const uint16_t milliwatts) {
  if (mode_ != EspIdfVtxMode::Tramp) return false;
  uint8_t frame[16] = {0x0F, 'P', static_cast<uint8_t>(milliwatts),
                       static_cast<uint8_t>(milliwatts >> 8)};
  frame[14] = tramp_checksum(frame);
  return send(frame, sizeof(frame));
}

bool EspIdfVtxControl::updateParameters() {
  if (mode_ == EspIdfVtxMode::SmartAudio) {
    const uint8_t frame[] = {0x00, 0xAA, 0x55, 0x03, 0x00, 0x9F};
    return send(frame, sizeof(frame));
  }
  uint8_t frame[16] = {0x0F, 'r'};
  frame[14] = tramp_checksum(frame);
  return send(frame, sizeof(frame));
}

bool EspIdfVtxControl::sa_setProtocolVersion(const EspIdfSmartAudioVersion version) {
  smart_audio_version_ = version;
  return true;
}
