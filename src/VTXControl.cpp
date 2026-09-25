#if defined(ARDUINO)
#include <Arduino.h>
// #include <util/delay.h>
#include "HardwareSerialAdapter.h"
#include "VTXControl.h"
#include "VTX_SmartAudio.h"
#include "VTX_Tramp.h"

// these parameters for EACHINE TX5258
const uint16_t powers[4] = {25, 200, 500, 800}; // in mW
// these parameters for JHEMCU RuiBet Tran3016W
// uint16_t powers[5] = { 25, 200, 400, 800, 1600 };//in mW
const uint16_t powers_v1[4] = {7 /*25mw*/, 16 /*200mw*/, 25 /*500mw*/,
                               40 /*800mw*/}; // for SmartAudio protocol v1
// Smartaudio v2.1 protocol seems to be not documented (TBS documented just v1
// and v2), but defined in ArduPilot
const uint16_t powers_v21[4] = {
    14 /*25mw*/, 20 /*200mw*/, 26 /*500mw*/,
    30 /*800mw*/}; // for SmartAudio protocol v2.1 in dbm
const uint16_t freqs[40] = {
    5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, // A band
    5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, // B band
    5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, // E band
    5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, // F band
    5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, // r Band
};

// Instantiates the VTX object with the specified parameters
VTXControl::VTXControl(int vtxMode, int softPin, int responseTimeOut /*= 1000*/,
                       bool invert /*= true*/, int numtries /*= 3*/) {
  DEBUG("VTXControl: Create");
  vtx_mode = vtxMode;
  // port = new SoftwareSerialWithHalfDuplex(softPin, softPin, false, false);
  port = new HardwareSerialAdapter(softPin, softPin, invert);
  _responseTimeOut = responseTimeOut;
  _numtries = numtries;
  _smartBaudRate = false; //smartBaudRate;

  // with SmartAudio protocol according to TBS documentation
  //!!! Please do remember, for SmartAudio - logic level = 3.3V !!!
  // we use 4800bps 1 Start bit and 2 Stop bit, 8 data bits
  // for SmartAudio clones(wthiout TBS license) - 1 start bit, 2 stop bit, 8
  // data bits with Tramp - 9600bps, 1 start bit, 1 stop bit, 8 data bits

  // port->begin(vtx_mode == VTXMode::SmartAudio ? AP_SMARTAUDIO_UART_BAUD :
  // AP_TRAMP_UART_BAUD,
  //   vtx_mode == VTXMode::SmartAudio ? AP_SMARTAUDIO_UART_CFG :
  //   AP_TRAMP_UART_CFG);*/
  //
  port->begin(vtx_mode == VTXMode::SmartAudio ? AP_SMARTAUDIO_UART_BAUD
                                              : AP_TRAMP_UART_BAUD,
              vtx_mode == VTXMode::SmartAudio
                  ? SERIAL_8N2
                  : SERIAL_8N1);

  // ESP32: on high speeds, we should turn off interrupts
  // port->enableIntTx(false);

  Serial.println("new VTXControl()");

  Serial.print("speed: ");
  Serial.println(port->getSpeed());

  Serial.print("config: ");
  Serial.println(port->getConfiguration());

  waitForInMs(200);
}
void VTXControl::flush() { port->flush(); }
void VTXControl::clearErrors() {
  errors = VTXErrors::vtxNoErrors;
  port->clearErrors();
}
long VTXControl::getSpeed() {
  DEBUG("VTXControl: getSpeed:" + (String)port->getSpeed());
  return port->getSpeed();
}
int VTXControl::getErrors() {
  sswhdErrors err = port->getErrors();
  if (err != sswhdErrors::sswhdNoErrors) {
    /*
    if (err & sswhdErrors::sswhdIsNotListening != 0)
      errors |= vtxportIsNotListening;
    if (err & sswhdErrors::sswhdBufferIsEmpty != 0)
      errors |= vtxportBufferIsEmpty;
    if (err & sswhdErrors::sswhdTxDelayIsZero != 0)
      errors |= vtxportTxDelayIsZero;
    if (err & sswhdErrors::sswhdRXDelayStopBitNotSet != 0)
      errors |= vtxportRXDelayStopBitNotSet;
    */

    if (err & sswhdErrors::sswhdIsNotListening)
      errors |= vtxportIsNotListening;
    if (err & sswhdErrors::sswhdBufferIsEmpty)
      errors |= vtxportBufferIsEmpty;
    if (err & sswhdErrors::sswhdTxDelayIsZero)
      errors |= vtxportTxDelayIsZero;
    if (err & sswhdErrors::sswhdRXDelayStopBitNotSet)
      errors |= vtxportRXDelayStopBitNotSet;
  }
  return errors;
}
void VTXControl::setError(VTXErrors error) { errors |= error; }
bool VTXControl::sa_updateSettings() {
  long startspeed = port->getSpeed();
  long newspeed = startspeed;
  DEBUG("Smart Audio Update_settings");
  while (1) // speed cycle
  {
    for (int i = 0; i < _numtries; i++) {
      if (sa_getSettings()) {
        if (sa_readResponse())
          return true; // in case of success we return true
      }
      port->flush();
    }
    if (_smartBaudRate) {
      // so we need to change baud rate?
      newspeed = sa_offerNewSpeed(newspeed);
      if (newspeed != startspeed) {
        DEBUG("update_settings, trying new speed:" + (String)newspeed);
        port->begin(newspeed, port->getConfiguration());
      } else {
        DEBUG("update_settings, end of tries");
        return false; // out of change speed cycle
      }
    } else
      break;
  }
  return false;
}
long VTXControl::sa_offerNewSpeed(long currentSpeed) {
  // if speed == AP_SMARTAUDIO_UART_BAUD we're going to
  // AP_SMARTAUDIO_SMARTBAUD_MAX by increasing step by step to
  // AP_SMARTAUDIO_SMARTBAUD_MAX if we achieved AP_SMARTAUDIO_SMARTBAUD_MAX
  // we're going to AP_SMARTAUDIO_SMARTBAUD_MIN and then increasing to
  // AP_SMARTAUDIO_UART_BAUD so
  // AP_SMARTAUDIO_UART_BAUD->AP_SMARTAUDIO_SMARTBAUD_MAX->AP_SMARTAUDIO_SMARTBAUD_MIN->AP_SMARTAUDIO_UART_BAUD(terminator)
  switch (currentSpeed) {
  case AP_SMARTAUDIO_UART_BAUD:
    return AP_SMARTAUDIO_UART_BAUD + AP_SMARTAUDIO_SMARTBAUD_STEP;
  case AP_SMARTAUDIO_SMARTBAUD_MIN:
    return AP_SMARTAUDIO_SMARTBAUD_MIN + AP_SMARTAUDIO_SMARTBAUD_STEP;
  case AP_SMARTAUDIO_SMARTBAUD_MAX:
    return AP_SMARTAUDIO_SMARTBAUD_MIN;
  }
  // if speed is in AP_SMARTAUDIO_UART_BAUD-AP_SMARTAUDIO_SMARTBAUD_MAX or
  // AP_SMARTAUDIO_SMARTBAUD_MIN-AP_SMARTAUDIO_UART_BAUD
  //  ranges - we just increase baud rate step-by-step
  if ((currentSpeed > AP_SMARTAUDIO_UART_BAUD &&
       currentSpeed < AP_SMARTAUDIO_SMARTBAUD_MAX) ||
      (currentSpeed > AP_SMARTAUDIO_SMARTBAUD_MIN &&
       currentSpeed < AP_SMARTAUDIO_UART_BAUD))
    return currentSpeed + AP_SMARTAUDIO_SMARTBAUD_STEP;
  if (currentSpeed > AP_SMARTAUDIO_SMARTBAUD_MAX)
    return AP_SMARTAUDIO_SMARTBAUD_MAX;
  if (currentSpeed < AP_SMARTAUDIO_SMARTBAUD_MIN)
    return AP_SMARTAUDIO_SMARTBAUD_MIN;
  return AP_SMARTAUDIO_UART_BAUD;
}
#if VTXCDEBUG
bool VTXControl::testSMAWrite() { return sa_getSettings(); }
bool VTXControl::testSMAResponseFromSerial1() {
  SettingsResponseFrame response;
  response.header.init(SMARTAUDIO_RSP_GET_SETTINGS_V1, 5);
  response.channel = 2;
  response.power = 25;           // v1
  response.operationMode = 0x04; // pitmode turned on
  response.frequency = 5800;
  DEBUG("push response (Serial1):" + (String)sizeof(SettingsResponseFrame));
  bool res =
      Serial1.write((uint8_t *)&response, sizeof(SettingsResponseFrame)) ==
      sizeof(SettingsResponseFrame);
  // Packet command;
  //// according to the spec the length should include the CRC, but no
  /// implementation appears to / do this
  // command.frame.header.init(SMARTAUDIO_CMD_GET_SETTINGS, 0);
  // command.frame_size = SMARTAUDIO_COMMAND_FRAME_SIZE;
  // command.frame.payload[0] = crc8_dvb_s2_update(0, &command.frame,
  // SMARTAUDIO_COMMAND_FRAME_SIZE - 1); DEBUG("push to write:" +
  // (String)sizeof(Packet)); bool res = Serial1.write((uint8_t*)&command,
  // sizeof(Packet)) == sizeof(Packet);

  return res;
}
#endif
// bool VTXControl::sa_setPitMode(int enabled)
//{
//   //activate - IN RANGE PIT FLAG
//   //deactivate - (Quit PIT MODE
//   uint8_t mode = enabled ? 0x01 : 0x04;
//   if (push_uint8_command_frame(SMARTAUDIO_CMD_SET_MODE, mode))
//   {
//     //DEBUG("End sma_setPitMode");
//     return readResponse(); //updateParameters();
//   }
//   return false;
// }
bool VTXControl::sa_setPower(int pwrLevel) {
  DEBUG("sa_setPower call level:" + (String)pwrLevel);

  static uint8_t buf[6] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_POWER, 1, 0x00, 0x00};
  switch (sa_protocol_version) {
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v1:
    // res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER,
    // powers_v1[pwrLevel]);
    buf[4] = powers_v1[pwrLevel];
    break;
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v2:
    // debug("Setting power to %d", power_level);
    // res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER, pwrLevel);
    buf[4] = pwrLevel;
    break;
  case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v21:
    // res = push_uint8_command_frame(SMARTAUDIO_CMD_SET_POWER,
    // powers_v21[pwrLevel] | 0x80);
    buf[4] = powers_v21[pwrLevel] | 0x80;
    break;
  }
  buf[5] = sa_CRC8(buf, 5);

  DEBUG("sa_setPower:" + (String)buf[4] + ", push to write:" + (String)sizeof(buf));
  
  dumpHex(buf, sizeof(buf));
  
  // according to SA documentation:
  // The SmartAudio line need to be low before a frame is sent.
  // If the host MCU can�t handle this it can be done by
  // sending a 0x00 dummy byte in front of the actual frame.
  port->enableTx(true);
  port->writeDummyByte();
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
#if SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END
  port->write((uint8_t)0x00);
#endif
  //port->listen();

  return res;
}

void VTXControl::dumpHex(const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n; ++i)
    Serial.printf("%02X%s", b[i], (i + 1 < n) ? " " : "\n");
}

bool VTXControl::sa_setChannel(uint8_t channel) {
  static uint8_t buf[6] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_CHANNEL, 1, 0, 0};
  buf[4] = channel;
  buf[5] = sa_CRC8(buf, 5); // exclude crc byte
  DEBUG("sa_setChannel, channel:" + (String)channel);
  DEBUG("sa_setChannel, push to write:" + (String)sizeof(buf));
  dumpHex(buf, sizeof(buf));
  // according to SA documentation:
  // The SmartAudio line need to be low before a frame is sent.
  // If the host MCU can�t handle this it can be done by
  // sending a 0x00 dummy byte in front of the actual frame.
  port->enableTx(true);
  port->writeDummyByte();
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
#if SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END
  port->write((uint8_t)0x00);
#endif
  //port->listen();

  return res;
}

bool VTXControl::sa_sendRaw(const String& hex) {
  uint8_t buf[64]; size_t n=0;
  int i=0;
  while (i<(int)hex.length() && n<sizeof(buf)) {
    while (i<(int)hex.length() && isspace((int)hex[i])) i++;
    if (i>= (int)hex.length()) break;
    int v = strtol(hex.substring(i).c_str(), NULL, 16);
    buf[n++] = (uint8_t)(v & 0xFF);
    // advance to next token (skip current hex token)
    while (i<(int)hex.length() && !isspace((int)hex[i])) i++;
  }

  DEBUG("sa_sendRaw, push to write:" + (String)sizeof(buf));
  // according to SA documentation:
  // The SmartAudio line need to be low before a frame is sent.
  // If the host MCU can�t handle this it can be done by
  // sending a 0x00 dummy byte in front of the actual frame.
  port->writeDummyByte();
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
#if SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END
  port->write((uint8_t)0x00);
#endif
  //port->listen();

  return res;
}

bool VTXControl::sa_getSettings() {
  // taken from betaflight vtx_smartaudio.c
  static uint8_t buf[5] = {0xAA, 0x55, SMARTAUDIO_CMD_GET_SETTINGS, 0x00, 0x9F};
  DEBUG("sa_GetSettings, push to write:" + (String)sizeof(buf));
  // according to SA documentation:
  // The SmartAudio line need to be low before a frame is sent.
  // If the host MCU can�t handle this it can be done by
  // sending a 0x00 dummy byte in front of the actual frame.

  dumpBuffer(buf, 5);

  port->enableTx(true);
  port->writeDummyByte();
  // port->write((uint8_t)0x00);
  bool res = port->write((uint8_t *)&buf, sizeof(buf)) == sizeof(buf);
  // port->writeDummyByte();
#if SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END
  port->write((uint8_t)0x00);
#endif  
  return res;
}

// relatively precise function as replacement of delay
// delay stop processor/interrupts usage, so we need to use delayMcroseconds
// but delayMicroseconds uses max 16384 value
void VTXControl::waitForInMs(unsigned int ms) {
  int in_tens_ms = ms / 10;
  int reminder_ms = ms - (in_tens_ms * 10);
  // tens of ms
  for (int i = 0; i < in_tens_ms; i++) {
    delayMicroseconds(10000);
  }
  // reminded ms
  for (int i = 0; i < reminder_ms; i++) {
    delayMicroseconds(1000);
  }
  // DEBUG("VTXControl::waitForInMs:" + (String)ms + "ms End");
}

bool VTXControl::sa_readResponse() {
  _ignoreSaCrc = false;

  port->flush();
  port->enableTx(false);
  delayMicroseconds(500);

  auto waitAvail = [&](uint32_t until){
    while (millis() < until) { if (port->available() > 0) return true; delay(2); }
    return false;
  };

  uint8_t buf[AP_SMARTAUDIO_MAX_PACKET_SIZE]; uint8_t i = 0;
  uint32_t headerDeadline = millis() + 500;

  // find 0xAA (skip 0x00)
  while (millis() < headerDeadline) {
    if (!waitAvail(headerDeadline)) break;
    int v = port->read();
    Serial.print("header first bit: ");
    Serial.println((uint8_t)v, HEX);
    if (v < 0) continue;
    uint8_t b = (uint8_t)v; if (b == 0x00) continue;
    if (b == 0xAA) { buf[i++] = b; break; }
  }
  if (!i) {
    return false;
  }
  

  // read 3 more header bytes
  while (i < 4) {
    if (!waitAvail(headerDeadline)) return false;
    int v = port->read();
    Serial.print("header rest bit: ");
    Serial.println((uint8_t)v, HEX);
    if (v < 0) continue;
    buf[i++] = (uint8_t)v;
  }

  auto* hdr = (FrameHeader*)buf;
  Serial.printf("SA hdr: cmd=0x%02X len=%u\n", hdr->command, hdr->length);

  uint8_t need = hdr->length;                 // payload+CRC on most SA


  uint8_t expected = hdr->length;
  // Read until we have 'expected' bytes OR the line goes idle for ~8 ms.
  // (8 ms > ~3 chars at 4800 8N2 => "end of frame" heuristic)
  uint32_t idleStart = micros();
  while ((i - sizeof(FrameHeader)) < expected && (micros() - idleStart) < 8000 && i < sizeof(buf)) {
    int v = port->read();
    if (v >= 0) { buf[i++] = (uint8_t)v; idleStart = micros(); }
    else { delayMicroseconds(200); }
  }

  uint8_t gotPayload = i - sizeof(FrameHeader);
  if (gotPayload < expected) {
    // Tolerate short frames from SA clones (Rush Solo/Max, etc.)
    Serial.printf("SA short frame [ignoreSRC]: expected=%u, got=%u; accepting partial.\n", expected, gotPayload);
    // Clamp the length so your parser doesn't read past what you have:
    ((FrameHeader*)buf)->length = gotPayload;
    // Also skip CRC check for these partial frames:
    _ignoreSaCrc = true;   // add a flag (see next section)
  } else {
    _ignoreSaCrc = true;
    Serial.printf("SA normal frame [ignoreSRC]: expected=%u, got=%u; accepting partial.\n", expected, gotPayload);
  }

  // uint32_t payloadDeadline = millis() + 2000;  // fresh time budget
  // while (need) {
  //   int avail = port->available();
  //   if (avail <= 0) {
  //     if (!waitAvail(payloadDeadline)) { 
  //       Serial.printf("need=%u\n", need);
  //       setError(VTXErrors::vtxBufferLengthLessWholePacket);
  //       return false;
  //     }
  //     continue;
  //   }
  //   int take = min<int>(avail, need);
  //   while (take--) {
  //     int v=port->read();
  //     if (v>=0) {
  //       buf[i++]=(uint8_t)v;
  //       Serial.print("bit: ");
  //       Serial.println((uint8_t)v, HEX);
  //       --need;
  //     }
  //   }
  // }

  Serial.printf("Read total bytes: %u (header+payload)\n", i);

  dumpBuffer(buf, AP_SMARTAUDIO_MAX_PACKET_SIZE);

  // 4) Parse & CRC
  bool ok = sa_parseResponseBuffer(buf);

  // Give VTX time to recover before next command
  waitForInMs(100);
  
  DEBUG("readResponse end, ok = " + (String)ok);
  
  return ok;
}

bool VTXControl::sa_parseResponseBuffer(const uint8_t *buffer) {
  DEBUG("sa_parseResponseBuffer - dump:");
  dumpBuffer(buffer, AP_SMARTAUDIO_UART_BUFSIZE_RX);

  const FrameHeader *header = (const FrameHeader *)buffer;
  const uint8_t fullFrameLength = sizeof(FrameHeader) + header->length;
  const uint8_t headerPayloadLength = fullFrameLength - 1;              // subtract crc byte from length
  const uint8_t *startPtr = buffer + 2; // exclude header and sync bytes
  const uint8_t *endPtr = buffer + headerPayloadLength;
  DEBUG("sa_parse_response_buffer(), fullFrameLength=" +
        (String)fullFrameLength);
  if (!sa_ignoreCrc()) {
#if VTXCDEBUG
    dumpBuffer(buffer, headerPayloadLength);
#endif
    uint8_t crc = sa_CRC8(buffer, headerPayloadLength);
    // uint8_t crc = sa_CRC8(startPtr, headerPayloadLength-2);
    uint8_t crc_resp = *(endPtr);
    if (crc != crc_resp) {
      DEBUG("sa_parse_response_buffer() failed - invalid CRC or header, crc in "
            "resp=" +
            (String)crc_resp + ", crc calc=" + (String)crc);
      setError(VTXErrors::vtxParseResponseInvalidCRCOrBuffer);
      return false;
    }
  }
  if (header->headerByte != SMARTAUDIO_HEADER_BYTE) {
    setError(VTXErrors::vtxIncomingByteNotEqualHeaderByte);
    return false;
  }
  if (header->syncByte != SMARTAUDIO_SYNC_BYTE) {
    setError(VTXErrors::vtxIncomingByteNotEqualSyncByte);
    return false;
  }
  switch (header->command) {
  case SMARTAUDIO_RSP_GET_SETTINGS_V1: {
    DEBUG("sa_parse_response_buffer(), Protocol version 1");
    sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v1;
    const SettingsResponseFrame *resp = (const SettingsResponseFrame *)buffer;
    pwr_Level = getPowerIndexFromV1(resp->power);
    ch_index = resp->channel;
    pitMode = (resp->operationMode & 0x04) != 0;
  } break;
  case SMARTAUDIO_RSP_GET_SETTINGS_V2: {
    DEBUG("sa_parse_response_buffer(), Protocol version 2");
    sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v2;
    const SettingsResponseFrame *respv2 = (const SettingsResponseFrame *)buffer;
    pwr_Level = respv2->power;
    ch_index = respv2->channel;
    pitMode = (respv2->operationMode & 0x04) != 0;
  } break;

  case SMARTAUDIO_RSP_GET_SETTINGS_V21: {
    DEBUG("sa_parse_response_buffer(), Protocol version 2.1");
    sa_protocol_version = SMARTAUDIO_SPEC_PROTOCOL_v21;
    const SettingsExtendedResponseFrame *respv21 =
        (const SettingsExtendedResponseFrame *)buffer;
    pwr_Level = getPowerIndexFromDbm(respv21->power_dbm);
    ch_index = respv21->settings.channel;
    pitMode = (respv21->settings.operationMode & 0x04) != 0;
  } break;
  case SMARTAUDIO_RSP_SET_POWER: {

    const U16ResponseFrame *respu16 = (const U16ResponseFrame *)buffer;
    const uint8_t power = respu16->payload & 0xFF;
    switch (sa_protocol_version) {
    case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v21:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 2.1");
      pwr_Level = getPowerIndexFromDbm(power);
      break;
    case ProtocolVersion::SMARTAUDIO_SPEC_PROTOCOL_v1:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 1");
      pwr_Level = getPowerIndexFromV1(power);
      break;
    default:
      DEBUG("sa_parse_response_buffer(), SetPower:Protocol version 2");
      pwr_Level = power;
      break;
    }
  } break;
  case SMARTAUDIO_RSP_SET_CHANNEL: {
    DEBUG("sa_parse_response_buffer(), SetChannel");
    const U8ResponseFrame *respu8 = (const U8ResponseFrame *)buffer;
    uint16_t channel = respu8->payload;
    ch_index = channel;
    // debug("Channel was set to %d", resp->payload);
  } break;
  }
  return true;
}

// bool VTXControl::setPitMode(bool enabled)
//{
//   clearErrors();
//   switch (vtx_mode)
//   {
//   case VTXMode::SmartAudio:
//     //DEBUG("setPitMode: SMA");
//     return sa_setPitMode(enabled);
//   case VTXMode::Tramp:
//     //DEBUG("setPitMode: Tramp");
//     return trampSetPitMode(enabled);
//   }
//   return false;
// }

bool VTXControl::setSmartAudioPowerRaw(uint8_t rawPower) {
  if (vtx_mode != VTXMode::SmartAudio) return false;

  clearErrors();
  uint8_t buf[6] = {0xAA, 0x55, SMARTAUDIO_CMD_SET_POWER, 1, rawPower, 0};
  buf[5] = sa_CRC8(buf, 5);

  // Preserve the proven SmartAudio send sequence: drive the line low with the
  // dummy byte, transmit the command, then let sa_readResponse() flush and
  // release the one-wire bus back to RX-only mode.
  port->enableTx(true);
  port->writeDummyByte();
  const bool sent = port->write(buf, sizeof(buf)) == sizeof(buf);
#if SMARTAUDIO_WRITE_ZEROBYTES_AT_THE_END
  port->write((uint8_t)0x00);
#endif
  return sent && sa_readResponse();
}

bool VTXControl::setPowerInmW(uint16_t pwrmW) {
  bool res = false;
  clearErrors();
  for (int i = 0; i < _numtries; i++) // trying to set _numtries times
  {
    switch (vtx_mode) {
    case VTXMode::SmartAudio: {
      int pwrLevel = getPowerIndexFromMW(pwrmW);
      if (pwrLevel != -1) // pwr level(index) not found
      {
        if (sa_setPower(pwrLevel)) {
          if (sa_readResponse()) {
            res = true;
          }
        }
      }
    } break;
    case VTXMode::Tramp: // tramp protocol sets power in mW
    {
      // tramp protocol needs to be initialized
      if (!initialized) {
        trampInit();
      }
      // in milliWatts
      if (initialized) {
        if (trampMaxPowerMw != 0 && pwrmW > trampMaxPowerMw) {
          DEBUG("setPowerInmW: Requested power exceeds VTX maximum: " +
                (String)pwrmW + " > " + (String)trampMaxPowerMw);
          return false;
        }
        res = trampSetPower(pwrmW);
      }
    } break;
    }
    // if procedure of setting is succesfull - check acvieved result
    // if (res) {
    //   int currPwrmW = getPowerInmW(pwr_Level);
    //   if (currPwrmW == pwrmW)
    //     return true;
    // }

    if (res) {
      return true;
    }
  }
  return false;
}

void VTXControl::setTrampPowerTable(const int* values, size_t count) {
  trampPowerValues.clear();
  trampPowerValues.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    if (values[i] >= 0 && values[i] <= UINT16_MAX) {
      trampPowerValues.push_back((uint16_t)values[i]);
    }
  }
}

bool VTXControl::setPower(int pwrLevel) {
  if (pwrLevel >= 0 && pwrLevel < sizeof(powers)) // check the index of power
  {
    bool res = false;
    clearErrors();
    for (int i = 0; i < _numtries; i++) // trying to set _numtries times
    {
      switch (vtx_mode) {
      case VTXMode::SmartAudio: {
        if (sa_setPower(pwrLevel)) {
          if (sa_readResponse()) {
            res = true;
          }
        }
      } break;
      case VTXMode::Tramp: // tramp protocol sets power in mW
      {
        // tramp protocol needs to be initialized
        if (!initialized) {
          trampInit();
        }
        // in milliWatts
        if (initialized)
          res = trampSetPower(powers[pwrLevel]);
      } break;
      }
      // if procedure of setting is succesfull - check acvieved result
      if (res) { // && pwr_Level == pwrLevel) {
        return true;
      }
    }
  }
  return false;
}
bool VTXControl::setPrevChannel() {
  int newCh = ch_index - 1;
  if (newCh < 0)
    newCh = sizeof(freqs) / sizeof(freqs[0]) - 1;
  return setChannel(newCh);
}
bool VTXControl::setNextChannel() {
  int newCh = ch_index + 1;
  if (newCh >= sizeof(freqs) / sizeof(freqs[0]))
    newCh = 0;
  return setChannel(newCh);
}
uint16_t VTXControl::getPowerInmW(int pwrIndex) {
  int pwrslen = sizeof(powers) / sizeof(powers[0]);
  return (pwrIndex >= 0 && pwrIndex < pwrslen) ? powers[pwrIndex] : 0;
}
int VTXControl::getPowerIndexFromMW(uint16_t pwrInmW) {
  if (vtx_mode == VTXMode::Tramp && !trampPowerValues.empty()) {
    for (size_t i = 0; i < trampPowerValues.size(); ++i) {
      if (trampPowerValues[i] == pwrInmW) {
        return (int)i;
      }
    }
    return -1;
  }

  int pwrslen = sizeof(powers) / sizeof(powers[0]);
  for (int i = 0; i < pwrslen; i++) {
    if (powers[i] == pwrInmW)
      return i;
  }
  return -1; // not found
}
int VTXControl::getPowerIndexFromDbm(uint16_t pwrInDbm) {
  int pwrslen = sizeof(powers_v21) / sizeof(powers_v21[0]);
  for (int i = 0; i < pwrslen; i++) {
    if (powers_v21[i] == pwrInDbm)
      return i;
  }
  return -1; // not found
}
int VTXControl::getPowerIndexFromV1(uint16_t pwrValue) {
  int pwrslen = sizeof(powers_v1) / sizeof(powers_v1[0]);
  for (int i = 0; i < pwrslen; i++) {
    if (powers_v1[i] == pwrValue)
      return i;
  }
  return -1; // not found
}
uint16_t VTXControl::getChannelFrequency(int chIndex) {
  int freqslen = sizeof(freqs) / sizeof(freqs[0]);
  return (chIndex >= 0 && chIndex < freqslen) ? freqs[chIndex] : 0;
}
int VTXControl::getChannelIndex(uint16_t freq) {
  int freqslen = sizeof(freqs) / sizeof(freqs[0]);
  for (int i = 0; i < freqslen; i++) {
    if (freqs[i] == freq)
      return i;
  }
  return -1; // not found
}

bool VTXControl::sa_setProtocolVersion(ProtocolVersion version) {
  Serial.println("sa_setProtocolVersion internal");
  sa_protocol_version = version;
  return true;
}

bool VTXControl::setFrequency(uint16_t freq) {
  clearErrors();
  //port->beginTX();
  bool res = false;
  for (int i = 0; i < _numtries; i++) // trying to set _numtries times
  {
    switch (vtx_mode) {
    case VTXMode::SmartAudio: {
      // debug("Setting channel to %d", channel);
      // if (push_uint8_command_frame(SMARTAUDIO_CMD_SET_CHANNEL, chIndex))
      int chIndex = getChannelIndex(freq);
      DEBUG("VTXControl: setFrequency");
      DEBUG(chIndex);
      if (chIndex != -1) // if frequency found in the table of freqs
      {
        if (sa_setChannel(chIndex)) {
          res = sa_readResponse();
        }
      }
    } break;
    case VTXMode::Tramp: // tramp protocol sets channel in Mhz
    {
      DEBUG("VTXControl: setFrequency");
      DEBUG(freq);
      // tramp protocol needs to be initialized
      if (!initialized) {
        trampInit();
      }
      if (initialized)
        res = trampSetFrequency(freq);
    } break;
    }
    // if channel set succesfully - check the updated info from VTX
    // TODO: this code doesn't actually query VTX for updated values
    //       and therefore always forces _numtries retries
    // if (res) {
    //   uint16_t curr_freq = getChannelFrequency(ch_index);
    //   if (curr_freq == freq)
    //     return true;
    // }

    if (res) {
      return true;
    }
  }
  return false;
}
// Sets the specified frequency to the VTX
bool VTXControl::setChannel(int chIndex) {
  clearErrors();
  bool res = false;
  for (int i = 0; i < _numtries; i++) // trying to set _numtries times
  {
    switch (vtx_mode) {
      case VTXMode::SmartAudio: {
        if (sa_setChannel(chIndex)) {
          res = sa_readResponse();
          DEBUG("sa_readResponse: " + res);
        }
      } break;
      case VTXMode::Tramp: // tramp protocol sets channel in Mhz
      {
        // tramp protocol needs to be initialized
        if (!initialized) {
          trampInit();
        }
        if (initialized)
          res = trampSetFrequency(freqs[chIndex]);
      } break;
    }
    // if channel set succesfully - check the updated info from VTX
    if (res) { // && chIndex == ch_index) {
      return true;
    }
  }

  return false;
}

bool VTXControl::updateParameters() {
  // port->stopListening();
  switch (vtx_mode) {
  case VTXMode::SmartAudio:
    // DEBUG("updateParameters: SMA"); waitForInMs(200);
    return sa_updateSettings();
  case VTXMode::Tramp: // tramp protocol sets channel in Mhz
    // DEBUG("updateParameters: Tramp"); waitForInMs(200);
    return trampUpdate();
  }
  return false;
}

bool VTXControl::trampSendPacket(uint8_t *packet,
                                 bool respRequired) //(trampFrame_t* packet)
{
  // DEBUG("trampSendPacket, before trampPush");
  if (trampPush(packet)) {
    // with setPower and setChannel we don't wait a response,
    // in this case we send UpdateParameters(GetConfig) to get new parameters
    if (respRequired) {
      bool res = trampReadResponse();
      DEBUG("trampSendPacket, After ReadResponse, response: " + (String)(res ? "SUCCESS" : "FAILURE"));
      return res;
    } else {
      DEBUG("trampSendPacket, no response required.");
      return true;
    }
  }
  DEBUG("trampSendPacket, return false");
  return false;
}

void VTXControl::trampWaitForPacketSlot() {
  if (trampLastPacketUs == 0) {
    return;
  }

  while ((uint32_t)(micros() - trampLastPacketUs) < TRAMP_MIN_REQUEST_PERIOD_US) {
    delay(1);
  }
}

bool VTXControl::trampPush(const uint8_t *packet) // const trampFrame_t* object)
{
  // port->writeDummyByte();//to get port low before sending command
  // port->write((uint8_t)0x00);
  // bool res = port->write((uint8_t*)&object, sizeof(trampFrame_t)) ==
  // sizeof(trampFrame_t);

  // IRC Tramp devices require at least 200 ms between requests. This also
  // covers the interval from initialization to the first setting command.
  trampWaitForPacketSlot();

  // ESP32
  port->drainRx();
  port->enableTx(true);

  bool res = port->write(packet, TRAMP_FRAME_LENGTH) == TRAMP_FRAME_LENGTH;
  if (res) {
    trampLastPacketUs = micros();
  }

  //port->listen(); // wait for the response
  // DEBUG("trampPush, written "+(String)sizeof(trampFrame_t) +" bytes");
  DEBUG("trampPush, written " + (String)TRAMP_FRAME_LENGTH + " bytes");
  DEBUG("trampPush, write() returned: " + (String)res);
#if VTXCDEBUG
  // dumpBuffer((uint8_t*)&object, sizeof(trampFrame_t));
  DEBUG("<- TX");
  dumpBuffer(packet, TRAMP_FRAME_LENGTH);
#endif
  return res;
}

bool VTXControl::trampUpdate() {
  long startspeed = port->getSpeed();
  long newspeed = startspeed;
  DEBUG("TrampUpdate");
  while (1) // speed cycle
  {
    // trying several times
    for (int i = 0; i < _numtries; i++) {
      // tramp protocol needs to be initialized
      if (!initialized) {
        trampInit();
        DEBUG("TrampUpdate, Initialized:" + (String)initialized);
      }
      if (initialized) {
        if (trampGetStatus()) {
          return true;
        }
      }
    }
    if (_smartBaudRate) {
      // so we need to change baud rate?
      newspeed = trampOfferNewSpeed(newspeed);
      if (newspeed != startspeed) {
        DEBUG("TrampUpdate, trying new speed:" + (String)newspeed);
        port->begin(newspeed, port->getConfiguration());
      } else {
        DEBUG("TrampUpdate, end of tries");
        return false; // out of change speed cycle
      }
    } else
      break;
  }
  return false;
}
long VTXControl::trampOfferNewSpeed(long currentSpeed) {
  // if speed == AP_SMARTAUDIO_UART_BAUD we're going to
  // AP_SMARTAUDIO_SMARTBAUD_MAX by increasing step by step to
  // AP_SMARTAUDIO_SMARTBAUD_MAX if we achieved AP_SMARTAUDIO_SMARTBAUD_MAX
  // we're going to AP_SMARTAUDIO_SMARTBAUD_MIN and then increasing to
  // AP_SMARTAUDIO_UART_BAUD so
  // AP_SMARTAUDIO_UART_BAUD->AP_SMARTAUDIO_SMARTBAUD_MAX->AP_SMARTAUDIO_SMARTBAUD_MIN->AP_SMARTAUDIO_UART_BAUD(terminator)
  switch (currentSpeed) {
  case AP_TRAMP_UART_BAUD:
    return AP_TRAMP_UART_BAUD + AP_TRAMP_SMARTBAUD_STEP;
  case AP_TRAMP_UART_BAUD_MIN:
    return AP_TRAMP_UART_BAUD_MIN + AP_TRAMP_SMARTBAUD_STEP;
  case AP_TRAMP_UART_BAUD_MAX:
    return AP_TRAMP_UART_BAUD_MIN;
  }
  // if speed is in AP_SMARTAUDIO_UART_BAUD-AP_SMARTAUDIO_SMARTBAUD_MAX or
  // AP_SMARTAUDIO_SMARTBAUD_MIN-AP_SMARTAUDIO_UART_BAUD
  //  ranges - we just increase baud rate step-by-step
  if ((currentSpeed > AP_TRAMP_UART_BAUD &&
       currentSpeed < AP_TRAMP_UART_BAUD_MAX) ||
      (currentSpeed > AP_TRAMP_UART_BAUD_MIN &&
       currentSpeed < AP_TRAMP_UART_BAUD))
    return currentSpeed + AP_TRAMP_SMARTBAUD_STEP;
  if (currentSpeed > AP_TRAMP_UART_BAUD_MAX)
    return AP_TRAMP_UART_BAUD_MAX;
  if (currentSpeed < AP_TRAMP_UART_BAUD_MIN)
    return AP_TRAMP_UART_BAUD_MIN;
  return AP_TRAMP_UART_BAUD;
}
bool VTXControl::trampSendCmd(uint8_t cmd) {
  /*uint8_t buf[TRAMP_FRAME_LENGTH];
  memset(buf, 0, TRAMP_FRAME_LENGTH);
  buf[0] = TRAMP_SYNC_START;
  buf[1] = cmd;
  buf[14] = trampCrc(buf);
  buf[15] = TRAMP_SYNC_STOP;
  bool res = trampSendPacket(buf);*/
  return trampSendCmd(cmd, 0);
}
bool VTXControl::trampSendCmd(uint8_t cmd, uint16_t param) {
  uint8_t buf[TRAMP_FRAME_LENGTH];
  memset(buf, 0, TRAMP_FRAME_LENGTH);
  buf[0] = TRAMP_SYNC_START;
  buf[1] = cmd;
  if (param != 0) {
    buf[2] = param & 0xff;
    buf[3] = (param >> 8) & 0xff;
  }
  buf[14] = trampCrc(buf);
  buf[15] = TRAMP_SYNC_STOP;
  // we need a reponse for certain commands only
  // so calling UpdateParameters required after send command
  bool respRequired = cmd == TRAMP_COMMAND_GET_CONFIG ||
                      cmd == TRAMP_COMMAND_CMD_RF ||
                      cmd == TRAMP_COMMAND_CMD_SENSOR;
  return trampSendPacket(buf, respRequired);
}
bool VTXControl::trampInit() {
  DEBUG("TrampInit:");
  /* trampFrame_t frame;
   trampFrameInit(TRAMP_COMMAND_CMD_RF, &frame);
   trampFrameClose(&frame);
   bool res = trampSendPacket((uint8_t*)&frame);*/

  // different version of sending
  if (trampSendCmd(TRAMP_COMMAND_CMD_RF)) {
    DEBUG("TrampInit: Inititalized Successfully");
    initialized = true;
    return true;
  }
  DEBUG("TrampInit: Not Inititalized");
  setError(VTXErrors::vtxtrampNotInited);
  return false;
}
bool VTXControl::trampGetStatus() {
  // trampFrame_t frame;
  // trampFrameInit(TRAMP_COMMAND_GET_CONFIG, &frame);
  // trampFrameClose(&frame);
  // return trampSendPacket((uint8_t*)&frame);
  // different version of sending
  bool res = trampSendCmd(TRAMP_COMMAND_GET_CONFIG);
  DEBUG("trampGetStatus, res=" + (String)res);
  return res;
}

bool VTXControl::trampSetFrequency(uint16_t freq) {
  // trampFrame_t frame;
  // trampFrameInit(TRAMP_COMMAND_SET_FREQ, &frame);
  // frame.payload.frequency = freq;
  // trampFrameClose(&frame);
  // return trampSendPacket((uint8_t*)&frame);
  return trampSendCmd(TRAMP_COMMAND_SET_FREQ, freq);
}
bool VTXControl::trampSetPower(uint16_t milliWatts) {
  // trampFrame_t frame;
  // trampFrameInit(TRAMP_COMMAND_SET_POWER, &frame);
  // frame.payload.power = milliWatts;
  // trampFrameClose(&frame);
  // return trampSendPacket((uint8_t*)&frame);
  if (!trampSendCmd(TRAMP_COMMAND_SET_POWER, milliWatts)) {
    return false;
  }

  // Set commands have no response. Only report success if the subsequent
  // status query confirms the requested configured value.
  if (!trampGetStatus()) {
    return false;
  }

  DEBUG("trampSetPower: Requested value=" + (String)milliWatts);
  DEBUG("trampSetPower: Configured value=" + (String)trampConfiguredPowerMw);
  DEBUG("trampSetPower: Reported value=" + (String)trampActualPowerMw);
  return trampConfiguredPowerMw == milliWatts;
}

bool VTXControl::trampReadResponse() {
  // On my Unify Pro32 the SmartAudio response is sent exactly 100ms after the
  // request and the initial response is 40ms long so we should wait at least
  // 140ms before giving up
  // waitForInMs(_responseTimeOut);
  // unsigned long currentTime = millis();
  ////wait to receive full packet

  port->flush();
  // ESP32
  port->enableTx(false);
  delayMicroseconds(500);

  const uint32_t deadline = millis() + _responseTimeOut;  // e.g. 500 ms
  uint8_t tramp_rx_buf[TRAMP_FRAME_LENGTH] = {};
  size_t n = 0;

  // Collect a full frame with resync on 0x0F start
  while (millis() < deadline && n < TRAMP_FRAME_LENGTH) {
    if (port->available() <= 0) { delay(5); continue; }
    int b = port->read();
    if (b < 0) continue;

    if (n == 0) {
      if ((uint8_t)b != TRAMP_SYNC_START) continue; // wait for 0x0F
    }
    tramp_rx_buf[n++] = (uint8_t)b;
  }

#if VTXCDEBUG
  // dumpBuffer((uint8_t*)&object, sizeof(trampFrame_t));
  DEBUG("RX ->");
  dumpBuffer(tramp_rx_buf, TRAMP_FRAME_LENGTH);
#endif

  if (n != TRAMP_FRAME_LENGTH) {
    DEBUG("trampReadResponse timeout; got "); DEBUG((int)n);
    setError(VTXErrors::vtxBufferLengthLessWholePacket);
    return false;
  }

  // Basic frame checks
  if (tramp_rx_buf[0]  != TRAMP_SYNC_START) { setError(VTXErrors::vtxIncomingByteNotEqualSyncByte); return false; }
  if (tramp_rx_buf[15] != TRAMP_SYNC_STOP ) { setError(VTXErrors::vtxLastByteNotSyncStop); return false; }

  const uint8_t crc = trampCrc(tramp_rx_buf);
  if (crc != tramp_rx_buf[14]) {
    DEBUG("trampReadResponse: CRC mismatch");
    setError(VTXErrors::vtxParseResponseInvalidCRCOrBuffer);
    return false;
  }
  
  DEBUG("TRAMP_SYNC_START");
  DEBUG(tramp_rx_buf[0]);
  DEBUG("TRAMP_SYNC_STOP");
  DEBUG(tramp_rx_buf[15]);

  const trampFrame_t *frame = (const trampFrame_t *)tramp_rx_buf;

  DEBUG("trampReadResponse: Update Data");
  // TODO:here we need to update data dependently of response code (cmd)
  switch (frame->header.command) {
  case TRAMP_COMMAND_CMD_RF: //'r', Init
  {
    trampMinFrequencyMhz = (uint16_t)tramp_rx_buf[2] |
                           ((uint16_t)tramp_rx_buf[3] << 8);
    trampMaxFrequencyMhz = (uint16_t)tramp_rx_buf[4] |
                           ((uint16_t)tramp_rx_buf[5] << 8);
    trampMaxPowerMw = (uint16_t)tramp_rx_buf[6] |
                      ((uint16_t)tramp_rx_buf[7] << 8);
    DEBUG("trampReadResponse: Response from Init");
    DEBUG("trampReadResponse: Minimum frequency MHz=" + (String)trampMinFrequencyMhz);
    DEBUG("trampReadResponse: Maximum frequency MHz=" + (String)trampMaxFrequencyMhz);
    DEBUG("trampReadResponse: Maximum power mW=" + (String)trampMaxPowerMw);
    break;
  }
  case TRAMP_COMMAND_GET_CONFIG: //'v' //we update data for current settings
                                  // just sending TRAMP_COMMAND_GET_CONFIG and
                                  // reading request
  {
    DEBUG("trampReadResponse: Response from Get_CONFIG: Update data");
    const uint16_t freq = frame->payload.settings.frequency;
    // Check we're not reading the request (indicated by freq zero)
    if (freq != 0) {
      // update data
      ch_index = getChannelIndex(frame->payload.settings.frequency);
      pwr_Level = getPowerIndexFromMW(frame->payload.settings.power);
      trampConfiguredPowerMw = frame->payload.settings.power;
      trampActualPowerMw = (uint16_t)tramp_rx_buf[8] |
                           ((uint16_t)tramp_rx_buf[9] << 8);
      pitMode = (bool)frame->payload.settings.pitModeEnabled;
      DEBUG("trampReadResponse: Updated, Freq:" +
            (String)frame->payload.settings.frequency);
      DEBUG("trampReadResponse: Converted to Channel:" + (String)ch_index);
      DEBUG("trampReadResponse: Updated, Power in Mw:" +
            (String)frame->payload.settings.power);
      DEBUG("trampReadResponse: Converted to Power Level:" +
            (String)pwr_Level);
      DEBUG("trampReadResponse: Updated, PitMode:" +
            (String)frame->payload.settings.pitModeEnabled);
    }
    break;
  }
  case TRAMP_COMMAND_CMD_SENSOR: //'s' - temperature sensor, currently not
                                  // used
  {
    // currently we ignore this response, just parse
    DEBUG("trampReadResponse: Response from CMD_Sensor: do nothing");
    break;
  }
  }
  // clear port
  port->flush();
  // The next packet is paced by trampWaitForPacketSlot().
  DEBUG("trampReadResponse: Succesful");
  return true;
  
  // clear port
  // port->flush();
  // return false;
}

#endif  // ARDUINO
