#if defined(ARDUINO)
#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

// Minimal error flags to satisfy existing code paths
enum sswhdErrors : uint8_t {
  sswhdNoErrors              = 0,
  sswhdIsNotListening        = 1 << 0,
  sswhdBufferIsEmpty         = 1 << 1,
  sswhdTxDelayIsZero         = 1 << 2,
  sswhdRXDelayStopBitNotSet  = 1 << 3,
};

// Use Serial1/UART1 for VTX
// Define this exactly once in your main .ino or a single .cpp:
//   HardwareSerial VTXSER(1);
extern HardwareSerial VTXSER;

class HardwareSerialAdapter {
public:
  // Keep the same ctor shape you used (tx, rx, invert) — rx is ignored
  HardwareSerialAdapter(int txPin, int rxPin, bool invert)
  : _txPin(txPin), _rxPin(rxPin), _invert(invert) {}

  void begin(long baud, uint32_t config) {
    _baud = baud; _config = config;
    // Start in RX-only so the bus is released and VTX can reply
    VTXSER.end();
    pinMode(_rxPin, INPUT_PULLUP);
    VTXSER.begin(_baud, _config, /*rx*/_rxPin, /*tx*/-1, /*invert*/_invert);
    _txMode = false;
  }
  
// Flip direction explicitly (library calls this)
  void enableTx(bool on) {
    if (_txMode == on) return;
    _txMode = on;
    VTXSER.end();
    if (_txMode) {
      // TX-only (drive the line, no RX)
      VTXSER.begin(_baud, _config, /*rx*/-1,       /*tx*/_txPin, _invert);
    } else {
      // RX-only (release the line to let VTX talk)
      pinMode(_rxPin, INPUT_PULLUP);
      VTXSER.begin(_baud, _config, /*rx*/_rxPin,   /*tx*/-1,     _invert);
    }
  }
  // Some code calls enableTX with capital X — make it an alias
  void enableTX(bool on) { enableTx(on); }

  void listen() { enableTx(false); }   // same semantic as classic SoftwareSerial

  // Write helpers — auto flip to TX if we are currently in RX-only mode
  size_t write(const uint8_t* buf, size_t len) {
    // bool restore = !_txMode;
    // if (restore) enableTx(true);
    size_t w = VTXSER.write(buf, len);
    // VTXSER.flush();
    // if (restore) enableTx(false);
    return w;
  }
  size_t write(uint8_t b) {
    return write(&b, 1);
  }

  void writeDummyByte() {
    uint8_t z = 0x00;
    (void)write(z);
  }

  int16_t available() { return VTXSER.available(); }
  int     read()      { return VTXSER.read(); }
  int     peek()      { return VTXSER.peek(); }

  void flush() { VTXSER.flush(); }           // TX flush only
  void drainRx() { while (VTXSER.available()) (void)VTXSER.read(); }

  void clearErrors() {}
  sswhdErrors getErrors() const { return sswhdNoErrors; }
  long getSpeed() const { return _baud; }
  uint32_t getConfiguration() const { return _config; }


  void dumpReceiveBuffer() {
    while (VTXSER.available()) {
      uint8_t b = VTXSER.read();
      Serial.printf("%02X ", b);
    }
    Serial.println();
  }


private:
  int _txPin, _rxPin;
  bool _invert;
  long _baud = 0;
  uint32_t _config = SERIAL_8N1;
  bool _txMode = false; // false=RX-only, true=TX-only
};

#endif  // ARDUINO
