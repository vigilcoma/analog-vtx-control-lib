#include <Arduino.h>

// VTXControl's ESP32 adapter uses this UART instance. Define it exactly once
// in the application before including VTXControl.h.
HardwareSerial VTXSER(1);

#include <VTXControl.h>

constexpr int VTX_LINE_PIN = 5;
VTXControl* vtx = nullptr;

void setup() {
  Serial.begin(115200);
  delay(300);

  // Choose SmartAudio or Tramp for the attached VTX.
  vtx = new VTXControl(VTXMode::SmartAudio, VTX_LINE_PIN);

  // SmartAudio needs an initial settings exchange to learn its protocol version.
  if (!vtx->updateParameters()) {
    Serial.println("VTX did not answer. Check the VTX control wire and common ground.");
    return;
  }

  // Use the exact frequency from the attached VTX's frequency table.
  if (vtx->setFrequency(5800)) {
    vtx->flush();
    Serial.println("VTX frequency command sent.");
  }
}

void loop() {
}
