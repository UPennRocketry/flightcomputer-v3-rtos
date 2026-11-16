// Ground receiver: ESP32 + RFM95 (SX1276) LoRa module.
//
// Receives every frame from the flight computer and forwards it over USB serial
// as one line per frame:
//
//     RX,<rssi_dbm>,<snr_db>,<hex bytes of the frame>
//
// ground_station.py reads these lines. The receiver does no decoding, so the
// packet format can change without touching this sketch.
//
// Library: RadioHead (RH_RF95). Install via Arduino Library Manager.
// The radio settings MUST match app_config.h on the flight computer:
//   frequency 427.760 MHz, SF7, 125 kHz, CR 4/5, sync word 0x12 (RadioHead default),
//   explicit header, CRC on.
//
// This is also written to be trivial to port to the Teensy if the ground side
// ends up on one; only the pin numbers change.

#include <SPI.h>
#include <RH_RF95.h>

// ---- Wiring (adjust to your board) ----
constexpr int kRfm95Cs = 5;
constexpr int kRfm95Rst = 14;
constexpr int kRfm95Irq = 26;   // DIO0, must be an interrupt-capable pin
constexpr int kLed = 2;

// ---- Must match the flight computer ----
constexpr float kFreqMHz = 427.760f;
// RH_RF95::Bw125Cr45Sf128 == 125 kHz, CR 4/5, SF7. If app_config.h moves to
// 250 kHz use Bw250Cr45Sf128 ... see RH_RF95.h for the preset list.
constexpr RH_RF95::ModemConfigChoice kModem = RH_RF95::Bw125Cr45Sf128;

RH_RF95 rf95(kRfm95Cs, kRfm95Irq);

void setup() {
  Serial.begin(115200);
  pinMode(kLed, OUTPUT);
  pinMode(kRfm95Rst, OUTPUT);
  digitalWrite(kRfm95Rst, HIGH);
  delay(10);
  digitalWrite(kRfm95Rst, LOW);
  delay(10);
  digitalWrite(kRfm95Rst, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("ERR,rf95 init failed");
    delay(1000);
  }
  rf95.setFrequency(kFreqMHz);
  rf95.setModemConfig(kModem);
  rf95.setPromiscuous(true);   // accept any RadioHead destination address
  Serial.println("INFO,receiver ready");
}

void loop() {
  if (!rf95.available()) {
    return;
  }
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (!rf95.recv(buf, &len)) {
    return;
  }
  digitalWrite(kLed, HIGH);

  // RadioHead strips its 4-byte header in recv(); re-add nothing. The flight
  // computer's own driver writes the same header, so payload starts at buf[0].
  Serial.print("RX,");
  Serial.print(rf95.lastRssi());
  Serial.print(',');
  Serial.print(rf95.lastSNR());
  Serial.print(',');
  static const char hex[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < len; ++i) {
    Serial.write(hex[buf[i] >> 4]);
    Serial.write(hex[buf[i] & 0x0F]);
  }
  Serial.println();
  digitalWrite(kLed, LOW);
}
