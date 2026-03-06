#include <SPI.h>

#define baudRate 115200
#define timeoutLimit 100

#define nop 0x00
#define rd_pos 0x10
#define set_zero_point 0x70

// Left-side SPI1 pins
#define PIN_SCK D0  // GPIO26 -> SPI1 SCK
#define PIN_MOSI D1 // GPIO27 -> SPI1 MOSI
#define PIN_MISO D2 // GPIO28 -> SPI1 MISO

// CS pins: **do NOT** reuse D2 here
const uint8_t CS_PINS[] = {D3, D4};
const uint8_t NUM_ENCODERS = sizeof(CS_PINS) / sizeof(CS_PINS[0]);

void setup() {
  Serial.begin(baudRate);
  delay(2000);

  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    pinMode(CS_PINS[i], OUTPUT);
    digitalWrite(CS_PINS[i], HIGH);
  }

  // Use SPI1, not SPI
  SPI1.setRX(PIN_MISO);
  SPI1.setTX(PIN_MOSI);
  SPI1.setSCK(PIN_SCK);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE1));

  delay(500);
  Serial.println("AMT203S-V Multi-Encoder Test - XIAO RP2350 (Left-side SPI1)");
  Serial.println();
}

void loop() {
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    int16_t pos = readEncoder(i);

    Serial.print("Encoder ");
    Serial.print(i + 1);
    Serial.print(": ");

    if (pos >= 0) {
      float deg = 360.0f * ((float)pos / 4096.0f);
      Serial.print(deg, 1);
      Serial.print(" deg  (raw: ");
      Serial.print(pos);
      Serial.print(")");
    } else {
      Serial.print("ERROR");
    }

    Serial.print("  |  ");
  }

  Serial.println();
  delay(100);
}

int16_t readEncoder(uint8_t encoderIndex) {
  uint8_t data;
  uint8_t timeoutCounter = 0;

  data = SPIWriteTo(encoderIndex, rd_pos);

  while (data != rd_pos && timeoutCounter++ < timeoutLimit) {
    data = SPIWriteTo(encoderIndex, nop);
  }

  if (timeoutCounter >= timeoutLimit) {
    return -1;
  }

  uint16_t pos = (SPIWriteTo(encoderIndex, nop) & 0x0F) << 8;
  pos |= SPIWriteTo(encoderIndex, nop);

  return (int16_t)pos;
}

uint8_t SPIWriteTo(uint8_t encoderIndex, uint8_t sendByte) {
  uint8_t csPin = CS_PINS[encoderIndex];

  digitalWrite(csPin, LOW);
  delayMicroseconds(5);
  uint8_t received = SPI1.transfer(sendByte); // SPI1 here
  digitalWrite(csPin, HIGH);
  delayMicroseconds(25);

  return received;
}