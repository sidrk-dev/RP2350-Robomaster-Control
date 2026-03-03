#include <SPI.h>

#define baudRate 115200
#define timeoutLimit 100

#define nop          0x00
#define rd_pos       0x10
#define set_zero_point 0x70

// XIAO RP2350 hardware SPI pins
#define PIN_SCK   D8
#define PIN_MISO  D9
#define PIN_MOSI  D10

// 
#define PIN_CS    D3
// #define PIN_CS2   D2   // Encoder 2

uint16_t position = 0;

void setup() {
  Serial.begin(baudRate);
  delay(2000); // Give time for Serial Monitor to connect on XIAO

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // Explicitly tell SPI which pins to use
  SPI.setRX(PIN_MISO);
  SPI.setTX(PIN_MOSI);
  SPI.setSCK(PIN_SCK);
  SPI.setCS(PIN_CS);
  SPI.begin();
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE1));

  delay(500);

  Serial.println("AMT203S-V Encoder Test - XIAO RP2350");
  Serial.println("Rotate encoder to see position changes");
  Serial.println();
}

void loop() {
  int16_t pos = readEncoder();

  if (pos >= 0) {
    position = (uint16_t)pos;
    float deg = 360.0 * ((float)position / 4096.0);

    Serial.print("Encoder: ");
    Serial.print(deg, 1);
    Serial.print(" degrees  (raw: ");
    Serial.print(position);
    Serial.println(")");
  } else {
    Serial.println("Encoder: ERROR - check wiring");
  }

  delay(100);
}

int16_t readEncoder() {
  uint8_t data;
  uint8_t timeoutCounter = 0;
  uint16_t currentPosition;

  // Send read position command
  data = SPIWriteTo(rd_pos);

  // Wait for encoder to echo back rd_pos (0x10) as acknowledgment
  while (data != rd_pos && timeoutCounter++ < timeoutLimit) {
    data = SPIWriteTo(nop);
  }

  if (timeoutCounter >= timeoutLimit) {
    return -1; // Timed out — wiring issue or wrong pins
  }

  // Read high byte (only lower 4 bits valid), then low byte
  currentPosition  = (SPIWriteTo(nop) & 0x0F) << 8;
  currentPosition |=  SPIWriteTo(nop);

  return (int16_t)currentPosition;
}

uint8_t SPIWriteTo(uint8_t sendByte) {
  uint8_t received;

  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(5);         // Let CS settle before clocking
  received = SPI.transfer(sendByte);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(25);        // Recovery time between transactions

  return received;
}