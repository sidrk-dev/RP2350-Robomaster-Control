#include <SPI.h>

// -------------------- ABSOLUTE ENCODER (AMT203S on SPI1) --------------------
#define ENC_SCK_PIN D0  // GPIO26 -> SPI1 SCK
#define ENC_MOSI_PIN D1 // GPIO27 -> SPI1 MOSI
#define ENC_MISO_PIN D2 // GPIO28 -> SPI1 MISO

#define ENC_NOP 0x00
#define ENC_RD_POS 0x10
#define ENC_SET_ZERO 0x70
#define ENC_TIMEOUT 100

// Define Chip Select (CS) pins for the encoders
const uint8_t ENC_CS_PINS[] = {D3, D4};
const uint8_t NUM_ENCODERS = sizeof(ENC_CS_PINS) / sizeof(ENC_CS_PINS[0]);

void SPI1init() {
  for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
    pinMode(ENC_CS_PINS[i], OUTPUT);
    digitalWrite(ENC_CS_PINS[i], HIGH);
  }
  SPI1.setRX(ENC_MISO_PIN);
  SPI1.setTX(ENC_MOSI_PIN);
  SPI1.setSCK(ENC_SCK_PIN);
  SPI1.begin();
  SPI1.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE1));
}

uint8_t SPIWriteToEncoder(uint8_t encoderIndex, uint8_t sendByte) {
  uint8_t csPin = ENC_CS_PINS[encoderIndex];
  digitalWrite(csPin, LOW);
  delayMicroseconds(5);
  uint8_t received = SPI1.transfer(sendByte);
  digitalWrite(csPin, HIGH);
  delayMicroseconds(25);
  return received;
}

int16_t readAbsEncoder(uint8_t encoderIndex) {
  uint8_t data;
  uint8_t timeoutCounter = 0;

  data = SPIWriteToEncoder(encoderIndex, ENC_RD_POS);
  while (data != ENC_RD_POS && timeoutCounter++ < ENC_TIMEOUT) {
    data = SPIWriteToEncoder(encoderIndex, ENC_NOP);
  }
  if (timeoutCounter >= ENC_TIMEOUT) {
    return -1;
  }
  uint16_t pos = (SPIWriteToEncoder(encoderIndex, ENC_NOP) & 0x0F) << 8;
  pos |= SPIWriteToEncoder(encoderIndex, ENC_NOP);
  return (int16_t)pos;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("--- AMT203S Absolute Encoder Test ---");
  Serial.print("Testing ");
  Serial.print(NUM_ENCODERS);
  Serial.println(" encoders...");

  SPI1init();
}

void loop() {
  for (uint8_t e = 0; e < NUM_ENCODERS; e++) {
    int16_t raw = readAbsEncoder(e);

    Serial.print("Encoder ");
    Serial.print(e);
    Serial.print(" (CS=D");
    Serial.print(ENC_CS_PINS[e]);
    Serial.print("): ");

    if (raw >= 0) {
      float deg = 360.0f * ((float)raw / 4096.0f);
      Serial.print("Raw=");
      Serial.print(raw);
      Serial.print(" \tDeg=");
      Serial.print(deg, 2);
    } else {
      Serial.print("ERROR / TIMEOUT (Check wiring or SPI pins)");
    }
    Serial.print("   |   ");
  }
  Serial.println();

  // Read any serial commands (like setting zero)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("ZERO ")) {
      int idx = cmd.substring(5).toInt();
      if (idx >= 0 && idx < NUM_ENCODERS) {
        Serial.print("Setting Zero for Encoder ");
        Serial.println(idx);
        SPIWriteToEncoder(idx, ENC_SET_ZERO);
        delay(100);
      }
    }
  }

  delay(100); // 10Hz read rate for display
}