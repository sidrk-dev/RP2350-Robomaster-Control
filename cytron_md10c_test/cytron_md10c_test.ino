// ---------------------------------------------------------
// Cytron MD10C Motor Driver Test for RP2350 (Raspberry Pi Pico)
// ---------------------------------------------------------
// The MD10C accepts "Sign-Magnitude" PWM:
// - DIR Pin controls Direction (HIGH = Forward, LOW = Reverse)
// - PWM Pin controls Speed (0 to 255)

// Define the pins connected to the MD10C
#define MD10C_PWM_PIN 15 // Connect to PWM pin on MD10C
#define MD10C_DIR_PIN 14 // Connect to DIR pin on MD10C

void setup() {
  Serial.begin(115200);

  // Configure pins as outputs
  pinMode(MD10C_PWM_PIN, OUTPUT);
  pinMode(MD10C_DIR_PIN, OUTPUT);

  // The MD10C supports PWM up to 20kHz, which eliminates audible whining.
  // For the RP2040/RP2350 Arduino core, you can set the analog write frequency.
  analogWriteFreq(20000);

  // Set resolution to 8-bit (0-255)
  analogWriteResolution(8);

  // Ensure the motor starts stopped
  digitalWrite(MD10C_DIR_PIN, LOW);
  analogWrite(MD10C_PWM_PIN, 0);

  delay(2000);
  Serial.println("MD10C Test Started");
}

void loop() {
  Serial.println("Spinning FORWARD at 50% Speed...");
  digitalWrite(MD10C_DIR_PIN, HIGH); // Set direction Forward
  analogWrite(MD10C_PWM_PIN, 127);   // Set speed to ~50% (127 out of 255)
  delay(3000);

  Serial.println("Coasting to a STOP...");
  analogWrite(MD10C_PWM_PIN, 0); // 0 PWM = Stop
  delay(2000);

  Serial.println("Spinning REVERSE at 100% Speed...");
  digitalWrite(MD10C_DIR_PIN, LOW); // Set direction Reverse
  analogWrite(MD10C_PWM_PIN, 255);  // Set speed to 100% (255 out of 255)
  delay(3000);

  Serial.println("Coasting to a STOP...");
  analogWrite(MD10C_PWM_PIN, 0); // 0 PWM = Stop
  delay(2000);
}
