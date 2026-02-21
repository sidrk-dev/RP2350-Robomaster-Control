#include "motorControl.h"
#include "src/RPI_PICO_TimerInterrupt/src/RPi_Pico_TimerInterrupt.h"
#include "src/autowp-mcp2515/mcp2515.h"
#include <SPI.h>
#include <regex>

// Joints 1, 2, 3 Gear ratio is 94.23076923076923
// 10:1 gear ratio for the differential wrist

// -------------------- CONFIGURATION --------------------
#define RX_PIN 4  // gpio4 (MISO)
#define CS_PIN 5  // gpio5 (CS)x
#define SCK_PIN 6 // gpio6 (SCK)
#define TX_PIN 7  // gpio7 (MOSI)

#define M2006_GEAR_RATIO 19.0
#define M3508_GEAR_RATIO 19.0

// -------------------- CYTRON MD10C CONFIG --------------------
#define CYTRON_PWM_PIN 26
#define CYTRON_DIR_PIN 27

// Supports IDs 1 through 8
#define MOTOR_NUM 8

#define DT_RECV_DATA 1.0f / (MOTOR_NUM * C620_FEEDBACK_125HZ)
#define DT_SEND_DATA 1.0f / C620_FEEDBACK_125HZ

// -------------------- OBJECTS --------------------
MCP2515 mcp(CS_PIN, 8000000, &SPI);
motorControl mc;
RPI_PICO_Timer recvTimer(0);
RPI_PICO_Timer sendTimer(1);

// -------------------- STATE MANAGEMENT --------------------
motor_t motor[MOTOR_NUM];
pidInterval_t pidInterval = {8, 4, 2};

struct {
  bool recvMotor = false;
  bool sendMotor = false;
} flag;

struct can_frame sendMsg[2] = {};
struct can_frame readMsg = {};

// -------------------- PROTOTYPES --------------------
void SPIinit();
void MCPinit();
void setMotorParam();
void recvMotor();
void sendMotor();
bool recvMotorFlag(struct repeating_timer *t);
bool sendMotorFlag(struct repeating_timer *t);
void printHelp();
void handleSerial();

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  SPIinit();
  MCPinit();
  setMotorParam();

  mc.setMotor(motor, MOTOR_NUM);
  mc.setPidInterval(&pidInterval);
  mc.init();

  // Timer interrupts
  recvTimer.attachInterruptInterval((uint32_t)(DT_RECV_DATA * 1000000),
                                    recvMotorFlag);
  sendTimer.attachInterruptInterval((uint32_t)(DT_SEND_DATA * 1000000),
                                    sendMotorFlag);

  // Prepare CAN frames (0x200 for IDs 1-4, 0x1FF for IDs 5-8)
  sendMsg[0].can_id = 0x200;
  sendMsg[0].can_dlc = 8;
  sendMsg[1].can_id = 0x1FF;
  sendMsg[1].can_dlc = 8;

  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize Cytron MD10C Pines
  pinMode(CYTRON_PWM_PIN, OUTPUT);
  pinMode(CYTRON_DIR_PIN, OUTPUT);
#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  analogWriteFreq(20000);   // 20kHz, eliminates whine
  analogWriteResolution(8); // 8-bit (0-255)
#endif
  digitalWrite(CYTRON_DIR_PIN, LOW);
  analogWrite(CYTRON_PWM_PIN, 0);

  printHelp();
}

// -------------------- LOOP --------------------
void loop() {
  handleSerial();

  // Status Printer for motors in ANGLE mode
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    bool anyInAngleMode = false;
    for (int i = 0; i < MOTOR_NUM; i++) {
      if (motor[i].mode == MODE::ANGLE) {
        anyInAngleMode = true;
        float currentMod = fmod(motor[i].totalAngle, 360.0);
        if (currentMod < 0)
          currentMod += 360.0;

        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" POS:");
        Serial.print(currentMod, 1);
        Serial.print(" Tot:");
        Serial.print(motor[i].totalAngle, 1);
        Serial.print(" RPM:");
        Serial.print(motor[i].actualSpeed, 1);
        Serial.print(" A:");
        Serial.print(motor[i].actualCurrent, 2);
        Serial.print(" | ");
      }
    }
    if (anyInAngleMode)
      Serial.println();
    lastPrint = millis();
  }

  // CAN Communication
  if (flag.recvMotor)
    recvMotor();
  if (flag.sendMotor)
    sendMotor();

  // Status LED: ON if any motor is in angle mode
  bool anyAngleMode = false;
  for (int i = 0; i < MOTOR_NUM; i++) {
    if (motor[i].mode == MODE::ANGLE)
      anyAngleMode = true;
  }
  digitalWrite(LED_BUILTIN, anyAngleMode ? HIGH : LOW);
}

// -------------------- SERIAL COMMANDS --------------------
void printHelp() {
  Serial.println("\n=== RoboMaster Motor Control ===");
  Serial.println("Position Control (All Motors):");
  Serial.println(
      "  P <id> <angle>   : Move Motor <id> to absolute angle (0-360)");
  Serial.println("  Example: P 1 90  (Motor 1 to 90 degrees)");
  Serial.println("  Example: P 2 270 (Motor 2 to 270 degrees)");
  Serial.println("");
  Serial.println("Speed Control (All Motors):");
  Serial.println(
      "  M <id> <speed>   : Set Motor <id> to <speed> RPM (continuous)");
  Serial.println("  Example: M 2 500 (Motor 2, Forward 500rpm)");
  Serial.println("  Example: M 3 -300 (Motor 3, Reverse 300rpm)");
  Serial.println("");
  Serial.println("PID Tuning (All Motors):");
  Serial.println(
      "  AP <id> <kp> <ki> <kd> : Set Angle PID gains for Motor <id>");
  Serial.println(
      "  SP <id> <kp> <ki> <kd> : Set Speed PID gains for Motor <id>");
  Serial.println("  SHOW <id>              : Show PID values for Motor <id>");
  Serial.println("  Example: AP 1 0.5 0 0.1  (Motor 1 angle PID)");
  Serial.println("");
  Serial.println("Calibration (All Motors):");
  Serial.println(
      "  CAL <id> <angle> : Set Motor <id> current position to <angle>");
  Serial.println("  Example: CAL 1 0   (Motor 1 current position = 0 degrees)");
  Serial.println(
      "  Example: CAL 2 180 (Motor 2 current position = 180 degrees)");
  Serial.println("");
  Serial.println("Gear Ratio (All Motors):");
  Serial.println(
      "  GEAR <id> <ratio> : Set external gear ratio for Motor <id>");
  Serial.println("  Example: GEAR 1 5   (Motor 1 has 5:1 external gearing)");
  Serial.println("  Example: GEAR 2 1   (Motor 2 has no external gearing)");
  Serial.println("");

  Serial.println("Cytron Motor Control (MD10C):");
  Serial.println(
      "  C <speed>              : Set Cytron motor speed (-255 to 255)");
  Serial.println("  Example: C 127         (Half speed forward)");
  Serial.println("  Example: C -255        (Full speed reverse)");
  Serial.println("");
  Serial.println("S                  : Stop ALL motors immediately");
  Serial.println("H                  : Help");
}

void handleSerial() {
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0)
    return;
  line.toUpperCase();

  // Command: HELP
  if (line == "H") {
    printHelp();
    return;
  }

  // Command: SHOW PID VALUES
  if (line.startsWith("SHOW ")) {
    String idStr = line.substring(5);
    int id = idStr.toInt();

    if (id < 1 || id > MOTOR_NUM) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    int idx = id - 1;
    Serial.print("\n--- Motor ");
    Serial.print(id);
    Serial.println(" PID Values ---");
    Serial.print("Angle PID: Kp=");
    Serial.print(motor[idx].anglePid.kp);
    Serial.print(", Ki=");
    Serial.print(motor[idx].anglePid.ki);
    Serial.print(", Kd=");
    Serial.print(motor[idx].anglePid.kd);
    Serial.print(", OutLimit=");
    Serial.print(motor[idx].anglePid.outputLimit);
    Serial.print(", IntLimit=");
    Serial.println(motor[idx].anglePid.integralLimit);

    Serial.print("Speed PID: Kp=");
    Serial.print(motor[idx].speedPid.kp);
    Serial.print(", Ki=");
    Serial.print(motor[idx].speedPid.ki);
    Serial.print(", Kd=");
    Serial.print(motor[idx].speedPid.kd);
    Serial.print(", OutLimit=");
    Serial.print(motor[idx].speedPid.outputLimit);
    Serial.print(", IntLimit=");
    Serial.println(motor[idx].speedPid.integralLimit);
    return;
  }

  // Command: STOP ALL
  if (line == "S") {
    for (int i = 0; i < MOTOR_NUM; i++) {
      motor[i].mode = MODE::SPEED;
      motor[i].target.speed = 0.0f;
    }
    digitalWrite(CYTRON_DIR_PIN, LOW);
    analogWrite(CYTRON_PWM_PIN, 0);
    Serial.println("STOPPED ALL MOTORS");
    return;
  }

  // Command: CYTRON MOTOR CONTROL
  if (line.startsWith("C ")) {
    int sp1 = line.indexOf(' ');
    if (sp1 == -1) {
      Serial.println("Error: Use format 'C <speed>' (-255 to 255)");
      return;
    }
    int speed = line.substring(sp1 + 1).toInt();

    if (speed > 255)
      speed = 255;
    if (speed < -255)
      speed = -255;

    if (speed >= 0) {
      digitalWrite(CYTRON_DIR_PIN, HIGH);
      analogWrite(CYTRON_PWM_PIN, speed);
    } else {
      digitalWrite(CYTRON_DIR_PIN, LOW);
      analogWrite(CYTRON_PWM_PIN, -speed);
    }
    Serial.print("Cytron Motor -> ");
    Serial.println(speed);
    return;
  }

  // Command: SET ANGLE PID
  if (line.startsWith("AP ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    int sp3 = line.indexOf(' ', sp2 + 1);
    int sp4 = line.indexOf(' ', sp3 + 1);

    if (sp1 == -1 || sp2 == -1 || sp3 == -1 || sp4 == -1) {
      Serial.println("Error: Use format 'AP <id> <kp> <ki> <kd>'");
      return;
    }

    int id = line.substring(sp1 + 1, sp2).toInt();
    float kp = line.substring(sp2 + 1, sp3).toFloat();
    float ki = line.substring(sp3 + 1, sp4).toFloat();
    float kd = line.substring(sp4 + 1).toFloat();

    if (id < 1 || id > MOTOR_NUM) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    int idx = id - 1;
    motor[idx].anglePid.kp = kp;
    motor[idx].anglePid.ki = ki;
    motor[idx].anglePid.kd = kd;
    motor[idx].anglePid.integral = 0;  // Reset integral
    motor[idx].anglePid.prevError = 0; // Reset derivative

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" Angle PID updated: Kp=");
    Serial.print(kp);
    Serial.print(", Ki=");
    Serial.print(ki);
    Serial.print(", Kd=");
    Serial.println(kd);
    return;
  }

  // Command: SET SPEED PID
  if (line.startsWith("SP ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    int sp3 = line.indexOf(' ', sp2 + 1);
    int sp4 = line.indexOf(' ', sp3 + 1);

    if (sp1 == -1 || sp2 == -1 || sp3 == -1 || sp4 == -1) {
      Serial.println("Error: Use format 'SP <id> <kp> <ki> <kd>'");
      return;
    }

    int id = line.substring(sp1 + 1, sp2).toInt();
    float kp = line.substring(sp2 + 1, sp3).toFloat();
    float ki = line.substring(sp3 + 1, sp4).toFloat();
    float kd = line.substring(sp4 + 1).toFloat();

    if (id < 1 || id > MOTOR_NUM) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    int idx = id - 1;
    motor[idx].speedPid.kp = kp;
    motor[idx].speedPid.ki = ki;
    motor[idx].speedPid.kd = kd;
    motor[idx].speedPid.integral = 0;  // Reset integral
    motor[idx].speedPid.prevError = 0; // Reset derivative

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" Speed PID updated: Kp=");
    Serial.print(kp);
    Serial.print(", Ki=");
    Serial.print(ki);
    Serial.print(", Kd=");
    Serial.println(kd);
    return;
  }

  // Command: SET EXTERNAL GEAR RATIO
  if (line.startsWith("GEAR ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'GEAR <id> <ratio>'");
      return;
    }

    int id = line.substring(sp1 + 1, sp2).toInt();
    float ratio = line.substring(sp2 + 1).toFloat();

    if (id < 1 || id > MOTOR_NUM) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    if (ratio <= 0) {
      Serial.println("Error: Gear ratio must be positive");
      return;
    }

    int idx = id - 1;
    motor[idx].externalGearRatio = ratio;

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" external gear ratio set to ");
    Serial.print(ratio);
    Serial.print(":1 (Total ratio: ");
    Serial.print(motor[idx].gearRatio * motor[idx].externalGearRatio);
    Serial.println(":1)");
    return;
  }

  // Command: CALIBRATE POSITION
  if (line.startsWith("CAL ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'CAL <id> <angle>'");
      return;
    }

    int id = line.substring(sp1 + 1, sp2).toInt();
    float calibAngle = line.substring(sp2 + 1).toFloat();

    if (id < 1 || id > MOTOR_NUM) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    int idx = id - 1;

    // Constrain to 0-360
    while (calibAngle >= 360.0)
      calibAngle -= 360.0;
    while (calibAngle < 0.0)
      calibAngle += 360.0;

    // CRITICAL: Stop the motor first and switch to SPEED mode
    motor[idx].mode = MODE::SPEED;
    motor[idx].target.speed = 0.0;

    // Get current raw encoder angle (0-360 from the encoder itself)
    float currentRawAngle = motor[idx].actualAngle;

    // Calculate what roundCount should be to make totalAngle equal calibAngle
    // totalAngle = (roundCount * 360) + (rawAngle / gearRatio)
    // We want: calibAngle = (roundCount * 360) + (currentRawAngle)
    // So: roundCount = (calibAngle - currentRawAngle) / 360, rounded to nearest
    // int

    float offset = calibAngle - currentRawAngle;
    motor[idx].roundCount = (int32_t)round(offset / 360.0f);

    // Now recalculate totalAngle with the new roundCount
    motor[idx].totalAngle = (motor[idx].roundCount * 360.0f) + currentRawAngle;

    // Update target angle to match so there's no error if switched back to
    // ANGLE mode
    motor[idx].target.angle = motor[idx].totalAngle;

    // Reset PID states to prevent jumps
    motor[idx].anglePid.integral = 0;
    motor[idx].anglePid.prevError = 0;
    motor[idx].speedPid.integral = 0;
    motor[idx].speedPid.prevError = 0;

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" calibrated: Current position set to ");
    Serial.print(motor[idx].totalAngle);
    Serial.print(" degrees (target was ");
    Serial.print(calibAngle);
    Serial.println(")");
    Serial.print("Motor stopped. Use 'P ");
    Serial.print(id);
    Serial.println(" <angle>' to move to new position.");
    return;
  }

  // Command: POSITION CONTROL (All Motors)
  if (line.startsWith("P ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'P <id> <angle>'");
      return;
    }

    int id = line.substring(sp1 + 1, sp2).toInt();
    float targetDeg = line.substring(sp2 + 1).toFloat();

    if (id < 1 || id > MOTOR_NUM) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    int idx = id - 1;

    // Constrain input to 0-360
    while (targetDeg >= 360.0)
      targetDeg -= 360.0;
    while (targetDeg < 0.0)
      targetDeg += 360.0;

    // Get current physical angle
    float currentTotal = motor[idx].totalAngle;

    // Calculate current position in 0-360 space
    float currentMod = fmod(currentTotal, 360.0);
    if (currentMod < 0)
      currentMod += 360.0;

    // Calculate shortest path error
    float error = targetDeg - currentMod;

    // Normalize error to shortest path (-180 to +180)
    if (error > 180.0)
      error -= 360.0;
    if (error < -180.0)
      error += 360.0;

    // Apply to Absolute Target
    motor[idx].mode = MODE::ANGLE;
    motor[idx].target.angle = currentTotal + error;

    Serial.print("M");
    Serial.print(id);
    Serial.print(" Going to: ");
    Serial.print(targetDeg);
    Serial.print(" | Path: ");
    Serial.print(error);
    Serial.print(" | Abs Target: ");
    Serial.println(motor[idx].target.angle);
    return;
  }

  // Command: SPEED CONTROL -> "M <id> <speed>"
  if (line.startsWith("M ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'M <id> <speed>'");
      return;
    }

    String idStr = line.substring(sp1 + 1, sp2);
    String speedStr = line.substring(sp2 + 1);

    int id = idStr.toInt();
    float speed = speedStr.toFloat();

    // Validation
    if (id < 1 || id > MOTOR_NUM) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    // Apply Command
    int idx = id - 1;
    motor[idx].mode = MODE::SPEED;
    motor[idx].target.speed = speed;

    Serial.print("Motor ");
    Serial.print(id);
    Serial.print(" -> ");
    Serial.print(speed);
    Serial.println(" rpm (continuous)");
    return;
  }

  Serial.println("Unknown command.");
}

// -------------------- INIT --------------------
void SPIinit() {
  SPI.setRX(RX_PIN);
  SPI.setCS(CS_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.setTX(TX_PIN);
  SPI.begin();
}

void MCPinit() {
  mcp.reset();
  delay(10);
  mcp.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp.setNormalMode();
}

void setMotorParam() {
  for (int i = 0; i < MOTOR_NUM; i++) {
    motor[i].id = i + 1;
    motor[i].mode = MODE::SPEED;
    motor[i].gearRatio = M2006_GEAR_RATIO;
    motor[i].externalGearRatio = 1.0; // Default: no external gearing
    motor[i].direction = DIRECTION::FWD;

    // Position PID: Output shaft degrees -> RPM
    // Higher kp because error is in output degrees (smaller numbers)
    motor[i].anglePid = {5, 1.0, 0.0, 500.0, 5.0, 0.0, 0.0}; // Max 500 RPM

    // Speed PID: RPM -> Amps
    motor[i].speedPid = {0.3, 0.0, 0.0, 20.0, 5.0, 0.0, 0.0}; // Max 20 Amps

    motor[i].currentPid = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Not used

    motor[i].target = {0.0, 0.0, 0.0};
  }

  // Special tuning for Motor 1 if it's an M3508
  // Uncomment if Motor 1 is M3508:
  // motor[0].gearRatio = M3508_GEAR_RATIO;
}

// -------------------- CAN TASKS --------------------
void recvMotor() {
  while (mcp.readMessage(&readMsg) == MCP2515::ERROR_OK) {
    mc.refresh(micros(), sendMsg, &readMsg);
  }
  flag.recvMotor = false;
}

void sendMotor() {
  mcp.sendMessage(&sendMsg[0]); // ID 1-4
  if (MOTOR_NUM > 4) {
    mcp.sendMessage(&sendMsg[1]); // ID 5-8
  }
  flag.sendMotor = false;
}

// -------------------- TIMERS --------------------
bool recvMotorFlag(struct repeating_timer *t) {
  flag.recvMotor = true;
  return true;
}

bool sendMotorFlag(struct repeating_timer *t) {
  flag.sendMotor = true;
  return true;
}
