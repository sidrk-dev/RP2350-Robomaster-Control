#include "motorControl.h"
#include "src/RPI_PICO_TimerInterrupt/src/RPi_Pico_TimerInterrupt.h"
#include "src/autowp-mcp2515/mcp2515.h"
#include <SPI.h>
#include <regex>

// Joints 1, 2, 3 Gear ratio is 94.23076923076923
// 10:1 gear ratio for the differential wrist

// -------------------- CONFIGURATION --------------------
/*#define RX_PIN 4  // gpio4 (MISO)
#define CS_PIN 5  // gpio5 (CS)x
#define SCK_PIN 6 // gpio6 (SCK)
#define TX_PIN 7  // gpio7 (MOSI)*/

#define RX_PIN 4  // D9  / GPIO4 (SPI0_MISO)
#define CS_PIN 1  // D7  / GPIO1 (SPI0_CSn)
#define SCK_PIN 2 // D8  / GPIO2 (SPI0_SCK)
#define TX_PIN 3  // D10 / GPIO3 (SPI0_MOSI)
#define HAND_DIR_PIN D5
#define HAND_PWM_PIN D6

// -------------------- ABSOLUTE ENCODER (AMT203S on SPI1) --------------------
#define ENC_SCK_PIN D0  // GPIO26 -> SPI1 SCK
#define ENC_MOSI_PIN D1 // GPIO27 -> SPI1 MOSI
#define ENC_MISO_PIN D2 // GPIO28 -> SPI1 MISO

#define ENC_NOP 0x00
#define ENC_RD_POS 0x10
#define ENC_SET_ZERO 0x70
#define ENC_TIMEOUT 100

const uint8_t ENC_CS_PINS[] = {D3, D4};
const uint8_t NUM_ENCODERS = sizeof(ENC_CS_PINS) / sizeof(ENC_CS_PINS[0]);

#define M2006_GEAR_RATIO 36.0
#define M3508_GEAR_RATIO 19.0

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
bool motorTelemetryEnabled[MOTOR_NUM] =
    {}; // per-motor telemetry toggle (T command)

// Per-joint logical limits (degrees). Enforced in P and PR commands.
// Defaults: 0 to 360 (full rotation). Set via JLIM command.
float jointMin[MOTOR_NUM];
float jointMax[MOTOR_NUM];

// Per-joint wrapped physical offset (degrees, 0-360) used only for
// diagnostics/telemetry after CAL. Closed-loop control uses zeroAtControlTotal.
float zeroAtPhysical[MOTOR_NUM];

// Per-joint logical zero in the same total-angle frame used by PID.
// logicalAngle = controlTotalAngle - zeroAtControlTotal.
float zeroAtControlTotal[MOTOR_NUM];

// Open-loop velocity feedforward gain (A per RPM), used by MV command.
float mvGainAperRpm[MOTOR_NUM];

// Encoder-to-motor mapping: absEncoderMotorMap[enc_idx] = motor_idx (0-based),
// -1 = unmapped
int8_t absEncoderMotorMap[8]; // supports up to 8 encoders
bool encoderDetected[8];      // tracks which encoders responded to SPI
uint8_t encoderRetryCount[8]; // retry counter for failed encoders
bool absEncoderReversed[8];   // true if encoder reads opposite from motor

struct {
  bool recvMotor = false;
  bool sendMotor = false;
} flag;

struct can_frame sendMsg[2] = {};
struct can_frame readMsg = {};
int handPwmCommand = 0;
uint32_t blinkActiveUntilMs = 0;
uint32_t blinkLastToggleMs = 0;
bool blinkLedState = false;

// -------------------- PROTOTYPES --------------------
void SPIinit();
void SPI1init();
void MCPinit();
void setMotorParam();
void recvMotor();
void sendMotor();
bool recvMotorFlag(struct repeating_timer *t);
bool sendMotorFlag(struct repeating_timer *t);
void printHelp();
void handleSerial();
void processCommand(String line);
int16_t readAbsEncoder(uint8_t encoderIndex);
uint8_t SPIWriteToEncoder(uint8_t encoderIndex, uint8_t sendByte);
void updateAbsEncoders();
void startBlinkLed();
static float getControlTotalAngle(uint8_t motorIndex);
static float getControlLogicalAngle(uint8_t motorIndex);
static bool isJointFullRange(uint8_t motorIndex);
static int8_t findMappedEncoderForMotor(uint8_t motorIndex);
static float projectLogicalTargetToLimits(uint8_t motorIndex, float targetDeg,
                                          float referenceDeg);
static void clampActiveAngleTargetToLimits(uint8_t motorIndex);
static void fallbackMotorToInternalEncoder(uint8_t motorIndex);
static void resetAbsStateForMotor(uint8_t motorIndex);
void setHandPwm(int pwmCommand);

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  SPIinit();
  SPI1init();
  MCPinit();
  setMotorParam();

  // Initialize joint limits to full range and zero offsets
  for (int i = 0; i < MOTOR_NUM; i++) {
    jointMin[i] = 0.0f;
    jointMax[i] = 360.0f;
    zeroAtPhysical[i] = 0.0f; // no offset by default
    zeroAtControlTotal[i] = 0.0f;
    mvGainAperRpm[i] = 0.02f; // default open-loop map: 100 RPM -> 2.0 A
  }

  // Default encoder-to-motor mapping: encoder 0 -> motor 0, encoder 1 -> motor
  // 1, etc.
  for (uint8_t i = 0; i < 8; i++) {
    absEncoderMotorMap[i] = (i < NUM_ENCODERS) ? (int8_t)i : -1;
    encoderDetected[i] = false;
    encoderRetryCount[i] = 0;
    absEncoderReversed[i] =
        true; // default: reversed (gearbox output reads opposite)
  }

  mc.setMotor(motor, MOTOR_NUM);
  mc.setPidInterval(&pidInterval);
  mc.init();

  // Use motor internal encoder for closed-loop control by default.
  // Absolute encoders remain available for APOS telemetry/debug.
  for (int i = 0; i < MOTOR_NUM; i++) {
    motor[i].useAbsEncoderForPID = false;
  }

  // Don't mark hasAbsEncoder=true yet - set on first successful SPI read

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
  pinMode(HAND_DIR_PIN, OUTPUT);
  pinMode(HAND_PWM_PIN, OUTPUT);
  analogWrite(HAND_PWM_PIN, 0);
  digitalWrite(HAND_DIR_PIN, LOW);

  printHelp();
}

// -------------------- LOOP --------------------
void loop() {
  handleSerial();

  // Read absolute encoders (~100Hz, time-gated)
  static uint32_t lastEncRead = 0;
  if (millis() - lastEncRead > 10) {
    updateAbsEncoders();
    lastEncRead = millis();
  }

  // Status Printer: prints any motor with telemetry enabled (via T command) or
  // in ANGLE mode
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 200) {
    bool anyPrinted = false;
    for (int i = 0; i < MOTOR_NUM; i++) {
      if (motor[i].mode == MODE::ANGLE || motorTelemetryEnabled[i]) {
        anyPrinted = true;

        // Choose control angle source used by closed-loop logic.
        float reportAngle = getControlTotalAngle(i);
        float logicalNow = getControlLogicalAngle(i);
        float rangeWidth = jointMax[i] - jointMin[i];
        float logicalDisplay =
            (rangeWidth >= 359.5f) ? wrap360f(logicalNow) : logicalNow;

        float physMod = fmod(reportAngle, 360.0f);
        if (physMod < 0)
          physMod += 360.0f;

        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" POS:"); // logical degrees (matches P command)
        Serial.print(logicalDisplay, 1);
        Serial.print(" PHYS:"); // raw physical reading
        Serial.print(physMod, 1);
        if (motor[i].hasAbsEncoder) {
          Serial.print(" APOS:"); // absolute encoder single-rev reading
          Serial.print(motor[i].absEncoderAngle, 1);
        }
        Serial.print(" Tot:");
        Serial.print(reportAngle, 1);
        Serial.print(" RPM:");
        Serial.print(motor[i].actualSpeed, 1);
        Serial.print(" A:");
        Serial.print(motor[i].actualCurrent, 2);
        Serial.print(" | ");
      }
    }
    if (anyPrinted)
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
  uint32_t nowMs = millis();
  if (blinkActiveUntilMs > nowMs) {
    if ((nowMs - blinkLastToggleMs) >= 200) {
      blinkLastToggleMs = nowMs;
      blinkLedState = !blinkLedState;
      digitalWrite(LED_BUILTIN, blinkLedState ? HIGH : LOW);
    }
  } else {
    blinkActiveUntilMs = 0;
    blinkLedState = false;
    digitalWrite(LED_BUILTIN, anyAngleMode ? HIGH : LOW);
  }
}

// -------------------- SERIAL COMMANDS --------------------
void printHelp() {
  Serial.println("\n=== RoboMaster Motor Control ===");
  Serial.println("  Use * as <id> to target ALL motors.  Separate cmds with ; "
                 "for batching.");
  Serial.println("");
  Serial.println("Position Control:");
  Serial.println("  P <id> <angle>    : Move to absolute logical angle");
  Serial.println(
      "  PR <id> <delta>   : Move RELATIVE delta degrees from current");
  Serial.println("  Example: P 1 90 | P * 0 | PR 2 -45");
  Serial.println("");
  Serial.println("Speed Control:");
  Serial.println("  M <id> <speed>    : Set speed in RPM (continuous)");
  Serial.println("  MV <id> <speed>   : Open-loop velocity (bypass Speed PID)");
  Serial.println("  MVG <id> <gain>   : Set MV gain (A per RPM)");
  Serial.println("  Example: M 1 500 | M * -300");
  Serial.println("");
  Serial.println("PID Tuning (full):");
  Serial.println("  AP <id> <kp> <ki> <kd> : Set all Angle PID gains");
  Serial.println("  SP <id> <kp> <ki> <kd> : Set all Speed PID gains");
  Serial.println("PID Tuning (single gain):");
  Serial.println("  KP  <id> <val>    : Angle Kp only");
  Serial.println("  KI  <id> <val>    : Angle Ki only");
  Serial.println("  KD  <id> <val>    : Angle Kd only");
  Serial.println("  AOL <id> <val>    : Angle output limit (max RPM)");
  Serial.println("  SKP <id> <val>    : Speed Kp only");
  Serial.println("  SKI <id> <val>    : Speed Ki only");
  Serial.println("  SKD <id> <val>    : Speed Kd only");
  Serial.println("  SOL <id> <val>    : Speed output limit (max Amps)");
  Serial.println("  SHOW <id>         : Show PID values");
  Serial.println("");
  Serial.println("Telemetry:");
  Serial.println("  T             : Toggle telemetry for ALL motors");
  Serial.println("  T <id>        : Toggle telemetry for one motor");
  Serial.println("");
  Serial.println("Calibration:");
  Serial.println("  CAL <id> <angle>  : Set current position to <angle>");
  Serial.println("");
  Serial.println("Joint Limits:");
  Serial.println(
      "  JLIM <id|*> <min> <max> : Set logical joint limits (degrees)");
  Serial.println(
      "  Example: JLIM 1 -90 90  : Joint 1 can only go -90 to +90 degrees");
  Serial.println("");
  Serial.println("Gear Ratio:");
  Serial.println("  GEAR <id> <ratio> : Set external gear ratio");
  Serial.println("");
  Serial.println("");
  Serial.println("Absolute Encoders:");
  Serial.println(
      "  ENCSHOW             : Show encoder mappings & raw readings");
  Serial.println(
      "  ENCMAP <enc> <motor> : Map encoder index to motor ID (0=unmap)");
  Serial.println(
      "  ABSPID <id|*> <0|1>  : Use mapped abs encoder for PID (safe sync)");
  Serial.println("  HOPEN [0-255]       : Open hand with PWM (default 255)");
  Serial.println("  HCLOSE [0-255]      : Close hand with PWM (default 255)");
  Serial.println("  HSTOP               : Stop hand PWM");
  Serial.println("  HAND <-255..255>    : Signed hand PWM (+open, -close)");
  Serial.println("  ENCZERO <enc>       : Set encoder zero point");
  Serial.println("  ENCREV <enc>        : Toggle encoder direction");
  Serial.println("  BLINK               : Blink onboard LED for 5 seconds");
  Serial.println("");
  Serial.println("S : Stop ALL motors");
  Serial.println("H : Help");
}

void handleSerial() {
  if (!Serial.available())
    return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0)
    return;
  line.toUpperCase();

  // --- Semicolon batch: split by ';' and process each sub-command ---
  int start = 0;
  while (true) {
    int sep = line.indexOf(';', start);
    String sub =
        (sep == -1) ? line.substring(start) : line.substring(start, sep);
    sub.trim();
    if (sub.length() > 0)
      processCommand(sub);
    if (sep == -1)
      break;
    start = sep + 1;
  }
}

// Resolves a motor ID string: returns -1 for wildcard *, else the 1-based int
// Sets isWildcard true when * is used
static float wrap360f(float angle) {
  while (angle >= 360.0f)
    angle -= 360.0f;
  while (angle < 0.0f)
    angle += 360.0f;
  return angle;
}

static float clampf(float v, float lo, float hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

// Return the wrapped equivalent of angleDeg that is closest to referenceDeg.
static float nearestEquivalent(float angleDeg, float referenceDeg) {
  float turns = roundf((referenceDeg - angleDeg) / 360.0f);
  return angleDeg + (turns * 360.0f);
}

static float getControlTotalAngle(uint8_t motorIndex) {
  if (motorIndex >= MOTOR_NUM)
    return 0.0f;
  if (motor[motorIndex].hasAbsEncoder &&
      motor[motorIndex].useAbsEncoderForPID) {
    return motor[motorIndex].absEncoderTotalAngle;
  }
  return motor[motorIndex].totalAngle;
}

static float getControlLogicalAngle(uint8_t motorIndex) {
  if (motorIndex >= MOTOR_NUM)
    return 0.0f;
  return getControlTotalAngle(motorIndex) - zeroAtControlTotal[motorIndex];
}

static bool isJointFullRange(uint8_t motorIndex) {
  if (motorIndex >= MOTOR_NUM)
    return true;
  return (jointMax[motorIndex] - jointMin[motorIndex]) >= 359.5f;
}

static int8_t findMappedEncoderForMotor(uint8_t motorIndex) {
  if (motorIndex >= MOTOR_NUM)
    return -1;
  for (uint8_t e = 0; e < NUM_ENCODERS; e++) {
    if (absEncoderMotorMap[e] == (int8_t)motorIndex)
      return (int8_t)e;
  }
  return -1;
}

static float projectLogicalTargetToLimits(uint8_t motorIndex, float targetDeg,
                                          float referenceDeg) {
  if (motorIndex >= MOTOR_NUM)
    return targetDeg;
  if (isJointFullRange(motorIndex)) {
    return nearestEquivalent(targetDeg, referenceDeg);
  }
  referenceDeg = clampf(referenceDeg, jointMin[motorIndex], jointMax[motorIndex]);
  if (targetDeg >= jointMin[motorIndex] && targetDeg <= jointMax[motorIndex]) {
    return targetDeg;
  }
  return clampf(nearestEquivalent(targetDeg, referenceDeg),
                jointMin[motorIndex], jointMax[motorIndex]);
}

static void clampActiveAngleTargetToLimits(uint8_t motorIndex) {
  if (motorIndex >= MOTOR_NUM || motor[motorIndex].mode != MODE::ANGLE)
    return;
  if (isJointFullRange(motorIndex))
    return;

  float currentLogical = getControlLogicalAngle(motorIndex);
  float targetLogical =
      motor[motorIndex].target.angle - zeroAtControlTotal[motorIndex];
  float clampedLogical =
      projectLogicalTargetToLimits(motorIndex, targetLogical, currentLogical);
  motor[motorIndex].target.angle =
      zeroAtControlTotal[motorIndex] + clampedLogical;
  motor[motorIndex].anglePid.integral = 0.0f;
  motor[motorIndex].anglePid.prevError = 0.0f;
}

static void fallbackMotorToInternalEncoder(uint8_t motorIndex) {
  if (motorIndex >= MOTOR_NUM)
    return;
  float previousLogical = getControlLogicalAngle(motorIndex);
  float previousTargetLogical =
      motor[motorIndex].target.angle - zeroAtControlTotal[motorIndex];
  zeroAtControlTotal[motorIndex] = motor[motorIndex].totalAngle - previousLogical;
  motor[motorIndex].useAbsEncoderForPID = false;
  if (motor[motorIndex].mode == MODE::ANGLE) {
    motor[motorIndex].target.angle =
        zeroAtControlTotal[motorIndex] + previousTargetLogical;
    clampActiveAngleTargetToLimits(motorIndex);
  }
  motor[motorIndex].anglePid.integral = 0.0f;
  motor[motorIndex].anglePid.prevError = 0.0f;
}

static void resetAbsStateForMotor(uint8_t motorIndex) {
  if (motorIndex >= MOTOR_NUM)
    return;
  motor[motorIndex].absEncoderAngle = 0.0f;
  motor[motorIndex].absEncoderTotalAngle = 0.0f;
  motor[motorIndex].absEncoderLastRaw = 0;
  motor[motorIndex].absEncoderRoundCount = 0;
  motor[motorIndex].hasAbsEncoder = false;
  motor[motorIndex].useAbsEncoderForPID = false;
}

void setHandPwm(int pwmCommand) {
  if (pwmCommand > 255)
    pwmCommand = 255;
  if (pwmCommand < -255)
    pwmCommand = -255;
  handPwmCommand = pwmCommand;
  if (pwmCommand > 0) {
    digitalWrite(HAND_DIR_PIN, HIGH);
    analogWrite(HAND_PWM_PIN, pwmCommand);
  } else if (pwmCommand < 0) {
    digitalWrite(HAND_DIR_PIN, LOW);
    analogWrite(HAND_PWM_PIN, -pwmCommand);
  } else {
    analogWrite(HAND_PWM_PIN, 0);
  }
}

void startBlinkLed() {
  blinkActiveUntilMs = millis() + 5000;
  blinkLastToggleMs = 0;
  blinkLedState = false;
  digitalWrite(LED_BUILTIN, LOW);
}

static void dispatchMotorIDs(String idStr, bool &isWildcard, int &singleId) {
  if (idStr == "*") {
    isWildcard = true;
    singleId = -1;
  } else {
    isWildcard = false;
    singleId = idStr.toInt();
  }
}

void processCommand(String line) {
  // Command: HELP
  if (line == "H") {
    printHelp();
    return;
  }

  if (line == "BLINK") {
    startBlinkLed();
    Serial.println("BLINK STARTED");
    return;
  }

  if (line == "HOPEN") {
    setHandPwm(255);
    Serial.println("HAND OPEN PWM 255");
    return;
  }

  if (line == "HCLOSE") {
    setHandPwm(-255);
    Serial.println("HAND CLOSE PWM 255");
    return;
  }

  if (line == "HSTOP") {
    setHandPwm(0);
    Serial.println("HAND STOP");
    return;
  }

  if (line.startsWith("HOPEN ")) {
    int pwm = line.substring(6).toInt();
    if (pwm < 0)
      pwm = -pwm;
    setHandPwm(pwm);
    Serial.print("HAND OPEN PWM ");
    Serial.println(abs(handPwmCommand));
    return;
  }

  if (line.startsWith("HCLOSE ")) {
    int pwm = line.substring(7).toInt();
    if (pwm < 0)
      pwm = -pwm;
    setHandPwm(-pwm);
    Serial.print("HAND CLOSE PWM ");
    Serial.println(abs(handPwmCommand));
    return;
  }

  if (line.startsWith("HAND ")) {
    int pwm = line.substring(5).toInt();
    setHandPwm(pwm);
    Serial.print("HAND PWM ");
    Serial.println(handPwmCommand);
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
      motor[i].mode = MODE::SLEEP;
      motor[i].target.speed = 0.0f;
      motor[i].target.current = 0.0f;
      // Reset PID state to prevent integral windup from keeping motors moving
      motor[i].speedPid.integral = 0;
      motor[i].speedPid.prevError = 0;
      motor[i].anglePid.integral = 0;
      motor[i].anglePid.prevError = 0;
    }
    Serial.println("STOPPED ALL MOTORS");
    return;
  }

  // Command: TELEMETRY TOGGLE -> "T" / explicit "TON" / "TOFF" / "T <id>"
  if (line == "T" || line == "TON" || line == "TOFF" || line.startsWith("T ")) {
    if (line == "TON") {
      for (int i = 0; i < MOTOR_NUM; i++)
        motorTelemetryEnabled[i] = true;
      Serial.println("Telemetry ALL motors: ON");
    } else if (line == "TOFF") {
      for (int i = 0; i < MOTOR_NUM; i++)
        motorTelemetryEnabled[i] = false;
      Serial.println("Telemetry ALL motors: OFF");
    } else if (line == "T") {
      // Toggle all motors — use the state of motor 0 as the reference
      bool newState = !motorTelemetryEnabled[0];
      for (int i = 0; i < MOTOR_NUM; i++)
        motorTelemetryEnabled[i] = newState;
      Serial.print("Telemetry ALL motors: ");
      Serial.println(newState ? "ON" : "OFF");
    } else {
      int id = line.substring(2).toInt();
      if (id < 1 || id > MOTOR_NUM) {
        Serial.print("Error: ID must be 1-");
        Serial.println(MOTOR_NUM);
        return;
      }
      motorTelemetryEnabled[id - 1] = !motorTelemetryEnabled[id - 1];
      Serial.print("Telemetry M");
      Serial.print(id);
      Serial.println(motorTelemetryEnabled[id - 1] ? ": ON" : ": OFF");
    }
    return;
  }

  // Command: SET ANGLE PID -> "AP <id|*> <kp> <ki> <kd>"
  if (line.startsWith("AP ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    int sp3 = line.indexOf(' ', sp2 + 1);
    int sp4 = line.indexOf(' ', sp3 + 1);
    if (sp1 == -1 || sp2 == -1 || sp3 == -1 || sp4 == -1) {
      Serial.println("Error: Use format 'AP <id|*> <kp> <ki> <kd>'");
      return;
    }
    String idStr = line.substring(sp1 + 1, sp2);
    float kp = line.substring(sp2 + 1, sp3).toFloat();
    float ki = line.substring(sp3 + 1, sp4).toFloat();
    float kd = line.substring(sp4 + 1).toFloat();
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      motor[i].anglePid.kp = kp;
      motor[i].anglePid.ki = ki;
      motor[i].anglePid.kd = kd;
      motor[i].anglePid.integral = 0;
      motor[i].anglePid.prevError = 0;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" APid: Kp=");
      Serial.print(kp);
      Serial.print(" Ki=");
      Serial.print(ki);
      Serial.print(" Kd=");
      Serial.println(kd);
    }
    return;
  }

  // Command: SET SPEED PID -> "SP <id|*> <kp> <ki> <kd>"
  if (line.startsWith("SP ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    int sp3 = line.indexOf(' ', sp2 + 1);
    int sp4 = line.indexOf(' ', sp3 + 1);
    if (sp1 == -1 || sp2 == -1 || sp3 == -1 || sp4 == -1) {
      Serial.println("Error: Use format 'SP <id|*> <kp> <ki> <kd>'");
      return;
    }
    String idStr = line.substring(sp1 + 1, sp2);
    float kp = line.substring(sp2 + 1, sp3).toFloat();
    float ki = line.substring(sp3 + 1, sp4).toFloat();
    float kd = line.substring(sp4 + 1).toFloat();
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      motor[i].speedPid.kp = kp;
      motor[i].speedPid.ki = ki;
      motor[i].speedPid.kd = kd;
      motor[i].speedPid.integral = 0;
      motor[i].speedPid.prevError = 0;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" SPid: Kp=");
      Serial.print(kp);
      Serial.print(" Ki=");
      Serial.print(ki);
      Serial.print(" Kd=");
      Serial.println(kd);
    }
    return;
  }

  // Command: SINGLE GAIN PID -> "KP/KI/KD/AOL/SKP/SKI/SKD/SOL <id> <val>"
  const char *singleGainCmds[] = {"KP ",  "KI ",  "KD ",  "AOL ",
                                  "SKP ", "SKI ", "SKD ", "SOL "};
  for (int g = 0; g < 8; g++) {
    if (line.startsWith(singleGainCmds[g])) {
      int sp1 = line.indexOf(' ');
      int sp2 = line.indexOf(' ', sp1 + 1);
      if (sp1 == -1 || sp2 == -1) {
        Serial.print("Error: Use format '");
        Serial.print(singleGainCmds[g]);
        Serial.println("<id> <val>'");
        return;
      }
      String idStr = line.substring(sp1 + 1, sp2);
      float val = line.substring(sp2 + 1).toFloat();
      bool isWild;
      int singleId;
      dispatchMotorIDs(idStr, isWild, singleId);
      int startM = isWild ? 0 : singleId - 1;
      int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
      if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
        Serial.print("Error: ID must be 1-");
        Serial.println(MOTOR_NUM);
        return;
      }
      for (int i = startM; i <= endM; i++) {
        switch (g) {
        case 0:
          motor[i].anglePid.kp = val;
          break;
        case 1:
          motor[i].anglePid.ki = val;
          motor[i].anglePid.integral = 0;
          break;
        case 2:
          motor[i].anglePid.kd = val;
          motor[i].anglePid.prevError = 0;
          break;
        case 3: // AOL
          motor[i].anglePid.outputLimit = val;
          break;
        case 4:
          motor[i].speedPid.kp = val;
          break;
        case 5:
          motor[i].speedPid.ki = val;
          motor[i].speedPid.integral = 0;
          break;
        case 6:
          motor[i].speedPid.kd = val;
          motor[i].speedPid.prevError = 0;
          break;
        case 7: // SOL
          motor[i].speedPid.outputLimit = val;
          break;
        }
        Serial.print(singleGainCmds[g]);
        Serial.print("M");
        Serial.print(i + 1);
        Serial.print(" = ");
        Serial.println(val);
      }
      return;
    }
  }

  // Command: SET EXTERNAL GEAR RATIO
  if (line.startsWith("GEAR ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'GEAR <id|*> <ratio>'");
      return;
    }

    String idStr = line.substring(sp1 + 1, sp2);
    float ratio = line.substring(sp2 + 1).toFloat();
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    if (ratio <= 0) {
      Serial.println("Error: Gear ratio must be positive");
      return;
    }

    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      // Backward compatibility: If ratio is large and motor has an internal
      // gearbox (like M2006/M3508), the user/GUI likely sent the TOTAL gear
      // ratio instead of just the external ratio.
      if (ratio > 10.0f && motor[i].gearRatio > 1.0f) {
        motor[i].externalGearRatio = ratio / motor[i].gearRatio;
      } else {
        motor[i].externalGearRatio = ratio;
      }

      Serial.print("Motor ");
      Serial.print(i + 1);
      Serial.print(" gear ratio: ");
      Serial.print(motor[i].externalGearRatio);
      Serial.print(":1 (Total: ");
      Serial.print(motor[i].gearRatio * motor[i].externalGearRatio);
      Serial.println(":1)");
    }
    return;
  }

  // Command: JOINT LIMITS -> "JLIM <id|*> <min> <max>"
  if (line.startsWith("JLIM ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    int sp3 = line.indexOf(' ', sp2 + 1);
    if (sp1 == -1 || sp2 == -1 || sp3 == -1) {
      Serial.println("Error: Use format 'JLIM <id|*> <min> <max>'");
      return;
    }
    String idStr = line.substring(sp1 + 1, sp2);
    float minA = line.substring(sp2 + 1, sp3).toFloat();
    float maxA = line.substring(sp3 + 1).toFloat();
    if (maxA <= minA) {
      Serial.println("Error: max must be > min");
      return;
    }
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      jointMin[i] = minA;
      jointMax[i] = maxA;
      clampActiveAngleTargetToLimits(i);
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" logical limits: [");
      Serial.print(minA);
      Serial.print(", ");
      Serial.print(maxA);
      Serial.print("]");
      if (motor[i].mode == MODE::ANGLE) {
        Serial.print(" target=");
        Serial.print(motor[i].target.angle - zeroAtControlTotal[i]);
      }
      Serial.println();
    }
    return;
  }

  // Command: CALIBRATE POSITION
  if (line.startsWith("CAL ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'CAL <id|*> <angle>'");
      return;
    }

    String idStr = line.substring(sp1 + 1, sp2);
    float calibAngle = line.substring(sp2 + 1).toFloat();
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    float calibAngleWrapped = wrap360f(calibAngle);

    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      motor[i].mode = MODE::SPEED;
      motor[i].target.speed = 0.0;
      motor[i].speedPid.integral = 0;
      motor[i].speedPid.prevError = 0;
      motor[i].anglePid.integral = 0;
      motor[i].anglePid.prevError = 0;

      // Use the same reference as closed-loop control.
      float refAngle = getControlTotalAngle(i);
      float physMod = fmod(refAngle, 360.0f);
      if (physMod < 0)
        physMod += 360.0f;
      zeroAtPhysical[i] = fmod(physMod - calibAngleWrapped + 360.0f, 360.0f);
      zeroAtControlTotal[i] = refAngle - calibAngle;

      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" CAL: angle%360=");
      Serial.print(physMod);
      Serial.print(motor[i].hasAbsEncoder ? " (abs)" : " (mot)");
      Serial.print(" -> logical ");
      Serial.print(calibAngle);
      Serial.print(" (offset=");
      Serial.print(zeroAtPhysical[i]);
      Serial.print(", totalZero=");
      Serial.print(zeroAtControlTotal[i]);
      Serial.println(")");
    }
    return;
  }

  // Command: RELATIVE POSITION -> "PR <id|*> <delta>"
  if (line.startsWith("PR ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'PR <id|*> <delta>'");
      return;
    }
    String idStr = line.substring(sp1 + 1, sp2);
    float delta = line.substring(sp2 + 1).toFloat();
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      float currentLogical = getControlLogicalAngle(i);
      float desiredLogical = projectLogicalTargetToLimits(
          i, currentLogical + delta, currentLogical);

      motor[i].mode = MODE::ANGLE;
      motor[i].target.angle = zeroAtControlTotal[i] + desiredLogical;
      motor[i].anglePid.integral = 0.0f;
      motor[i].anglePid.prevError = 0.0f;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" PR ");
      Serial.print(delta);
      Serial.print(" -> log ");
      Serial.println(desiredLogical);
    }
    return;
  }

  // Command: POSITION CONTROL -> "P <id|*> <angle>"
  if (line.startsWith("P ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'P <id|*> <angle>'");
      return;
    }

    String idStr = line.substring(sp1 + 1, sp2);
    float targetDeg = line.substring(sp2 + 1).toFloat();
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    // NOTE: Do NOT normalize or clamp targetDeg here — it's a LOGICAL angle.
    // Normalization and clamping happen per-motor inside the loop,
    // after the zero offset is applied.

    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      float currentLogical = getControlLogicalAngle(i);
      float desiredLogical =
          projectLogicalTargetToLimits(i, targetDeg, currentLogical);
      float error = desiredLogical - currentLogical;

      motor[i].mode = MODE::ANGLE;
      motor[i].target.angle = zeroAtControlTotal[i] + desiredLogical;
      // Reset angle PID to prevent stale integral from fighting the new target
      motor[i].anglePid.integral = 0;
      motor[i].anglePid.prevError = 0;

      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" -> log ");
      Serial.print(desiredLogical);
      Serial.print(" (err=");
      Serial.print(error);
      Serial.println(")");
    }
    return;
  }

  // Command: SPEED CONTROL -> "M <id|*> <speed>"
  if (line.startsWith("MVG ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'MVG <id|*> <gain>'");
      return;
    }
    String idStr = line.substring(sp1 + 1, sp2);
    float gain = line.substring(sp2 + 1).toFloat();
    if (gain <= 0.0f) {
      Serial.println("Error: gain must be > 0");
      return;
    }
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      mvGainAperRpm[i] = gain;
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" MV gain: ");
      Serial.print(gain, 4);
      Serial.println(" A/RPM");
    }
    return;
  }

  // Command: OPEN-LOOP VELOCITY -> "MV <id|*> <speed>"
  if (line.startsWith("MV ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'MV <id|*> <speed>'");
      return;
    }

    String idStr = line.substring(sp1 + 1, sp2);
    float speed = line.substring(sp2 + 1).toFloat();
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      float currentFF = speed * mvGainAperRpm[i];
      if (currentFF > 20.0f)
        currentFF = 20.0f;
      if (currentFF < -20.0f)
        currentFF = -20.0f;
      if (fabsf(speed) < 0.5f)
        currentFF = 0.0f;

      motor[i].mode = MODE::CURRENT;
      motor[i].target.current = currentFF;
      motor[i].speedPid.integral = 0.0f;
      motor[i].speedPid.prevError = 0.0f;
      motor[i].anglePid.integral = 0.0f;
      motor[i].anglePid.prevError = 0.0f;

      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" MV ");
      Serial.print(speed, 1);
      Serial.print(" rpm -> ");
      Serial.print(currentFF, 2);
      Serial.println(" A (open-loop)");
    }
    return;
  }

  // Command: SPEED CONTROL -> "M <id|*> <speed>"
  if (line.startsWith("M ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);

    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'M <id|*> <speed>'");
      return;
    }

    String idStr = line.substring(sp1 + 1, sp2);
    float speed = line.substring(sp2 + 1).toFloat();
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      if (fabsf(speed) < 1.0f) {
        motor[i].mode = MODE::SLEEP;
        motor[i].target.speed = 0.0f;
        motor[i].target.current = 0.0f;
        motor[i].speedPid.integral = 0;
        motor[i].speedPid.prevError = 0;
        motor[i].anglePid.integral = 0;
        motor[i].anglePid.prevError = 0;
      } else {
        motor[i].mode = MODE::SPEED;
        motor[i].target.speed = speed;
      }
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" -> ");
      if (fabsf(speed) < 1.0f) {
        Serial.println("SLEEP");
      } else {
        Serial.print(speed);
        Serial.println(" rpm");
      }
    }
    return;
  }

  // Command: ENCODER MAP -> "ENCMAP <enc> <motor>"
  if (line.startsWith("ENCMAP ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'ENCMAP <enc_idx> <motor_id>'");
      return;
    }
    int encIdx = line.substring(sp1 + 1, sp2).toInt();
    int motorId = line.substring(sp2 + 1).toInt();
    if (encIdx < 0 || encIdx >= NUM_ENCODERS) {
      Serial.print("Error: Encoder index must be 0-");
      Serial.println(NUM_ENCODERS - 1);
      return;
    }
    if (motorId < 0 || motorId > MOTOR_NUM) {
      Serial.print("Error: Motor ID must be 0 (unmapped) or 1-");
      Serial.println(MOTOR_NUM);
      return;
    }
    // Clear old mapping
    int8_t oldMotor = absEncoderMotorMap[encIdx];
    if (oldMotor >= 0 && oldMotor < MOTOR_NUM) {
      fallbackMotorToInternalEncoder(oldMotor);
      resetAbsStateForMotor(oldMotor);
    }
    // Set new mapping
    absEncoderMotorMap[encIdx] = (motorId > 0) ? (int8_t)(motorId - 1) : -1;
    encoderDetected[encIdx] = false;
    encoderRetryCount[encIdx] = 0;
    if (motorId > 0) {
      resetAbsStateForMotor(motorId - 1); // wait for first valid read
    }
    Serial.print("Encoder ");
    Serial.print(encIdx);
    Serial.print(" -> Motor ");
    Serial.println(motorId);
    return;
  }

  // Command: SELECT PID FEEDBACK SOURCE -> "ABSPID <id|*> <0|1>"
  if (line.startsWith("ABSPID ")) {
    int sp1 = line.indexOf(' ');
    int sp2 = line.indexOf(' ', sp1 + 1);
    if (sp1 == -1 || sp2 == -1) {
      Serial.println("Error: Use format 'ABSPID <id|*> <0|1>'");
      return;
    }
    String idStr = line.substring(sp1 + 1, sp2);
    int enable = line.substring(sp2 + 1).toInt();
    if (!(enable == 0 || enable == 1)) {
      Serial.println("Error: ABSPID value must be 0 or 1");
      return;
    }
    bool isWild;
    int singleId;
    dispatchMotorIDs(idStr, isWild, singleId);
    if (!isWild && (singleId < 1 || singleId > MOTOR_NUM)) {
      Serial.print("Error: ID must be 1-");
      Serial.println(MOTOR_NUM);
      return;
    }

    int startM = isWild ? 0 : singleId - 1;
    int endM = isWild ? MOTOR_NUM - 1 : singleId - 1;
    for (int i = startM; i <= endM; i++) {
      float previousLogical = getControlLogicalAngle(i);
      float previousTargetLogical = motor[i].target.angle - zeroAtControlTotal[i];

      if (enable == 1) {
        int8_t encIdx = findMappedEncoderForMotor(i);
        if (encIdx < 0) {
          Serial.print("M");
          Serial.print(i + 1);
          Serial.println(" ABSPID skipped: no encoder mapped");
          continue;
        }
        if (!motor[i].hasAbsEncoder || !encoderDetected[encIdx]) {
          Serial.print("M");
          Serial.print(i + 1);
          Serial.println(" ABSPID skipped: encoder not detected yet");
          continue;
        }

        zeroAtControlTotal[i] = motor[i].absEncoderTotalAngle - previousLogical;
        motor[i].useAbsEncoderForPID = true;
      } else {
        zeroAtControlTotal[i] = motor[i].totalAngle - previousLogical;
        motor[i].useAbsEncoderForPID = false;
      }

      if (motor[i].mode == MODE::ANGLE) {
        motor[i].target.angle = zeroAtControlTotal[i] + previousTargetLogical;
        clampActiveAngleTargetToLimits(i);
      }
      motor[i].anglePid.integral = 0.0f;
      motor[i].anglePid.prevError = 0.0f;

      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(" PID source: ");
      Serial.print(motor[i].useAbsEncoderForPID ? "ABS" : "MOTOR");
      Serial.print(" logical=");
      Serial.println(getControlLogicalAngle(i));
    }
    return;
  }

  // Command: ENCODER SET ZERO -> "ENCZERO <enc_idx>"
  if (line.startsWith("ENCZERO ")) {
    int encIdx = line.substring(8).toInt();
    if (encIdx < 0 || encIdx >= NUM_ENCODERS) {
      Serial.print("Error: Encoder index must be 0-");
      Serial.println(NUM_ENCODERS - 1);
      return;
    }
    // Send set_zero_point command to encoder
    SPIWriteToEncoder(encIdx, ENC_SET_ZERO);
    delay(100);
    // Reset unwrapping state for the mapped motor
    int8_t mIdx = absEncoderMotorMap[encIdx];
    if (mIdx >= 0 && mIdx < MOTOR_NUM) {
      fallbackMotorToInternalEncoder(mIdx);
      resetAbsStateForMotor(mIdx);
      encoderDetected[encIdx] = false;
      encoderRetryCount[encIdx] = 0;
    }
    Serial.print("Encoder ");
    Serial.print(encIdx);
    Serial.println(" zero set");
    return;
  }

  // Command: ENCODER SHOW -> "ENCSHOW"
  if (line == "ENCSHOW") {
    Serial.println("\n--- Absolute Encoders ---");
    for (uint8_t e = 0; e < NUM_ENCODERS; e++) {
      int16_t raw = readAbsEncoder(e);
      Serial.print("  Enc ");
      Serial.print(e);
      Serial.print(" (CS=D");
      Serial.print(ENC_CS_PINS[e]);
      Serial.print(") -> Motor ");
      int8_t mIdx = absEncoderMotorMap[e];
      if (mIdx >= 0)
        Serial.print(mIdx + 1);
      else
        Serial.print("NONE");
      Serial.print("  Raw: ");
      if (raw >= 0) {
        float deg = 360.0f * ((float)raw / 4096.0f);
        Serial.print(raw);
        Serial.print(" (");
        Serial.print(deg, 1);
        Serial.print(" deg)");
      } else {
        Serial.print("ERROR");
      }
      Serial.println();
    }
    return;
  }

  // Command: ENCODER REVERSE -> "ENCREV <enc_idx>"
  if (line.startsWith("ENCREV ")) {
    int encIdx = line.substring(7).toInt();
    if (encIdx < 0 || encIdx >= NUM_ENCODERS) {
      Serial.print("Error: Encoder index must be 0-");
      Serial.println(NUM_ENCODERS - 1);
      return;
    }
    absEncoderReversed[encIdx] = !absEncoderReversed[encIdx];
    // Reset unwrapping so direction change takes effect cleanly
    int8_t mIdx = absEncoderMotorMap[encIdx];
    if (mIdx >= 0 && mIdx < MOTOR_NUM) {
      fallbackMotorToInternalEncoder(mIdx);
      resetAbsStateForMotor(mIdx);
      encoderDetected[encIdx] = false;
      encoderRetryCount[encIdx] = 0;
    }
    Serial.print("Encoder ");
    Serial.print(encIdx);
    Serial.print(" direction: ");
    Serial.println(absEncoderReversed[encIdx] ? "REVERSED" : "NORMAL");
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
  motor[0].gearRatio = M3508_GEAR_RATIO;
}

// -------------------- CAN TASKS --------------------
void recvMotor() {
  while (mcp.readMessage(&readMsg) == MCP2515::ERROR_OK) {
    mc.refresh(micros(), sendMsg, &readMsg);
  }
  flag.recvMotor = false;
}

void sendMotor() {
  // Keep command outputs fresh even if feedback frames pause.
  mc.refresh(micros(), sendMsg, nullptr);
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

// -------------------- ABSOLUTE ENCODER (SPI1) --------------------
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

void updateAbsEncoders() {
  for (uint8_t e = 0; e < NUM_ENCODERS; e++) {
    int8_t mIdx = absEncoderMotorMap[e];
    if (mIdx < 0 || mIdx >= MOTOR_NUM)
      continue;

    float previousLogical = getControlLogicalAngle(mIdx);
    float previousTargetLogical =
        motor[mIdx].target.angle - zeroAtControlTotal[mIdx];
    bool firstDetect = !encoderDetected[e];
    int16_t raw = readAbsEncoder(e);
    if (raw < 0) {
      encoderRetryCount[e]++;
      continue;
    }
    encoderRetryCount[e] = 0;

    motor[mIdx].absEncoderAngle = 360.0f * ((float)raw / 4096.0f);

    if (firstDetect) {
      motor[mIdx].absEncoderLastRaw = (uint16_t)raw;
      motor[mIdx].absEncoderRoundCount = 0;
    } else {
      int32_t diff = (int32_t)raw - (int32_t)motor[mIdx].absEncoderLastRaw;
      if (diff < -2048)
        motor[mIdx].absEncoderRoundCount++;
      else if (diff > 2048)
        motor[mIdx].absEncoderRoundCount--;
    }
    motor[mIdx].absEncoderLastRaw = (uint16_t)raw;

    motor[mIdx].absEncoderTotalAngle =
        (motor[mIdx].absEncoderRoundCount * 360.0f) +
        (360.0f * ((float)raw / 4096.0f));

    if (absEncoderReversed[e]) {
      motor[mIdx].absEncoderAngle = 360.0f - motor[mIdx].absEncoderAngle;
      motor[mIdx].absEncoderTotalAngle = -motor[mIdx].absEncoderTotalAngle;
    }

    if (firstDetect) {
      encoderDetected[e] = true;
      motor[mIdx].hasAbsEncoder = true;
      motor[mIdx].useAbsEncoderForPID = false;
      Serial.print("Abs encoder ");
      Serial.print(e);
      Serial.print(" detected -> Motor ");
      Serial.print(mIdx + 1);
      Serial.println(" (telemetry only)");
    } else {
      motor[mIdx].hasAbsEncoder = true;
    }
  }
}
