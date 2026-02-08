#include "motorControl.h"

motorControl::motorControl()
    : motors(nullptr), motorCount(0), pidIntervals(nullptr), lastUpdateTime(0) {
}

void motorControl::setMotor(motor_t *motorArray, uint8_t count) {
  motors = motorArray;
  motorCount = count;
}

void motorControl::setPidInterval(pidInterval_t *intervals) {
  pidIntervals = intervals;
}

void motorControl::init() {
  lastUpdateTime = micros();
  for (uint8_t i = 0; i < motorCount; i++) {
    motors[i].lastRawAngle = 0;
    motors[i].roundCount = 0;
    motors[i].totalAngle = 0;
    motors[i].speedPid.prevError = 0;
    motors[i].speedPid.integral = 0;
    motors[i].anglePid.prevError = 0;
    motors[i].anglePid.integral = 0;
  }
}

float motorControl::calculatePID(pid_params_t *pid, float error, float dt) {
  float pOut = pid->kp * error;
  pid->integral += pid->ki * error * dt;
  if (pid->integral > pid->integralLimit)
    pid->integral = pid->integralLimit;
  if (pid->integral < -pid->integralLimit)
    pid->integral = -pid->integralLimit;
  float derivative = (error - pid->prevError) / dt;
  pid->prevError = error;
  float output = pOut + pid->integral + (pid->kd * derivative);
  if (output > pid->outputLimit)
    output = pid->outputLimit;
  if (output < -pid->outputLimit)
    output = -pid->outputLimit;
  return output;
}

void motorControl::parseMotorFeedback(struct can_frame *readMsg) {
  uint8_t motorId = readMsg->can_id - 0x201;
  if (motorId >= motorCount)
    return;

  uint16_t rawAngle = (readMsg->data[0] << 8) | readMsg->data[1];
  int16_t rawSpeed = (readMsg->data[2] << 8) | readMsg->data[3];
  int16_t rawCurrent = (readMsg->data[4] << 8) | readMsg->data[5];
  motors[motorId].temperature = readMsg->data[6];

  // Calculate total gear ratio (motor internal * external gearing)
  float totalGearRatio =
      motors[motorId].gearRatio * motors[motorId].externalGearRatio;

  // RPM needs to be divided by total gear ratio to get final Output RPM
  motors[motorId].actualSpeed = (float)rawSpeed / totalGearRatio;
  motors[motorId].actualCurrent = (float)rawCurrent / 1000.0;

  // Angle Unwrapping
  int32_t diff = rawAngle - motors[motorId].lastRawAngle;
  if (diff < -4096)
    motors[motorId].roundCount++;
  else if (diff > 4096)
    motors[motorId].roundCount--;
  motors[motorId].lastRawAngle = rawAngle;

  // Calculate Total Rotor Angle
  float rotorAngle = (motors[motorId].roundCount * 360.0f) +
                     ((float)rawAngle * 360.0f / 8192.0f);

  // *** CRITICAL: Divide by Total Gear Ratio ***
  // Now 'totalAngle' represents the FINAL OUTPUT SHAFT angle
  motors[motorId].totalAngle = rotorAngle / totalGearRatio;

  // Also update actualAngle for single-revolution display (0-360)
  motors[motorId].actualAngle = (float)rawAngle * 360.0f / 8192.0f;
}

void motorControl::updateMotorCommand(struct can_frame *sendMsg,
                                      uint8_t motorIndex) {
  if (motorIndex >= motorCount)
    return;

  float dt = (micros() - lastUpdateTime) / 1000000.0f;
  if (dt <= 0.0f)
    dt = 0.001f;

  float currentCmd = 0.0f;
  motor_t *m = &motors[motorIndex];

  if (m->mode == MODE::SPEED) {
    float error = m->target.speed - m->actualSpeed;
    currentCmd = calculatePID(&m->speedPid, error, dt);
  } else if (m->mode == MODE::ANGLE) {
    // Cascade: Position Loop -> Speed Loop
    float posError = m->target.angle - m->totalAngle;
    float targetSpeed = calculatePID(&m->anglePid, posError, dt);
    float speedError = targetSpeed - m->actualSpeed;
    currentCmd = calculatePID(&m->speedPid, speedError, dt);
  }

  if (m->direction == DIRECTION::REV)
    currentCmd = -currentCmd;

  // Current (Amps) to C620 value (-16384 to 16384)
  // C620 max current is 20A, mapped to 16384
  int16_t sendVal = (int16_t)(currentCmd * (16384.0f / 20.0f));

  if (sendVal > 16384)
    sendVal = 16384;
  if (sendVal < -16384)
    sendVal = -16384;

  uint8_t frameIndex = (m->id - 1) / 4;
  uint8_t offset = (m->id - 1) % 4;
  sendMsg[frameIndex].data[offset * 2] = (sendVal >> 8) & 0xFF;
  sendMsg[frameIndex].data[offset * 2 + 1] = sendVal & 0xFF;
}

void motorControl::refresh(uint32_t currentTime, struct can_frame *sendMsg,
                           struct can_frame *readMsg) {
  lastUpdateTime = currentTime;

  if (readMsg && readMsg->can_id >= 0x201 && readMsg->can_id <= 0x208) {
    parseMotorFeedback(readMsg);
  }

  memset(sendMsg[0].data, 0, 8);
  memset(sendMsg[1].data, 0, 8);

  for (uint8_t i = 0; i < motorCount; i++) {
    if (motors[i].mode != MODE::SLEEP) {
      updateMotorCommand(sendMsg, i);
    }
  }
}

void motorControl::wakeAll() {
  for (uint8_t i = 0; i < motorCount; i++)
    motors[i].mode = MODE::SPEED;
}

void motorControl::sleepAll() {
  for (uint8_t i = 0; i < motorCount; i++) {
    motors[i].mode = MODE::SLEEP;
    motors[i].target.speed = 0;
    motors[i].target.angle = 0;
  }
}

bool motorControl::isConnected() {
  return true; // Placeholder
}
