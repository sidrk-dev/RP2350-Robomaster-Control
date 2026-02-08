// -------------------- HOMING FUNCTIONS --------------------
void startHoming(uint8_t motorIndex, float speed, float currentThreshold) {
  homingState[motorIndex].isHoming = true;
  homingState[motorIndex].startTime = millis();
  homingState[motorIndex].lastAngle = motor[motorIndex].totalAngle;
  homingState[motorIndex].lastMoveTime = millis();
  homingState[motorIndex].homingSpeed = speed;
  homingState[motorIndex].stallCurrent = currentThreshold;
  homingState[motorIndex].stallTime = 300; // 500ms stall confirmation

  // Set motor to slow speed mode
  motor[motorIndex].mode = MODE::SPEED;
  motor[motorIndex].target.speed = speed;

  Serial.print("Homing started: ");
  Serial.print(speed);
  Serial.print(" RPM, stall threshold ");
  Serial.print(currentThreshold);
  Serial.println(" A");
}

void updateHoming() {
  uint32_t currentTime = millis();

  for (uint8_t i = 0; i < MOTOR_NUM; i++) {
    if (!homingState[i].isHoming)
      continue;

    // Check for stall condition: high current + no movement
    bool highCurrent =
        fabs(motor[i].actualCurrent) >= homingState[i].stallCurrent;
    bool notMoving = fabs(motor[i].totalAngle - homingState[i].lastAngle) <
                     0.5; // Less than 0.5 degree movement

    if (highCurrent && notMoving) {
      // Motor is stalled - check if it's been stalled long enough
      if ((currentTime - homingState[i].lastMoveTime) >=
          homingState[i].stallTime) {
        // Stall confirmed! Stop motor and set zero position
        motor[i].mode = MODE::SPEED;
        motor[i].target.speed = 0.0;

        // Set current position as zero
        motor[i].totalAngle = 0.0;
        motor[i].roundCount = 0;
        motor[i].target.angle = 0.0;

        // Reset PID states
        motor[i].anglePid.integral = 0;
        motor[i].anglePid.prevError = 0;
        motor[i].speedPid.integral = 0;
        motor[i].speedPid.prevError = 0;

        homingState[i].isHoming = false;

        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.println(" homing complete! Position set to 0 degrees.");
        Serial.print("Stall detected at ");
        Serial.print(motor[i].actualCurrent);
        Serial.println(" A");
      }
    } else {
      // Motor is still moving, update last move time
      if (fabs(motor[i].totalAngle - homingState[i].lastAngle) >= 0.5) {
        homingState[i].lastMoveTime = currentTime;
        homingState[i].lastAngle = motor[i].totalAngle;
      }
    }

    // Timeout check (30 seconds)
    if ((currentTime - homingState[i].startTime) > 30000) {
      motor[i].mode = MODE::SPEED;
      motor[i].target.speed = 0.0;
      homingState[i].isHoming = false;

      Serial.print("Motor ");
      Serial.print(i + 1);
      Serial.println(" homing TIMEOUT! No limit found in 30s.");
    }
  }
}
