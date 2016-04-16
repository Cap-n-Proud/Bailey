
#define maxSpeed 255


int loopCounter = 0;
int lastSpeedUpdate = 0;
int lastDebugEncoders = 0;

 void debugEncoders() {
  Serial.print("L: ");
  Serial.print(leftMotorPosition);
  Serial.print("R: ");
  Serial.print(rightMotorPosition);
  Serial.print(" ");
  Serial.print(leftMotorSpeed*speedMultiplier);
  Serial.print(" ");
  Serial.print(rightMotorSpeed*speedMultiplier);

}

//Updates the motor speeds and the SpeedPID input
void updateMotorSpeeds() {
  leftMotorPosition = - MotorLeft.read(); //The minus sign is needed as the motor directions are opposite
  rightMotorPosition = MotorRight.read();
  
  leftMotorSpeed = (float) (leftMotorPosition - lastLeftMotorPosition);
  rightMotorSpeed = (float) (rightMotorPosition - lastRightMotorPosition);
  lastLeftMotorPosition = leftMotorPosition;
  lastRightMotorPosition = rightMotorPosition;  
  
  speedKalmanFilter.correct(rightMotorSpeed+leftMotorSpeed);
  speedKalmanFiltered = speedKalmanFilter.getState();
  speedPIDInput = speedKalmanFiltered;
  
 /* motorBtn.read();
    if (motorBtn.wasReleased()) {
      configuration.motorsON = !configuration.motorsON;
      motorLeft.setSpeed(0);
      motorRight.setSpeed(0); 
      //Serial.println(configuration.motorsON);
      
    }
  */
}






