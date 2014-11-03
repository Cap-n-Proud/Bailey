
void debugChart2() {
  sendPlotData("rightMotorSpeed", rightMotorSpeed);
  sendPlotData("leftMotorSpeed", leftMotorSpeed);
  sendPlotData("motorSpeed", motorSpeed);
  sendPlotData("speedPIDInput", speedPIDInput);
  sendPlotData("speedPIDOutput", speedPIDOutput);
  sendPlotData("speedPIDSetpoint", speedPIDSetpoint);
  sendPlotData("anglePIDInput", anglePIDInput);
  sendPlotData("anglePIDOutput", anglePIDOutput);
  sendPlotData("anglePIDSetpoint", anglePIDSetpoint);
  //sendPlotData("steering", steering);
  //sendPlotData("motor1SteeringOffset", motor1SteeringOffset);
  //sendPlotData("motor2SteeringOffset", motor2SteeringOffset);
}
void sendPlotData(String seriesName, float data) {
  Serial1.print("{");
  Serial1.print(seriesName);
  Serial1.print(",T,");
  Serial1.print(data);
  Serial1.println("}");
}

void debugLoopTime(){
 Serial.print("\tLT: ");
 Serial.print(millis()-StartL); 
}

void debugISE(){
 Serial.print("\tISTE: ");
 Serial.print(ISTE); 
}

void debugSensorValues() {
 Serial.print("\tfirroll: ");
 printFloat(roll, 5);
 Serial.println();
 }
 
 void debugAnglePID() {
 Serial.print("\tanglePID I: ");
 printFloat(anglePIDInput, 4);
 Serial.print("\tO: ");
 printFloat(anglePIDOutput, 4);
 Serial.print("\tS: ");
 printFloat(anglePIDSetpoint, 4);
 }

 void debugSpeedPID() {
 Serial.print("\tspeedPID I: ");
  Serial.print(speedPIDInput, 2);
 Serial.print("\tO: ");
  Serial.print(speedPIDOutput, 2);
 Serial.print("\tS: ");
  Serial.print(speedPIDSetpoint, 2);
 }
 
  void debugSpeedPIDCoeff() {
 Serial.print("\tanglePID Ki: ");
  Serial.print(configuration.speedPIDKp, 2);
 Serial.print("\tKi: ");
  Serial.print(configuration.speedPIDKi, 2);
 Serial.print("\tKd: ");
  Serial.print(configuration.speedPIDKd, 2);
 }
 
  void debugAnglePIDCoeff() {
 Serial.print("\tanglePID Ki: ");
  Serial.print(configuration.anglePIDConKp, 2);
 Serial.print("\tKi: ");
  Serial.print(configuration.anglePIDConKi, 2);
 Serial.print("\tKd: ");
  Serial.print(configuration.anglePIDConKd, 2);
 }
 

 void debugMotorCalibrations() {
 Serial.print("\tm1 C: ");
 printFloat(motor1Calibration, 4);
 Serial.print("\tm2 C: ");
 printFloat(motor2Calibration, 4);
 }
 void debugMotorSpeeds() {
 Serial.print("\tlmS: ");
 printFloat(leftMotorSpeed, 4);
 Serial.print("\trmS: ");
 printFloat(rightMotorSpeed, 4);
 }
 
 void debugTorque() {
 Serial.print("t1: ");
 printFloat(leftMotorSpeed, 4);
 Serial.print("t2:: ");
 printFloat(rightMotorSpeed, 4);
 }
 
void  debugSteerThrottle() {
   Serial.print("T: ");
   printFloat(configuration.throttleGain * UserControl[1], 4);
   Serial.print("S: ");
   printFloat(configuration.steerGain * UserControl[0], 4);
 }
 
 void debugImu() {
 Serial.print("aX: ");
 printInt(imuValues[0], 4);
 Serial.print("\taY: ");
 printInt(imuValues[1], 4);
 Serial.print("\taZ: ");
 printInt(imuValues[2], 4);
 Serial.print("\tgX: ");
 printInt(imuValues[3], 4);
 Serial.print("\tgY: ");
 printInt(imuValues[4], 4);
 Serial.print("\tgZ: ");
 printInt(imuValues[5], 4);
 }
 
 void debugMotorSpeedCalibration() {
 Serial.print("\t");
 Serial.print(anglePIDOutput*motor1Calibration);
 Serial.print("\t");
 Serial.print(anglePIDOutput*motor2Calibration);
 Serial.print("\t");
 Serial.print(rightMotorSpeed);
 Serial.print("\t");
 Serial.print(leftMotorSpeed);
 }
 
  
 void unrecognizedCMD() {
 Serial.print("\t");
 Serial.print("Command unknown");
 }
 
 void printInt(int number, byte width) {
 int currentMax = 10;
 if (number < 0) 
 currentMax = 1;
 for (byte i=1; i<width; i++){
 if (fabs(number) < currentMax) {
 Serial.print(" ");
 }
 currentMax *= 10;
 }
 Serial.print(number);
 }
 void printFloat(float number, byte width) {
 int currentMax = 10;
 if (number < 0) 
 currentMax = 1;
 for (byte i=1; i<width; i++){
 if (fabs(number) < currentMax) {
 Serial.print(" ");
 }
 currentMax *= 10;
 }
 Serial.print(number);
 }
 
 void debugConfiguration() {
 Serial.print("speedPIDKp: ");
 Serial.println(configuration.speedPIDKp);
 Serial.print("speedPIDKi: ");
 Serial.println(configuration.speedPIDKi);
 Serial.print("speedPIDKd: ");
 Serial.println(configuration.speedPIDKd);
 
 Serial.print("speedPidOutputLowerLimit: ");
 Serial.println(configuration.speedPIDOutputLowerLimit);
 Serial.print("speedPidOutputHigherLimit: ");
 Serial.println(configuration.speedPIDOutputHigherLimit);
 
 Serial.print("anglePIDAggKp: ");
 Serial.println(configuration.anglePIDAggKp);
 Serial.print("anglePIDAggKi: ");
 Serial.println(configuration.anglePIDAggKi);
 Serial.print("anglePIDAggKd: ");
 Serial.println(configuration.anglePIDAggKd);
 
 Serial.print("anglePIDConKp: ");
 Serial.println(configuration.anglePIDConKp);
 Serial.print("anglePIDConKi: ");
 Serial.println(configuration.anglePIDConKi);
 Serial.print("anglePIDConKd: ");
 Serial.println(configuration.anglePIDConKd);
 
 Serial.print("anglePIDLowerLimit: ");
 Serial.println(configuration.anglePIDOutputLowerLimit);
 
 Serial.print("anglePIDSampling: ");
 Serial.println(configuration.anglePIDSampling);
 Serial.print("motorsPIDSampling: ");
 Serial.println(configuration.speedPIDSampling);
 Serial.print("angleSensorSampling: ");
 Serial.println(configuration.angleSensorSampling);
 Serial.print("motorSpeedSensorSampling: ");
 Serial.println(configuration.motorSpeedSensorSampling);
 
  Serial.print("speedKalmanFilterR: ");
 Serial.println(configuration.speedKalmanFilterR);
 Serial.print("motorLeftMinimumSpeed: ");
 Serial.println(configuration.motorLeftMinimumSpeed);
 Serial.print("motorRightMinimumSpeed: ");
 Serial.println(configuration.motorRightMinimumSpeed);
 Serial.print("debugLevel: ");
 Serial.println(configuration.debugLevel);
 
   Serial.print("debugSampleRate: ");
 Serial.println(configuration.debugSampleRate);
 //Serial.print("speedPIDOutputDebug: ");

 }

