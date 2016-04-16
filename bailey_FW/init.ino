

void controlConfig() {
  // init speed PID
  speedPIDSetpoint = 0;
  speedPID.SetOutputLimits(configuration.speedPIDOutputLowerLimit, configuration.speedPIDOutputHigherLimit);
  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(configuration.speedPIDSampling);
  speedPID.SetTunings(configuration.speedPIDKp, configuration.speedPIDKi, configuration.speedPIDKd);

  //init angle PID
  anglePIDSetpoint = 0;
  anglePID.SetOutputLimits(configuration.anglePIDOutputLowerLimit, configuration.anglePIDOutputHigherLimit);
  anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(configuration.anglePIDSampling);
  anglePID.SetTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);

  //angleKalmanFilter.setR(configuration.angleKalmanFilterR);
  //speedKalmanFilter.setR(configuration.speedKalmanFilterR);

  // init motors
  //motorLeft.setMinimumSpeed(configuration.motorLeftMinimumSpeed);
  //motorRight.setMinimumSpeed(configuration.motorRightMinimumSpeed);
}






