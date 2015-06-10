void initTimedActions() {
// These take care of the timing of things
TimedAction debugTimedAction = TimedAction(1000, debugEverything); //Print debug info
TimedAction updateMotorStatusesTimedAction = TimedAction(configuration.motorSpeedSensorSampling, updateMotorSpeeds); //
TimedAction updateIMUSensorsTimedAction = TimedAction(configuration.angleSensorSampling, updateIMUSensors);

//TimedAction remoteControlWatchdogTimedAction = TimedAction(5000, stopRobot);

//Reads serial for commands
TimedAction RemoteReadTimedAction = TimedAction(250, RemoteRead);

//Upload telemetry data
TimedAction TelemetryTXTimedAction = TimedAction(250, TelemetryTX);

//Swarn Particle Optimization
TimedAction SwarnTimedAction = TimedAction(configuration.SPOConfigEval, SPO);

}

void controlConfig() {
  // init speed PID
  speedPIDSetpoint = 0;
  speedPID.SetOutputLimits(configuration.speedPIDOutputLowerLimit, configuration.speedPIDOutputHigherLimit);
  speedPID.SetMode(AUTOMATIC);
  speedPID.SetSampleTime(configuration.speedPIDSampling);
  speedPID.SetTunings(configuration.speedPIDKp, configuration.speedPIDKi, configuration.speedPIDKd);

  //init angle PID
  anglePIDSetpoint = configuration.calibratedZeroAngle; //Originally was zero 0;
  anglePID.SetOutputLimits(configuration.anglePIDOutputLowerLimit, configuration.anglePIDOutputHigherLimit);
  anglePID.SetMode(AUTOMATIC);
  anglePID.SetSampleTime(configuration.anglePIDSampling);
  anglePID.SetTunings(configuration.anglePIDConKp, configuration.anglePIDConKi, configuration.anglePIDConKd);

  //angleKalmanFilter.setR(configuration.angleKalmanFilterR);
  //speedKalmanFilter.setR(configuration.speedKalmanFilterR);
  
  // init motors
  motorLeft.setMinimumSpeed(configuration.motorLeftMinimumSpeed);
  motorRight.setMinimumSpeed(configuration.motorRightMinimumSpeed);
}






