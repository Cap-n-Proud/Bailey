//SCMD SerialDebug_ON
//SCMD Motors_ON

void RemoteRead() {
  SCmd.readSerial();     // We don't do much, just process serial commands
}


//Triggered by READ command
void printCommand() {
  char *arg = SCmd.next();

  //SCMD PIDParamTX
  if (String("PIDParamTX").equals(arg))
    PIDParamTX();
  else if (String("RemoteInit").equals(arg))
    RemoteInit();
  else if (String("SYSParamTX").equals(arg))
    SYSParamTX();



  else if (String("printConfig").equals(arg)) {
    Serial.println("DebugCFG");
    Serial.println("*** Configuration");
    Serial.println("*** --------------------------------------------------");
    Serial.print("*** configuration.speedPIDKp = ");
    Serial.print(configuration.speedPIDKp);
    Serial.print(";\n*** configuration.speedPIDKi = ");
    Serial.print(configuration.speedPIDKi);
    Serial.print(";\n*** configuration.speedPIDKd = ");
    Serial.print(configuration.speedPIDKd, 4);
    Serial.print(";\n*** configuration.speedPIDOutputLowerLimit = ");
    Serial.print(configuration.speedPIDOutputLowerLimit);
    Serial.print(";\n*** configuration.speedPIDOutputHigherLimit = ");
    Serial.print(configuration.speedPIDOutputHigherLimit);
    Serial.print(";\n*** configuration.anglePIDAggKp = ");
    Serial.print(configuration.anglePIDAggKp);
    Serial.print(";\n*** configuration.anglePIDAggKi = ");
    Serial.print(configuration.anglePIDAggKi);
    Serial.print(";\n*** configuration.anglePIDAggKd = ");
    Serial.print(configuration.anglePIDAggKd);
    Serial.print(";\n*** configuration.anglePIDConKp = ");
    Serial.print(configuration.anglePIDConKp);
    Serial.print(";\n*** configuration.anglePIDConKi = ");
    Serial.print(configuration.anglePIDConKi);
    Serial.print(";\n*** configuration.anglePIDConKd = ");
    Serial.print(configuration.anglePIDConKd);
    Serial.print(";\n*** configuration.anglePIDLowerLimit = ");
    Serial.print(configuration.anglePIDOutputLowerLimit);
    Serial.print(";\n*** configuration.anglePIDHigherLimit = ");
    Serial.print(configuration.anglePIDOutputHigherLimit);
    Serial.print(";\n*** configuration.TriggerAngleAggressive = ");
    Serial.print(configuration.TriggerAngleAggressive);

    Serial.print(";\n*** configuration.calibratedZeroAngle = ");
    Serial.print(configuration.calibratedZeroAngle);
    Serial.print(";\n*** configuration.anglePIDSampling = ");
    Serial.print(configuration.anglePIDSampling);
    Serial.print(";\n*** configuration.speedPIDSampling = ");
    Serial.print(configuration.speedPIDSampling);
    Serial.print(";\n*** configuration.angleKalmanFilterR = ");
    Serial.print(configuration.angleKalmanFilterR);
    Serial.print(";\n*** configuration.angleSensorSampling = ");
    Serial.print(configuration.angleSensorSampling);
    Serial.print(";\n*** configuration.motorSpeedSensorSampling = ");
    Serial.print(configuration.motorSpeedSensorSampling);
    Serial.print(";\n*** configuration.speedKalmanFilterR = ");
    Serial.print(configuration.speedKalmanFilterR);
    Serial.print(";\n*** configuration.motorLeftMinimumSpeed = ");
    Serial.print(configuration.motorLeftMinimumSpeed);
    Serial.print(";\n*** configuration.motorRightMinimumSpeed = ");
    Serial.print(configuration.motorRightMinimumSpeed);
    Serial.println("*** --------------------------------------------------");

  }



}

void TelemetryTX()
{ // for help on dtostrf http://forum.arduino.cc/index.php?topic=85523.0

  String line = "";
  if (!configuration.debug) {
    //balanceKalmanFilter.correct(dISTE);
    char buffer[10];
    //if (AUTOTUNE == 1) {LastEvent = LastEventSPO;}
    //Need to calculate parameters here because the main loop has a different frequency
    pitchd = (pitch - prev_pitch);
    prev_pitch = pitch;
    //PrevTxLoopTime = TxLoopTime;
    TxLoopTime = millis()-TxLoopTime;
    
    dISTE = (TxLoopTime/1000*(anglePIDSetpoint - pitch)*(anglePIDSetpoint - pitch));  
    
    line = "T" + SEPARATOR
           + yaw + SEPARATOR
           + pitch + SEPARATOR
           + roll + SEPARATOR
           + pitchd + SEPARATOR
           + dISTE + SEPARATOR
           //+ anglePIDOutput + SEPARATOR
           + leftMotorSpeed + SEPARATOR
           + rightMotorSpeed + SEPARATOR
           + LoopT;
    //+ SEPARATOR
    //+ LastEvent;
    Serial.println(line);
  }

}

//Transmits the PID parmaeters. To be called upon initialization event and every time a parameter changes
void PIDParamTX()
{

  String line = "";
  if (!configuration.debug)
  {
    line = "PID" + SEPARATOR
           + int(configuration.speedPIDKp * 10000)  + SEPARATOR
           + int(configuration.speedPIDKi * 10000) + SEPARATOR
           + int(configuration.speedPIDKd * 10000) + SEPARATOR
           + int(configuration.anglePIDConKp * 100) + SEPARATOR
           + int(configuration.anglePIDConKi * 100) + SEPARATOR
           + int(configuration.anglePIDConKd * 100) + SEPARATOR
           + int(configuration.anglePIDAggKp * 100) + SEPARATOR
           + int(configuration.anglePIDAggKi * 100) + SEPARATOR
           + int(configuration.anglePIDAggKd * 100) + SEPARATOR
           + int(configuration.TriggerAngleAggressive * 100) + SEPARATOR
           + int(configuration.calibratedZeroAngle * 100) + SEPARATOR
           + int(5);
    Serial.println(line);
  }

}

//Transmits the system parmaeters. To be called upon initialization event and every time a parameter changes
void SYSParamTX()
{

  String line = "";
  if (!configuration.debug) {
    line = "SYS" + SEPARATOR
           + configuration.FirmwareVersion;

    Serial.println(line);
  }

}

void RemoteInit()
{
  //Initialze the headers for the various dictionaries
  //These MUST be in the same order as in RemoteUpload()
  //TH - Telemetry
  //PIDH - PID
  //SYSH - System
  String headers = "";
  headers = "TH" + SEPARATOR +
            "yaw" + SEPARATOR +
            "pitch" + SEPARATOR +
            "roll" + SEPARATOR +
            "pitchSpeed" + SEPARATOR +
            //"bal" + SEPARATOR +
            "dISTE" + SEPARATOR +
            //"anglePIDOutput" + SEPARATOR +
            "leftMotorSpeed" + SEPARATOR +
            "rightMotorSpeed" + SEPARATOR +
            "LoopT";
  //+ SEPARATOR +
  //"LastEvent";
  Serial.println(headers);
  delay(100);
  headers = "PIDH" + SEPARATOR +
            "speedPIDKp" + SEPARATOR +
            "speedPIDKi" + SEPARATOR +
            "speedPIDKd" + SEPARATOR +
            "anglePIDConKp" + SEPARATOR +
            "anglePIDConKi" + SEPARATOR +
            "anglePIDConKd" + SEPARATOR +
            "anglePIDAggKp" + SEPARATOR +
            "anglePIDAggKi" + SEPARATOR +
            "anglePIDAggKd" + SEPARATOR +
            "TriggerAngleAggressive" + SEPARATOR +
            "calibratedZeroAngle" + SEPARATOR +
            "OPT";
  Serial.println(headers);
  delay(100);
  headers = 	"SYSH" + SEPARATOR +
              "FirmwareVersion";
  Serial.println(headers);
}


void setCommand2()
{
  char *arg;
  char *value;
  arg = SCmd.next();
  value = SCmd.next();

  if (value != NULL)
  {
    if (String("test").equals(arg)) {
      delay(5);
      Serial.print("this is a test arg ");
      Serial.println(value);
    }
    else if (String("SKp").equals(arg)) {
      Serial.print("SPIDKp set to ");
      Serial.println(value);
    }






  }


}


void setCommand()
{
  char *arg;
  char *value;
  //String cmd = "";
  arg = SCmd.next();
  value = SCmd.next();
//This echoes the command back
    String cmd = String("SCMD " + String(arg) + " " + String(value));
    Serial.println(cmd);
 
    
  if (value != NULL)
  {
    // parameters
    if (String("speedPIDKp").equals(arg)) {
      configuration.speedPIDKp = atof(value) / 10000;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("test").equals(arg)) {
      Serial.print("this is a test arg ");
      Serial.println(value);
    }
    else if (String("speedPIDKi").equals(arg)) {
      configuration.speedPIDKi = atof(value) / 10000;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("speedPIDKd").equals(arg)) {
      configuration.speedPIDKd = atof(value) / 10000;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("speedPIDOutputLowerLimit").equals(arg)) {
      configuration.speedPIDOutputLowerLimit = atof(value);
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("speedPIDOutputHigherLimit").equals(arg)) {
      configuration.speedPIDOutputHigherLimit = atof(value);
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("anglePIDAggKp").equals(arg)) {
      configuration.anglePIDAggKp = atof(value) / 100;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("anglePIDAggKi").equals(arg)) {
      configuration.anglePIDAggKi = atof(value) / 100;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("anglePIDAggKd").equals(arg)) {
      configuration.anglePIDAggKd = atof(value) / 100;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("anglePIDConKp").equals(arg)) {
      configuration.anglePIDConKp = atof(value) / 100;
      delay(5);
      controlConfig();
      PIDParamTX();
      // Serial.println("-------------------- APIDConKp ------------------------------");

    }
    else if (String("anglePIDConKi").equals(arg)) {
      configuration.anglePIDConKi = atof(value) / 100;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("anglePIDConKd").equals(arg)) {
      configuration.anglePIDConKd = atof(value) / 100;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("TriggerAngleAggressive").equals(arg)) {
      configuration.anglePIDConKd = atof(value) / 100;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("calibratedZeroAngle").equals(arg)) {
      configuration.calibratedZeroAngle = atof(value) / 100;
      delay(5);
      controlConfig();
      PIDParamTX();
    }
    else if (String("Motors").equals(arg)) {
      configuration.motorsON = atoi(value);
      if (atoi(value) == 0)
      {
        motorLeft.setSpeed(0);
        motorRight.setSpeed(0);
      }

      //Serial.println("Motors CMD");
    }


    else if (String("SerialDebug").equals(arg)) {
      configuration.debug = atoi(value);
    }
    else if (String("AUTOTUNE").equals(arg)) {
      AUTOTUNE = atof(value);
      if (atoi(value) == 0)
      {
        LastEvent = "Autotune OFF";
      }
      else
        LastEvent = "Autotune ON";

    }
    else if (String("anglePIDLowerLimit").equals(arg))
      configuration.anglePIDOutputLowerLimit = atof(value);
    else if (String("anglePIDSampling").equals(arg))
      configuration.anglePIDOutputLowerLimit = atof(value);
    else if (String("speedPIDSampling").equals(arg))
      configuration.speedPIDSampling = atof(value);
    else if (String("angleKalmanFilterR").equals(arg))
      configuration.angleKalmanFilterR = atof(value);
    else if (String("angleSensorSampling").equals(arg))
      configuration.angleSensorSampling = atof(value);
    else if (String("motorSpeedSensorSampling").equals(arg))
      configuration.motorSpeedSensorSampling = atof(value);
    else if (String("speedKalmanFilterR").equals(arg))
      configuration.speedKalmanFilterR = atof(value);
    else if (String("motorLeftMinimumSpeed").equals(arg))
    {
      configuration.motorLeftMinimumSpeed = atoi(value);

      controlConfig();
    }
    else if (String("motorRightMinimumSpeed").equals(arg))
    {
      configuration.motorRightMinimumSpeed = atoi(value);
      controlConfig();
    }
    else if (String("torqueScale").equals(arg)) {
      //The idea here is to give one command to proportionally scale the minimum for both motors
      configuration.motorRightMinimumSpeed = atoi(value) * configuration.motorRightMinimumSpeed;
      configuration.motorLeftMinimumSpeed = atoi(value) * configuration.motorLeftMinimumSpeed;
    }
    //EEPROM
    else if (String("E_EEPROM").equals(arg)) {
      for (int i = 0; i < 512; i++)
        EEPROM.write(i, 255);
      LastEvent = "EEPROM Erased";
      loadConfig();
      RemoteInit();


    }
    else if (String("SaveCfg").equals(arg)) {
      saveConfig();
      LastEvent = "Config Saved to EEPROM";
    }
    else if (String("Load_def").equals(arg)) {
      setConfiguration((boolean) int(value));
      LastEvent = "Default config loaded";
    }

    // steering
    else if (String("SetsteerGain").equals(arg))
      configuration.steerGain = atoi(value) / 100;
    else if (String("SetthrottelGain").equals(arg))
      configuration.steerGain = atoi(value) / 100;
    else if (String("Steer").equals(arg))
    { //sign * value
      UserControl[0] = (atof(value) / 100) * configuration.Maxsteer; //((atof(value))/(atof(value))) * max(abs(atof(value)), configuration.Maxsteer);
      //Serial.println(UserControl[0]);
    }
    else if (String("Maxsteer").equals(arg))
    { //sign * value
      configuration.Maxsteer = atoi(value);
    }

    else if (String("Throttle").equals(arg))
    { //SCMD Throttle 50
      UserControl[1] =  (atof(value) / 100) * configuration.Maxthrottle; //((atof(value))/(atof(value))) * max(abs(atof(value)), configuration.Maxthrottle);
      //Serial.println(UserControl[1]);
    }
    else if (String("Maxthrottle").equals(arg))
    { //sign * value
      configuration.Maxthrottle = atoi(value);
    }
    else if (String("speedPIDOutputDebug").equals(arg))
      configuration.speedPIDOutputDebug = atoi(value);
    else if (String("speedPIDInputDebug").equals(arg))
      configuration.speedPIDInputDebug = atoi(value);
    else if (String("speedKalmanFilterDebug").equals(arg))
      configuration.speedKalmanFilterDebug = atoi(value);
    else if (String("speedRawDebug").equals(arg))
      configuration.speedRawDebug = atoi(value);
    else if (String("speedMovingAvarageFilter2Debug").equals(arg))
      configuration.speedMovingAvarageFilter2Debug = atoi(value);
    else if (String("anglePIDSetpointDebug").equals(arg))
      configuration.anglePIDSetpointDebug = atoi(value);
    else if (String("anglePIDInputDebug").equals(arg))
      configuration.anglePIDInputDebug = atoi(value);
    else if (String("anglePIDOutputDebug").equals(arg))
      configuration.anglePIDOutputDebug = atoi(value);
    else if (String("angleRawDebug").equals(arg))
      configuration.angleRawDebug = atoi(value);
    else if (String("debugLevel").equals(arg))
      configuration.debugLevel = atoi(value);
    else if (String("debugSampleRate").equals(arg))
      configuration.debugSampleRate = atoi(value);

    
    else {
      
      Serial.print("----------------");
      Serial.print("Unknown command ");
      Serial.print(arg);
      Serial.println(" ----------------");

    }

  }



}


void unrecognizedCMD() {

  Serial.print("--------------------- unrecognized command");

  Serial.println(" -----------------------------");

}

