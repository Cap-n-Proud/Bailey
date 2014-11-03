//SCMD SerialDebug_ON
//SCMD Motors_ON

void RemoteRead(){
  SCmd.readSerial();     // We don't do much, just process serial commands 
}


 void printCommand() {
	char *arg = SCmd.next();

	
 }
 

void RemoteUpload()
{
  char *arg = SCmd.next();
  String line = "";
  if (!configuration.debug){      
          balanceKalmanFilter.correct(dISTE);
          line = pitch + SEPARATOR
                 + roll + SEPARATOR
                 + yaw + SEPARATOR
                 + LoopT + SEPARATOR
                 + anglePIDOutput + SEPARATOR
                 + leftMotorSpeed + SEPARATOR
                 + rightMotorSpeed) + SEPARATOR
                 + (dISTE) + SEPARATOR
                 + (balanceKalmanFilter.getState()*(abs(leftMotorSpeed)+abs(rightMotorSpeed))/2) + SEPARATOR
                 + (configuration.speedPIDKp * 10000) + SEPARATOR
                 + (configuration.speedPIDKi * 10000) + SEPARATOR
                 + (configuration.speedPIDKd * 10000) + SEPARATOR
                 + (configuration.anglePIDConKp * 100) + SEPARATOR
                 + (configuration.anglePIDConKi * 100) + SEPARATOR
                 + (configuration.anglePIDConKd * 100) + SEPARATOR
                 + (configuration.anglePIDAggKp * 100) + SEPARATOR
                 + (configuration.anglePIDAggKi * 100) + SEPARATOR
                 + (configuration.anglePIDAggKd * 100) + SEPARATOR
                 + n(configuration.TriggerAngleAggressive*100) + SEPARATOR
                 + (configuration.calibratedZeroAngle*100);
        Serial.println(line);
}

}
void RemoteInit()
{
 
            Serial.print("READ Read_SPIDKp ");Serial.println(configuration.speedPIDKp * 10000);
            Serial.print("READ Read_SPIDKi ");Serial.println(configuration.speedPIDKi * 10000);
            Serial.print("READ Read_SPIDKd ");Serial.println(configuration.speedPIDKd * 10000);
            Serial.print("READ Read_APIDKp ");Serial.println(configuration.anglePIDConKp * 100);
            Serial.print("READ Read_APIDKi ");Serial.println((float)configuration.anglePIDConKi * 100);
            Serial.print("READ Read_APIDKd ");Serial.println(configuration.anglePIDConKd * 100);
            Serial.print("READ Read_APIDAggKp ");Serial.println(configuration.anglePIDAggKp * 100);
            Serial.print("READ Read_APIDAggKi ");Serial.println((float)configuration.anglePIDAggKi * 100);
            Serial.print("READ Read_APIDAggKd ");Serial.println(configuration.anglePIDAggKd * 100);
            Serial.print("READ Read_TriggerAngleAggressive ");Serial.println(configuration.TriggerAngleAggressive*100);
            Serial.print("READ Read_calibratedZeroAngle ");Serial.println(configuration.calibratedZeroAngle*100);
            Serial.print("READ FirmwareVersion ");Serial.println(configuration.FirmwareVersion);
            LastEvent = "System ready";
  
}

void setCommand() 
{
	char *arg = SCmd.next();
	char *value = SCmd.next();
	if (*value != NULL) 
      {
		// parameters
		if (String("SPIDKp").equals(arg))
		configuration.speedPIDKp = atof(value)/10000;                  
                else if (String("SPIDKi").equals(arg))
		configuration.speedPIDKi = atof(value)/10000;
		else if (String("SPIDKd").equals(arg))
		configuration.speedPIDKd = atof(value)/10000;
		else if (String("speedPIDOutputLowerLimit").equals(arg))
		configuration.speedPIDOutputLowerLimit = atof(value);
		else if (String("speedPIDOutputHigherLimit").equals(arg))
		configuration.speedPIDOutputHigherLimit = atof(value);
		else if (String("APIDAggKp").equals(arg))
		configuration.anglePIDAggKp = atof(value)/100;
		else if (String("APIDAggKi").equals(arg))
		configuration.anglePIDAggKi = atof(value)/100;
		else if (String("APIDAggKd").equals(arg))
		configuration.anglePIDAggKd = atof(value)/100;
		else if (String("APIDConKp").equals(arg))
		configuration.anglePIDConKp = atof(value)/100;
                else if (String("APIDConKi").equals(arg))
		configuration.anglePIDConKi = atof(value)/100;
                else if (String("APIDConKd").equals(arg))
		configuration.anglePIDConKd = atof(value)/100;
                else if (String("TriggerAngleAggressive").equals(arg))
		configuration.anglePIDConKd = atof(value)/100;
                else if (String("calibratedZeroAngle").equals(arg))
		configuration.calibratedZeroAngle = atof(value)/100;
                else if (String("Motors").equals(arg)){
                  configuration.motorsON = atoi(value);
                  if (atoi(value)==0)
                    {
                    motorLeft.setSpeed(0);
                    motorRight.setSpeed(0); 
                    }
                    
                  }
          
          
                else if (String("SerialDebug").equals(arg)){
                     configuration.debug = atoi(value); 
                  }                  
                else if (String("AUTOTUNE").equals(arg)){
                     AUTOTUNE = atof(value); 
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
		configuration.motorLeftMinimumSpeed = atoi(value);
		else if (String("motorRightMinimumSpeed").equals(arg))
		configuration.motorRightMinimumSpeed = atoi(value);
		
                //EEPROM
                else if (String("E_EEPROM").equals(arg)){
                    for (int i = 0; i < 512; i++)
                      EEPROM.write(i, 255);
                     LastEvent= "EEPROM Erased/n";
                     loadConfig();
                     RemoteInit();
                     
                     
                  }
                 else if (String("SaveCfg").equals(arg)){
                     saveConfig(); 
                     LastEvent= "Config Saved to EEPROM/n";
                  }
                 else if (String("Load_def").equals(arg)){
                     setConfiguration((boolean) int(value));
                      LastEvent= "Defaul config loaded/n";
                  }
				
		// steering
		else if (String("SetsteerGain").equals(arg))
                configuration.steerGain = atoi(value)/100;
		else if (String("SetthrottelGain").equals(arg))
                configuration.steerGain = atoi(value)/100;
		else if (String("Steer").equals(arg))
                UserControl[0] = min(atoi(value), configuration.Maxsteer);              
		else if (String("Throttle").equals(arg))
                UserControl[1] = min(atoi(value), configuration.Maxthrottle = 50);
  		else if (String("speedPIDOutputDebug").equals(arg))
		configuration.speedPIDOutputDebug = atoi(value);
		else if (String("speedPIDInputDebug").equals(arg))
		configuration.speedPIDInputDebug= atoi(value);
		else if (String("speedKalmanFilterDebug").equals(arg))
		configuration.speedKalmanFilterDebug= atoi(value);
		else if (String("speedRawDebug").equals(arg))
		configuration.speedRawDebug= atoi(value);
		else if (String("speedMovingAvarageFilter2Debug").equals(arg))
		configuration.speedMovingAvarageFilter2Debug= atoi(value);
		else if (String("anglePIDSetpointDebug").equals(arg))
		configuration.anglePIDSetpointDebug= atoi(value);
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
		
                else if (String("printConfig").equals(arg)) {
		Serial.println("*** Configuration");
		Serial.println("*** --------------------------------------------------");
		Serial.print("*** configuration.speedPIDKp = ");
		Serial.print(configuration.speedPIDKp);
		Serial.print(";\n*** configuration.speedPIDKi = ");
		Serial.print(configuration.speedPIDKi);
		Serial.print(";\n*** configuration.speedPIDKd = ");
		Serial.print(configuration.speedPIDKd,4);
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

            else {
                  Serial.println("--------------------------------------------------");
                  Serial.print("Unknown command ");
                  Serial.println(arg);
                  }



RemoteInit();
controlConfig();
}



}


