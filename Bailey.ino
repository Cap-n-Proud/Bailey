#include <FreeSixIMU.h> // imu. www.varesano.net/projects/hardware/FreeIMU#library
#include <FIMU_ADXL345.h> // imu
#include <FIMU_ITG3200.h> // imu
#include <Wire.h> // for i2c
//#include <SoftwareSerial.h>
#include <PID_v1.h> //github.com/mwoodward/Arduino-PID-Library
#include <Button.h>   //github.com/JChristensen/Button 
#include <TimedAction.h> // for updating sensors and debug
#include <EEPROM.h> // for storing configuraion
#include <avr/wdt.h> // watchdog
#include <MovingAvarageFilter.h> //github.com/sebnil/Moving-Avarage-Filter--Arduino-Library-
#include <FIR.h> // github.com/sebnil/FIR-filter-Arduino-Library
#include <KalmanFilter.h> // github.com/nut-code-monkey/KalmanFilter-for-Arduino

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h> //https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <L29x.h>

#define SERIAL_BAUD 38400
#define CONFIG_START 32
#define SERIALCOMMAND_HARDWAREONLY
#include <SerialCommand.h> //https://github.com/scogswell/ArduinoSerialCommand

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi
#define speedMultiplier 1
#define LCDSerial Serial1 // 18 (TX);

L29x motorRight(8, 9, 10); // enable (PWM), motor pin 1, motor pin 2
L29x motorLeft(11, 12, 13);
// enable (PWM), motor pin 1, motor pin 2

//Button motorBtn(13, false, false, 20);
const uint8_t LED_PIN = 13;
String SEPARATOR = ","; //Used as separator for telemetry

// Tell it where to store your config data in EEPROM
boolean LCD_Output = false;
int debug = 0;

/* Configutation parameters */
struct Configuration {
  String FirmwareVersion;
  double speedPIDKp;
  double speedPIDKi;
  double speedPIDKd;
  double speedPIDOutputLowerLimit;
  double speedPIDOutputHigherLimit;
  double anglePIDAggKp;
  double anglePIDAggKi;
  double anglePIDAggKd;
  double anglePIDConKp;
  double anglePIDConKi;
  double anglePIDConKd;
  double anglePIDOutputLowerLimit;
  double anglePIDOutputHigherLimit;
  double TriggerAngleAggressive;
  double calibratedZeroAngle;
  uint8_t anglePIDSampling;
  double angleKalmanFilterR;
  uint8_t speedPIDSampling;
  uint8_t angleSensorSampling;
  uint8_t motorSpeedSensorSampling;
  double speedKalmanFilterR;
  uint8_t motorLeftMinimumSpeed;
  uint8_t motorRightMinimumSpeed;
  int motorsON;
  boolean debug;
  double steerGain;
  double throttleGain;
  double Maxsteer;
  double Maxthrottle;
  //int speakerPin;
  uint8_t debugLevel;
  uint8_t debugSampleRate;
  uint8_t speedPIDOutputDebug;
  uint8_t speedPIDInputDebug;
  uint8_t speedKalmanFilterDebug;
  uint8_t speedRawDebug;
  uint8_t speedMovingAvarageFilter2Debug;
  uint8_t anglePIDSetpointDebug;
  uint8_t anglePIDInputDebug;
  uint8_t anglePIDOutputDebug;
  uint8_t angleRawDebug;
  uint8_t  activePIDTuningDebug;
};
Configuration configuration;
byte b[sizeof(Configuration)];

double UserControl[1]; //Steer, Throttle
float Thr = 0;

void setConfiguration(boolean force) {
  /* Flash is erased every time new code is uploaded. Write the default configuration to flash if first time */
  // running for the first time?
  uint8_t codeRunningForTheFirstTime = EEPROM.read(CONFIG_START); // flash bytes will be 255 at first run
  if (codeRunningForTheFirstTime || force) {
    if (configuration.debug){
      Serial.print("No config found, defaulting ");
    }
    /* First time running, set defaults */
    configuration.FirmwareVersion = "0.9";
    configuration.speedPIDOutputLowerLimit = -10.00; //Default was -5
    configuration.speedPIDOutputHigherLimit = 10.00;
    configuration.anglePIDAggKp = 10.70;
    configuration.anglePIDAggKi = 2.5;
    configuration.anglePIDAggKd = 5.51;
    configuration.anglePIDConKp = 4.80;
    configuration.anglePIDConKi = 2.21;
    configuration.anglePIDConKd = 4.75;
    configuration.speedPIDKp = 0.3854;
    configuration.speedPIDKi = 0.1374; //0.0051
    configuration.speedPIDKd = 0.00245;
    configuration.anglePIDOutputLowerLimit = -100;
    configuration.anglePIDOutputHigherLimit = 100;
    
    configuration.motorLeftMinimumSpeed = 55;//40;
    configuration.motorRightMinimumSpeed = 58;//44;
    configuration.steerGain = 1;
    configuration.throttleGain = 1;
    configuration.Maxsteer = 10; //Max allowed percentage difference. Up to the remote to provide the right scale.  
    configuration.Maxthrottle = 3; //Max speed expressed in inclination degrees. Up to the remote to provide the right scale.
    
    configuration.motorsON = 0;
    configuration.debug = 0;
  
    configuration.TriggerAngleAggressive = 3.50;
    configuration.calibratedZeroAngle = 1.8;
    
    configuration.anglePIDSampling = 10;
    configuration.speedPIDSampling = 10;
  
    configuration.angleKalmanFilterR = 10.00;
    configuration.angleSensorSampling = 5;
    configuration.motorSpeedSensorSampling = 5;
    configuration.speedKalmanFilterR = 20.00;
      
    configuration.debugLevel = 0;
    configuration.debugSampleRate = 50;
    //  configuration.speedPIDSetpointDebug = 1;
    configuration.speedPIDOutputDebug = 1;
    configuration.speedPIDInputDebug = 1;
    configuration.speedKalmanFilterDebug = 1;
    configuration.speedRawDebug = 1;
    configuration.speedMovingAvarageFilter2Debug = 0;
    configuration.anglePIDSetpointDebug = 1;
    configuration.anglePIDInputDebug = 1;
    configuration.anglePIDOutputDebug = 1;
    configuration.angleRawDebug = 1;
    configuration.activePIDTuningDebug = 1;
    //configuration.speakerPin = 13;
    
    saveConfig();
    delay(100);
  }
  else {
    if (debug)
      Serial.println("Config found");
    loadConfig();
  }
};

/* Encoders */

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
//#define ENCODER_OPTIMIZE_INTERRUPTS

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
#define leftEncoder1 2
#define leftEncoder2 4
#define rightEncoder1 3
#define rightEncoder2 5

Encoder MotorLeft(leftEncoder1, leftEncoder2);
Encoder MotorRight(rightEncoder1, rightEncoder2);

long lastLeftMotorPosition  = 0;
long lastRightMotorPosition  = 0;
volatile long leftMotorPosition  = 0;
volatile long rightMotorPosition = 0;

// motor speeds and calibrations
float motorSpeed;
float leftMotorSpeed;
float rightMotorSpeed;

float speedKalmanFiltered = 0;
float speedFIRFiltered = 0;

// These take care of the timing of things
TimedAction debugTimedAction = TimedAction(1000, debugEverything); //Print debug info
TimedAction updateMotorStatusesTimedAction = TimedAction(20, updateMotorSpeeds); //
TimedAction updateIMUSensorsTimedAction = TimedAction(20, updateIMUSensors);

//TimedAction remoteControlWatchdogTimedAction = TimedAction(5000, stopRobot);

//Reads serial for commands
TimedAction RemoteReadTimedAction = TimedAction(250, RemoteRead);

//Upload telemetry data
TimedAction RemoteUploadTimedAction = TimedAction(250, RemoteUpload);

//Swarn Particle Optimization
TimedAction SwarnTimedAction = TimedAction(2000, SPO);

SerialCommand SCmd;   // The SerialCommand object

// imu variables
float imuValues[6];
float ypr[3];
float pitch, roll, yaw;
long StartL=0, LoopT=0, StartL2=0;

String LastEvent="";

// PID variables
double anglePIDSetpoint, anglePIDInput, anglePIDOutput;
double speedPIDInput, speedPIDOutput, speedPIDSetpoint;
double anglePIDInputFiltered;
double ISTE = 0, dISTE = 0;
int  AUTOTUNE = 0;

// filters
MovingAvarageFilter speedMovingAvarageFilter(14); 
MovingAvarageFilter angleMovingAvarageFilter(4);
KalmanFilter speedKalmanFilter;
KalmanFilter angleKalmanFilter;
KalmanFilter balanceKalmanFilter;
FIR speedMovingAvarageFilter2;

// The cascading PIDs. The tunings are updated from the remote
PID anglePID(&anglePIDInput, &anglePIDOutput, &anglePIDSetpoint, 0, 0, 0, DIRECT); //REVERSE???
PID speedPID(&speedPIDInput, &speedPIDOutput, &speedPIDSetpoint, 0, 0, 0, DIRECT);

// Set the FreeSixIMU object. This handles the communcation to the IMU.
FreeSixIMU sixDOF = FreeSixIMU();

//char notes[] = "ccggaagffeeddc "; // a space represents a rest
//int beats[] = { 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 4 };
//int tempo = 300;
 TimedAction LCDUpdateTimedAction = TimedAction(100, LCDUpdate);

void setup() { 


  //pinMode(configuration.speakerPin, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(50);
  //Serial.println("connection estabilished");
  if (LCD_Output)
  {
    LCDSerial.begin(9600); // set up serial port for 9600 baud
    delay(500); // wait for display to boot up
    backlightOn(0); 
    cursorSet(0,0);
    clearLCD();  
    LCDSerial.print("Init. sequence");
  }

  // Load config from eeprom
  setConfiguration(false);
  // init i2c and IMU
  delay(100);
  Wire.begin();

  //Init control systems
  controlConfig();
  //Init the timers
  initTimedActions();

  // filters
  speedKalmanFilter.setState(0);
  balanceKalmanFilter.setState(0);
  float coefficients[] = {
    1,1,1,1,1,1,1,1,1              };
  speedMovingAvarageFilter2.setCoefficients(coefficients);
  //speedMovingAvarageFilter2.setNumberOfTaps(9);
  speedMovingAvarageFilter2.setGain((float) (1/9));

  //Serial.println("IMU...");
  delay(50);
  sixDOF.init(); //begin the IMU
  delay(150);
  //Serial.println("OK...");
  if (LCD_Output){
    clearLCD(); 
    LCDSerial.print(" IMU");  
  }


  if (debug==1)
    debugConfiguration();

  //saveConfig();
  // set the watchdog to 2s (this will restart the arduino if it freezes)
  wdt_enable(WDTO_2S);
  
  swarnInit();
  LastEvent = "Swarn ready";
  RemoteInit();
  
  // Setup callbacks for SerialCommand commands 
  SCmd.addCommand("SCMD", setCommand);       
  SCmd.addCommand("READ", printCommand);     
  //play(notes, beats);
}

void updateIMUSensors() {
  double angleT;
  sixDOF.getYawPitchRoll(ypr);
  roll = ypr[1];
  pitch = ypr[2];
  yaw = ypr[0];
  angleT = pitch; 
  // move angle to around equilibrium
  angleKalmanFilter.correct(angleT);
  anglePIDInput = angleKalmanFilter.getState() - configuration.calibratedZeroAngle;// + configuration.throttleGain*UserControl[1]; //Need to add the input throttle 
}


void loop() { 
  wdt_reset();
  StartL = millis();
  
  if (LCD_Output){
   LCDUpdateTimedAction.check();
  }

  // update sensors and motors, also chek commands and send back telemetry
  updateIMUSensorsTimedAction.check();
  updateMotorStatusesTimedAction.check();
  RemoteReadTimedAction.check();
  RemoteUploadTimedAction.check();
  
  if (AUTOTUNE==1) {
   SwarnTimedAction.check();    
  // remoteControlWatchdogTimedAction.check();
  }
  

    // Speed pid,  input is wheel speed, output is angleSetpoint
    speedPIDSetpoint =  configuration.Maxthrottle*UserControl[1];
    speedPID.Compute();
    anglePIDSetpoint = - speedPIDOutput;

    // Update angle pid tuning
   if(abs(anglePIDInput) < (float)configuration.TriggerAngleAggressive && configuration.TriggerAngleAggressive != 0) { 
      //we're close to setpoint, use conservative tuning parameters
      anglePID.SetTunings((float)configuration.anglePIDConKp, (float)configuration.anglePIDConKi, (float)configuration.anglePIDConKd);
    }
    else if (abs(anglePIDInput) >  (float)configuration.TriggerAngleAggressive && abs(anglePIDInput) <= 30 ) {
      //we're far from setpoint, use aggressive tuning parameters
      anglePID.SetTunings((float)configuration.anglePIDAggKp, (float)configuration.anglePIDAggKi, (float)configuration.anglePIDAggKd);
    }
    else
    {
      anglePID.SetTunings(0, 0, 0);
    }

    anglePID.Compute();
    //If we are very close to the target we stop the motors to avoid overheating the motor driver
    //angleKalmanFilter.correct(anglePIDOutput);
    //anglePIDOutput = angleKalmanFilter.getState();
    if (configuration.motorsON==1){
      motorLeft.setSpeedPercentage(-anglePIDOutput - configuration.steerGain * (UserControl[0]));// + configuration.throttleGain*UserControl[1]);
      motorRight.setSpeedPercentage(-anglePIDOutput + configuration.steerGain * (UserControl[0]));// + configuration.throttleGain*UserControl[1]);
    }

  else{
    motorLeft.setSpeed(0);
    motorRight.setSpeed(0); 
  }
  if (configuration.debug == 1 )
    debugTimedAction.check();
  
  LoopT = millis()-StartL;
  dISTE = (LoopT*(anglePIDSetpoint - configuration.calibratedZeroAngle -pitch)*(anglePIDSetpoint - configuration.calibratedZeroAngle -pitch))/1000;  
  ISTE = ISTE + dISTE;
}

/* just debug functions. uncomment the debug information you want in debugEverything */
void debugEverything() {
  //debugImu();
  //debugAnglePID();
  //debugSpeedPID();
  debugISE();
  //debugAnglePIDCoeff();
  //debugSpeedPIDCoeff();
  debugEncoders();
  debugMotorSpeeds();
  //debugMotorCalibrations();
  //debugMotorSpeedCalibration();
  //debugChart2();
  //unrecognizedCMD();
  debugLoopTime();
  Serial.println();

};

void LCDUpdate() {
  clearLCD(); 
  cursorSet(0,0);
  LCDSerial.print(configuration.speedPIDKp,1);
  cursorSet(4,0);
  LCDSerial.print((float)configuration.speedPIDKi,2);
  cursorSet(8,0); 
  LCDSerial.print((float)configuration.speedPIDKd,2);
  cursorSet(13,0);
  LCDSerial.print(pitch,1);

}

