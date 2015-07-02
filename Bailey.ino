
//------------------ Libraries ------------------


#include <FreeSixIMU.h> // http://bildr.org/2012/03/stable-orientation-digital-imu-6dof-arduino/
                         // Modified version from imu. www.varesano.net/projects/hardware/FreeIMU#library
                        // Try also: https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter.git 
//Do we need the following two? They might be included in the FreeSixIMU. Also check http://www.i2cdevlib.com/
#include <FIMU_ADXL345.h> // imu
#include <FIMU_ITG3200.h> // imu
//#include <MPU60X0.h>
//#include <MS561101BA.h>
//#include <HMC58X3.h>
//#include <DebugUtils.h>
   
#include <Wire.h> // for i2c
#include <SPI.h>

//#include <SoftwareSerial.h>
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library.git
#include <Button.h>   // github.com/JChristensen/Button 
#include <TimedAction.h> // for updating sensors and debug http://bit.ly/pATDBi http://playground.arduino.cc/Code/TimedAction
#include <EEPROM.h> // for storing configuraion
#include <avr/wdt.h> // watchdog http://savannah.nongnu.org/projects/avr-libc/
#include <MovingAvarageFilter.h> // github.com/sebnil/Moving-Avarage-Filter--Arduino-Library-
//#include <FIR.h> // github.com/sebnil/FIR-filter-Arduino-Library
#include <KalmanFilter.h> // github.com/nut-code-monkey/KalmanFilter-for-Arduino
                          // Try also: https://github.com/TKJElectronics/KalmanFilter.git

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h> // https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <L29x.h> // https://github.com/sebnil/L29x.git 

#define SERIALCOMMAND_HARDWAREONLY 1
#include <SerialCommand.h> // https://github.com/kroimon/Arduino-SerialCommand.git

//------------------ Constants ------------------ 
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi
#define speedMultiplier 1
//#define LCDSerial Serial1 // 18 (TX);
#define SERIAL_BAUD 38400
#define CONFIG_START 32 //EEPROM address to start the config

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
  uint8_t speedPIDSampling;
  
  double angleKalmanFilterR;
  uint8_t angleSensorSampling;
  uint8_t motorSpeedSensorSampling;
  double speedKalmanFilterR;
  
  int MotorLeftENABLEA;
  int MotorLeftIN1;
  int MotorLeftIN2;
  int MotorRightENABLEA;
  int MotorRightIN1;
  int MotorRightIN2;
  int leftEncoder1;
  int leftEncoder2;
  int rightEncoder1;
  int rightEncoder2;
  
  double steerGain;
  double throttleGain;
  double Maxsteer;
  double Maxthrottle;
  uint8_t motorLeftMinimumSpeed;
  uint8_t motorRightMinimumSpeed;
  int motorsON;
  
  int numParticles;
  double maxInteractions;
  int SPOConfigEval;
  boolean debugSPO;
  double SPOspread = 0.10;
  
  int commandDelay;
  //int speakerPin;
  
  boolean debug;
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
// float Thr = 0; //CHECK IF NEEDED

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
    configuration.anglePIDConKd = 0.475;
    configuration.speedPIDKp = 0.3854;
    configuration.speedPIDKi = 0.1374; //0.0051
    configuration.speedPIDKd = 0.00245;
    configuration.anglePIDOutputLowerLimit = -100;
    configuration.anglePIDOutputHigherLimit = 100;
    
    configuration.MotorLeftENABLEA = 8;
    configuration.MotorLeftIN1 = 9;
    configuration.MotorLeftIN2 = 10;
    configuration.MotorRightENABLEA = 11;
    configuration.MotorRightIN1 = 12;
    configuration.MotorRightIN2 =13;
    configuration.leftEncoder1 = 2;
    configuration.leftEncoder2 = 4;
    configuration.rightEncoder1 = 3;
    configuration.rightEncoder2 = 5;
  
    configuration.motorLeftMinimumSpeed = 55;//40;
    configuration.motorRightMinimumSpeed = 58;//44;
    configuration.steerGain = 1;
    configuration.throttleGain = 1;
    configuration.Maxsteer = 10; //Max allowed percentage difference. Up to the remote to provide the right scale.  
    configuration.Maxthrottle = 1.5; //Max speed expressed in inclination degrees. Up to the remote to provide the right scale.
        
    configuration.motorsON = 0;
    configuration.debug = 0;
  
    configuration.TriggerAngleAggressive = 2.50;
    configuration.calibratedZeroAngle = 1.8;
    
    configuration.anglePIDSampling = 10;
    configuration.speedPIDSampling = 10;
  
    configuration.angleKalmanFilterR = 10.00;
    configuration.angleSensorSampling = 5;
    configuration.motorSpeedSensorSampling = 5;
    configuration.speedKalmanFilterR = 20.00;
  
    configuration.maxInteractions = 30;
    configuration.numParticles = 20;
    configuration.SPOConfigEval = 2000;
    configuration.debugSPO = false;
    //Define a percentage around the known stable values
    configuration.SPOspread = 0.10;
    
    configuration.commandDelay = 5;
    configuration.debugLevel = 0;
    configuration.debugSampleRate = 1000;
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
    if (configuration.debug)
      Serial.println("Config found");
    loadConfig();
  }
};


//------------------ Definitions ------------------ 
//L29x motorRight((int)configuration.MotorLeftENABLEA , (int)configuration.MotorLeftIN1, (int)configuration.MotorLeftIN2); // pin8 PWM=EnableA, pin 9 is IN1, pin 10 is IN2
//L29x motorLeft((int)configuration.MotorRightENABLEA , (int)configuration.MotorRightIN1 , (int)configuration.MotorRightIN2);

L29x motorRight(8, 9, 10); // pin8 PWM=EnableA, pin 9 is IN1, pin 10 is IN2
L29x motorLeft(11, 12, 13);

//Button motorBtn(13, false, false, 20);
const uint8_t LED_PIN = 13;
String SEPARATOR = ","; //Used as separator for telemetry

// Tell it where to store your config data in EEPROM
boolean LCD_Output = false;
//int debug = 0;
int particleNumber = 0;

/* Encoders */

// This optional setting causes Encoder to use more optimized code,
// It must be defined before Encoder.h is included.
//#define ENCODER_OPTIMIZE_INTERRUPTS

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//#define leftEncoder1 configuration.leftEncoder1
//#define leftEncoder2 configuration.leftEncoder2
//#define rightEncoder1 configuration.rightEncoder1
//#define rightEncoder2 configuration.rightEncoder2

Encoder MotorLeft(configuration.leftEncoder1, configuration.leftEncoder2);
Encoder MotorRight(configuration.rightEncoder1, configuration.rightEncoder2);


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


SerialCommand SCmd;   // The SerialCommand object

// imu variables
float imuValues[6];
float ypr[3];
float pitch, roll, yaw, prev_pitch, prev_yaw, prev_roll;
float pitchd1, pitchd2;

long StartL=0, LoopT=0, StartL2=0;

String LastEvent="";

// PID variables
double anglePIDSetpoint, anglePIDInput, anglePIDOutput;
double speedPIDInput, speedPIDOutput, speedPIDSetpoint;
double anglePIDInputFiltered;
double ISTE = 0, dISTE = 0;
int AUTOTUNE = 0;

// filters
//CHECK if the moving average filters and the FIR are needed
//MovingAvarageFilter speedMovingAvarageFilter(14); 
//MovingAvarageFilter angleMovingAvarageFilter(4);
KalmanFilter speedKalmanFilter;
KalmanFilter angleKalmanFilter;
KalmanFilter balanceKalmanFilter;
//FIR speedMovingAvarageFilter2;

// The cascading PIDs. The tunings are updated from the remote
PID anglePID(&anglePIDInput, &anglePIDOutput, &anglePIDSetpoint, 0, 0, 0, DIRECT); 
PID speedPID(&speedPIDInput, &speedPIDOutput, &speedPIDSetpoint, 0, 0, 0, DIRECT);

// Set the FreeSixIMU object. This handles the communcation to the IMU.
FreeSixIMU sixDOF = FreeSixIMU();

//char notes[] = "ccggaagffeeddc "; // a space represents a rest
//int beats[] = { 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 2, 4 };
//int tempo = 300;


String SPACER = " ";
String Note = "";
String LastEventSPO ="";

//Init code goes in the main body of the sketch
const int numParticles = 20;//configuration.numParticles; //Needs to be small as it gets the feedback from the real system for example 10 particles for 10 iteractions for 3 s will take 5 mis to finish. An idea can be to define a criteria to kill particles
double maxInteractions = configuration.maxInteractions;
double bestParticle = 0;
double bestIteraction = 0;
double SPOiteraction = 0;
double bestGlobalFitness = 9999;


//------------------ SPO ------------------ 
//Define search space for test
double    minKp =  configuration.anglePIDConKp * (1 - configuration.SPOspread), 
 maxKp =  configuration.anglePIDConKp * (1 + configuration.SPOspread),
 minKi =  configuration.anglePIDConKi * (1 - configuration.SPOspread),
 maxKi =  configuration.anglePIDConKi * (1 + configuration.SPOspread),
 minKd =  configuration.anglePIDConKd * (1 - configuration.SPOspread), 
 maxKd =  configuration.anglePIDConKd * (1 + configuration.SPOspread);

//Need to be smarter. Define a function based on: a) domain, d/dt of ISTE 
double maxVel = 5;//min(minKp, minKi)/10;


typedef struct  // create a new user defined structure called particle
{
  double pos[2]; // Kp, Ki, Kd
  double vel[2];
  double fitness;
  double Bpos[2];
  double PARbestFitness; 
} 
particle;

particle swarn[numParticles]; 

typedef struct //Defines the space where the particle can move
{
  double minR;
  double maxR;        
} 
space;

space domain[2];

double bestGlobalPosition[3];

  //Init the timers

  // These take care of the timing of things
TimedAction debugTimedAction = TimedAction(configuration.debugSampleRate, debugEverything); //Print debug info
TimedAction updateMotorStatusesTimedAction = TimedAction(configuration.motorSpeedSensorSampling, updateMotorSpeeds); //
TimedAction updateIMUSensorsTimedAction = TimedAction(configuration.angleSensorSampling, updateIMUSensors);

//TimedAction remoteControlWatchdogTimedAction = TimedAction(5000, stopRobot);

//Reads serial for commands
TimedAction RemoteReadTimedAction = TimedAction(250, RemoteRead);

//Upload telemetry data
TimedAction TelemetryTXTimedAction = TimedAction(250, TelemetryTX);

//Swarn Particle Optimization
TimedAction SwarnTimedAction = TimedAction(configuration.SPOConfigEval, SPO);




//------------------ Setup ------------------ 
void setup() { 
  //pinMode(configuration.speakerPin, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(50);
 
  // Load config from eeprom
  setConfiguration(false);
  // init i2c and IMU
  delay(100);
  Wire.begin();

  //Init control systems
  controlConfig();

  // Filters
  speedKalmanFilter.setState(0);
  balanceKalmanFilter.setState(0);
  //float coefficients[] = {
  //  1,1,1,1,1,1,1,1,1              };
  //speedMovingAvarageFilter2.setCoefficients(coefficients);
  //speedMovingAvarageFilter2.setNumberOfTaps(9);
  //speedMovingAvarageFilter2.setGain((float) (1/9));

  //Serial.println("IMU...");
  delay(50);
  sixDOF.init(); //init the IMU
  delay(150);



  if (configuration.debug==1)
    debugConfiguration();

  //saveConfig();
  // set the watchdog to 2s (this will restart the arduino if it freezes)
  wdt_enable(WDTO_2S);
  
  swarnInit();
  LastEvent = "Swarn ready";
  //RemoteInit();
  
  // Setup callbacks for SerialCommand commands 
  SCmd.addCommand("SCMD", setCommand);       
  SCmd.addCommand("SCMD2", setCommand2);       
  SCmd.addCommand("READ", printCommand); 
  //SCmd.addDefaultHandler(unrecognizedCMD);  // Handler for command that isn't matched  
  //play(notes, beats);
}

void updateIMUSensors() {
  double angleT;
  prev_pitch = pitch;
  sixDOF.getYawPitchRoll(ypr);
  //sixDOF.getEuler(ypr);
  yaw = ypr[0];
  roll = ypr[1];
  pitch = ypr[2];
  pitchd1 = pitch - prev_pitch;  
 
  
  angleT = pitch; 
  // move angle to around equilibrium
  angleKalmanFilter.correct(angleT);
  anglePIDInput = angleKalmanFilter.getState();// Dont think it is needed here "- configuration.calibratedZeroAngle;"
}

//------------------ Main loop ------------------ 
void loop() { 
  StartL = millis();
  wdt_reset();

  // update sensors and motors, also chek commands and send back telemetry
  updateIMUSensorsTimedAction.check();
  updateMotorStatusesTimedAction.check();
  RemoteReadTimedAction.check();
  TelemetryTXTimedAction.check();
  
  if (AUTOTUNE==1) {
   SwarnTimedAction.check();    
  }

    // Speed pid,  input is wheel speed, output is angleSetpoint
    speedPIDSetpoint =  configuration.Maxthrottle*UserControl[1];
    speedPID.Compute();
    anglePIDSetpoint = - speedPIDOutput;

    // Update angle PID tuning
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
    //Filter the angle with Kalman
    //angleKalmanFilter.correct(anglePIDOutput);
    //anglePIDOutput = angleKalmanFilter.getState();
   
    if (configuration.motorsON==1){
      //Serial.println("input sent to motors");
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
  dISTE = (LoopT/1000*(anglePIDSetpoint - pitch)*(anglePIDSetpoint - pitch));  
  ISTE = ISTE + dISTE;
}

/* just debug functions. uncomment the debug information you want in debugEverything */
void debugEverything() {
  debugImu();
  debugAnglePID();
  debugSpeedPID();
  //debugISE();
  //debugAnglePIDCoeff();
  //debugSpeedPIDCoeff();
  //debugEncoders();
  debugMotorSpeeds();
  //debugMotorCalibrations();
  //debugMotorSpeedCalibration();
  //debugChart2();
  //unrecognizedCMD();
  debugLoopTime();
  Serial.println();

};


