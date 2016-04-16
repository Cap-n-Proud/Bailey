// Check this out https://github.com/adafruit/AccelStepper.git
// https://www.pjrc.com/teensy/td_libs_AccelStepper.html
//http://42bots.com/tutorials/bipolar-stepper-motor-control-with-arduino-and-an-h-bridge/
//------------------ Libraries ------------------


//#define DEBUG

#include "FreeIMU.h"
#include <Wire.h>
#include <SPI.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#define M_PI 3.14159265358979323846f
#include <MS561101BA.h>
#include <HMC58X3.h>
#include <Wire.h> // for i2c
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library.git
#include <TimedAction.h> // for updating sensors and debug http://bit.ly/pATDBi http://playground.arduino.cc/Code/TimedAction
#include <EEPROM.h> // for storing configuraion
//#include <avr/wdt.h> // watchdog http://savannah.nongnu.org/projects/avr-libc/
//#include <KalmanFilter.h> // github.com/nut-code-monkey/KalmanFilter-for-Arduino
// Try also: https://github.com/TKJElectronics/KalmanFilter.git
#include <AccelStepper.h> //https://github.com/adafruit/AccelStepper.git

#define SERIALCOMMAND_HARDWAREONLY 1
#include <SerialCommand.h> // https://github.com/kroimon/Arduino-SerialCommand.git


//------------------ Constants ------------------
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi
#define speedMultiplier 1
#define SERIAL_BAUD 38400
#define CONFIG_START 32 //EEPROM address to start the config

/* Configutation parameters */
struct Configuration {
  String FirmwareVersion;

  double speedPIDKp;
  double speedPIDKi;
  double speedPIDKd;
  double speedPIDMoveFactor;
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
    if (configuration.debug) {
      Serial.print("No config found, defaulting ");
    }
    /* First time running, set defaults */
    configuration.FirmwareVersion = "1.2";
    configuration.speedPIDOutputLowerLimit = -10.00; //Default was -5
    configuration.speedPIDOutputHigherLimit = 10.00;

    configuration.anglePIDConKp = 8.80;
    configuration.anglePIDConKi = 2.21;
    configuration.anglePIDConKd = 0.975;
    configuration.speedPIDMoveFactor = 0.7;

    configuration.anglePIDAggKp = 12.70;
    configuration.anglePIDAggKi = 3.5;
    configuration.anglePIDAggKd = 1.951;

    configuration.speedPIDKp = 0.3854;
    configuration.speedPIDKi = 0.1374;
    configuration.speedPIDKd = 0.00245;
    configuration.anglePIDOutputLowerLimit = -100;
    configuration.anglePIDOutputHigherLimit = 100;

    configuration.MotorLeftENABLEA = 8;
    configuration.MotorLeftIN1 = 9;
    configuration.MotorLeftIN2 = 10;
    configuration.MotorRightENABLEA = 11;
    configuration.MotorRightIN1 = 12;
    configuration.MotorRightIN2 = 13;
    configuration.leftEncoder1 = 2;
    configuration.leftEncoder2 = 4;
    configuration.rightEncoder1 = 3;
    configuration.rightEncoder2 = 5;

    configuration.motorLeftMinimumSpeed = 55;//40;
    configuration.motorRightMinimumSpeed = 58;//44;
    configuration.steerGain = 1;
    configuration.throttleGain = 1;
    configuration.Maxsteer = 20; //Max allowed percentage difference. Up to the remote to provide the right scale.
    configuration.Maxthrottle = 6; //Max speed expressed in inclination degrees. Up to the remote to provide the right scale.

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

//------------------ Definitions MOTORS ------------------

//------------------ Definitions IMU  ------------------
String SEPARATOR = ","; //Used as separator for telemetry
float ypr[3];
float altimeter; // yaw pitch roll
float imuValues[6];
float pitch, roll, yaw, prev_pitch, prev_yaw, prev_roll;
float pitchd1, pitchd2;


// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();

float speedKalmanFiltered = 0;
float speedFIRFiltered = 0;
float dISTEKalmanFiltered = 0;

SerialCommand SCmd;   // The SerialCommand object

long StartL = 0, LoopT = 0, StartL2 = 0, TxLoopTime = 0;

String LastEvent = "";
String PIDConfigType = "";

// PID variables
double anglePIDSetpoint, anglePIDInput, anglePIDOutput;
double speedPIDInput, speedPIDOutput, speedPIDSetpoint;
double anglePIDInputFiltered;
double ISTE = 0, dISTE = 0;
int AUTOTUNE = 0;

// The cascading PIDs. The tunings are updated from the remote
PID anglePID(&anglePIDInput, &anglePIDOutput, &anglePIDSetpoint, 0, 0, 0, DIRECT);
PID speedPID(&speedPIDInput, &speedPIDOutput, &speedPIDSetpoint, 0, 0, 0, DIRECT);

//Init the timers
// These take care of the timing of things
TimedAction debugTimedAction = TimedAction(configuration.debugSampleRate, debugEverything); //Print debug info
//TimedAction updateMotorSpeedTimedAction = TimedAction(100, updateMotorSpeeds); //
//ADD HERE A TIMED ACTIONS FOR SENSORS
//TimedAction remoteControlWatchdogTimedAction = TimedAction(5000, stopRobot);

//Reads serial for commands
TimedAction RemoteReadTimedAction = TimedAction(250, RemoteRead);

//Reads serial for commands
TimedAction ReadIMUTimedAction = TimedAction(100, ReadIMU);

//Upload telemetry data
TimedAction TelemetryTXTimedAction = TimedAction(250, TelemetryTX);



//------------------ Setup ------------------
void setup() {
  //pinMode(configuration.speakerPin, OUTPUT);

  Serial.begin(SERIAL_BAUD);
  delay(50);

  // Load config from eeprom
  setConfiguration(true);
  // init i2c and IMU
  delay(100);
  Wire.begin();

  //Init control systems
  controlConfig();
  // motorsSetup();
  // Filters
  // speedKalmanFilter.setState(0);
  // balanceKalmanFilter.setState(0);
  // dISTEKalmanFilter.setState(0);
  //float coefficients[] = {
  //  1,1,1,1,1,1,1,1,1              };
  //speedMovingAvarageFilter2.setCoefficients(coefficients);
  //speedMovingAvarageFilter2.setNumberOfTaps(9);
  //speedMovingAvarageFilter2.setGain((float) (1/9));

  //Serial.println("IMU...");
  delay(50);
  my3IMU.init(); //init the IMU
  delay(150);



  if (configuration.debug == 1)
    debugConfiguration();

  //saveConfig();
  // set the watchdog to 2s (this will restart the arduino if it freezes)
  //wdt_enable(WDTO_2S);

  // swarnInit();
  LastEvent = "Swarn ready";
  //RemoteInit();

  // Setup callbacks for SerialCommand commands
  SCmd.addCommand("SCMD", setCommand);
  SCmd.addCommand("READ", printCommand);
  //SCmd.addDefaultHandler(unrecognizedCMD);  // Handler for command that isn't matched
  //play(notes, beats);
}

//------------------ Main loop ------------------
void loop() {
  StartL = millis();
  //wdt_reset();
  // update sensors and motors, also chek commands and send back telemetry
  //ADD HERE A TIMED ACTIONS FOR SENSORS
  ReadIMUTimedAction.check();
  RemoteReadTimedAction.check();
  TelemetryTXTimedAction.check();
  //updateMotorSpeedTimedAction.check();
  //updateMotorSpeeds(UserControl[0], UserControl[1]);

  if (AUTOTUNE == 1) {
    //SwarnTimedAction.check();
  }

  // Speed pid,  input is wheel speed, output is angleSetpoint
  speedPIDSetpoint = configuration.Maxthrottle * UserControl[1];
  //If we are moving we need to reduce the Speed PID values in order to avoud "stops and go"
  if (UserControl[1] != 0) {
    speedPID.SetTunings(configuration.speedPIDMoveFactor * configuration.speedPIDKp, configuration.speedPIDMoveFactor * configuration.speedPIDKi, configuration.speedPIDMoveFactor * configuration.speedPIDKd);
  } else {
    speedPID.SetTunings(configuration.speedPIDKp, configuration.speedPIDKi, configuration.speedPIDKd);
  }
  speedPID.Compute();
  anglePIDSetpoint = -speedPIDOutput;

  // Update angle PID tuning
  if (abs(anglePIDInput) < (float) configuration.TriggerAngleAggressive && configuration.TriggerAngleAggressive != 0) {
    //we're close to setpoint, use conservative tuning parameters
    anglePID.SetTunings((float) configuration.anglePIDConKp, (float) configuration.anglePIDConKi, (float) configuration.anglePIDConKd);
    PIDConfigType = "CONSERVATIVE";

  } else if (abs(anglePIDInput) > (float) configuration.TriggerAngleAggressive && abs(anglePIDInput) <= 30) {

    //we're far from setpoint, use aggressive tuning parameters
    anglePID.SetTunings((float) configuration.anglePIDAggKp, (float) configuration.anglePIDAggKi, (float) configuration.anglePIDAggKd);
    PIDConfigType = "AGGRESSIVE";

  } else {
    anglePID.SetTunings(0, 0, 0);
  }

  anglePID.Compute();
  //Filter the angle with Kalman
  //angleKalmanFilter.correct(anglePIDOutput);
  //anglePIDOutput = angleKalmanFilter.getState();

  if (configuration.motorsON == 1) {
    //Serial.println("input sent to motors");

    //motorLeft.setSpeedPercentage(-anglePIDOutput - configuration.steerGain * (UserControl[0])); // + configuration.throttleGain*UserControl[1]);
    //motorRight.setSpeedPercentage(-anglePIDOutput + configuration.steerGain * (UserControl[0])); // + configuration.throttleGain*UserControl[1]);
  } else {
    //motorLeft.setSpeed(0);
    //motorRight.setSpeed(0);
  }
  if (configuration.debug == 1)
    debugTimedAction.check();


  dISTE = (LoopT / 1000 * (anglePIDSetpoint - pitch) * (anglePIDSetpoint - pitch));
  ISTE = ISTE + dISTE;
  LoopT = millis() - StartL;
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


void ReadIMU() {
  my3IMU.getYawPitchRoll(ypr);
  altimeter = my3IMU.getBaroAlt();
}

