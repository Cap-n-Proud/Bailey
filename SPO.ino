//This script must run as a timed action. Target function (ISTE) is calculate in the main loop and resetted here

/*
For the moment just calculate ISTE, in future define some criteria to ensure the robot doesn't fall
even in case of wrong parameters, e.g. if dISTE > than Max, reset the particle to its best known pos
and PID to overall best known pos?

https://github.com/kiuz/Arduino-Statistic-Library
Need to add the measure using the Statistic library and check if stdev goes big
As an alternative we can use the derivative ISTE2-ISTE1

*/  

//Init code goes in the main body of the sketch
const int numParticles = 30; //Needs to be small as it gets the feedback from the real system for example 10 particles for 10 iteractions for 3 s will take 5 mis to finish. An idea can be to define a criteria to kill particles
double maxInteractions = 30;
int particleNumber = 0;
double bestParticle = 0;
double bestIteraction = 0;
double SPOiteraction = 0;
//int feedbackTime = 3000; //feedback time in milliseconds. Maybe we can make it dynamic, first short time, after long time
double bestGlobalFitness = 9999;
//double maxIncreaseRate = 1; //condition to stop the particle test in before the feedbackTime if the error rate increase out of control
//Define a percentage arounf the known stable values
const int spread = 20;
double    minKp =  configuration.anglePIDConKp * (1 - spread/100), 
          maxKp =  configuration.anglePIDConKp * (1 + spread/100),
          minKi =  configuration.anglePIDConKi * (1 - spread/100),
          maxKi =  configuration.anglePIDConKi * (1 + spread/100),
          minKd =  configuration.anglePIDConKd * (1 - spread/100), 
          maxKd =  configuration.anglePIDConKd * (1 + spread/100);
//double minKp = -20, maxKp = 20, minKi = -20, maxKi = 20, minKd = -50, maxKd =50;
//Need to be smarter. Define a function based on: a) domain, d/dt of ISTE 
double maxVel = 2;//min(minKp, minKi)/10;
//String LastEvent ="";
boolean debugSPO = false;

typedef struct  // create a new user defined structure called particle
{
    double pos[2]; // Kp, Ki, Kd
    double vel[2];
    double fitness;
    double Bpos[2];
    double PARbestFitness; 
} particle;

particle swarn[numParticles]; 

typedef struct //Defines the space where the partilce can move
{
    double minR;
    double maxR;        
} space;

space domain[2];


double bestGlobalPosition[3];

//------------------------------------------
//Call this in the setup
void swarnInit(){
double curVal = 9999;
double bestVal = 9999;


randomSeed(analogRead(0));
//Initialize the swarn positions and velocites
for (int i = 0; i < numParticles; i++){
    swarn[i].pos[0] = random(minKp, maxKp);// Kp space
    swarn[i].pos[1] = random(minKi, maxKi);// Ki space
    swarn[i].pos[2] = random(minKd, maxKd);// Kd space

    swarn[i].Bpos[0] = random(minKp, maxKp);// Kp space
    swarn[i].Bpos[1] = random(minKi, maxKi);// Ki space
    swarn[i].Bpos[2] = random(minKd, maxKd);// Kd space
    
    swarn[i].vel[0] = min(random(minKp, maxKp), maxVel);// Kp space
    swarn[i].vel[1] = min(random(minKi, maxKi), maxVel);// Ki space
    swarn[i].vel[2] = min(random(minKd, maxKd), maxVel);// Kd space
 
    swarn[i].PARbestFitness = 9999;
}


domain[0].minR = minKp;
domain[0].maxR = maxKp;
domain[1].minR = minKi;
domain[1].maxR = maxKi;
domain[2].minR = minKd;
domain[2].maxR = maxKd;


bestGlobalPosition[0]= 9999;
bestGlobalPosition[1]= 9999;
bestGlobalPosition[2]= 9999;
 

for (int i = 0; i < numParticles; i++){
  curVal = ISTEF(swarn[i].pos[0], swarn[i].pos[1], swarn[i].pos[2]);
  bestVal = ISTEF(swarn[i].Bpos[0], swarn[i].Bpos[1], swarn[i].Bpos[2]);
  swarn[i].PARbestFitness = bestVal;

 if(curVal < bestGlobalFitness){
  bestGlobalPosition[0]= configuration.anglePIDConKp;
  bestGlobalPosition[1]= configuration.anglePIDConKi;
  bestGlobalPosition[2]= configuration.anglePIDConKd;
 }
  
}

}

//Call this as a scheduled task with feedbackTime as a trigger. The task is checked only if the varible AUTOTUNE is true. 
//SCMD AUTOTUNE 1
//------------------------------------------

double ISTEF(double x, double y, double z)
{
  //x, y, z are for testing, when you us an analytical function instead of ISTE
  //z = x * exp( -(x^2 + y^2) ) The function has a known minimum value of z = -0.4288819 at x = -0.7071068 and y = 0.0.
  //return x * exp( -(x*x + y*y) ); 
   //return -1/(x*x+y*y+1);
   return ISTE;
}

void SPO(){

//Particle data
double w = 0.729; // inertia weight
double c1 = 1.49445; // cognitive weight def 1.49445
double c2 = 1.49445; // social weight
double c1Min = 1, c1Max = 1.8;
double c2Min = 1, c2Max = 1.8;
double curVal = 9999;

double r1, r2; // randomizers
double newVel, newPos, mv; 

    curVal = ISTEF(swarn[particleNumber].pos[0], swarn[particleNumber].pos[1], swarn[particleNumber].pos[2]);
    LastEvent = "P: " + String(particleNumber) + " I: " + String((int)SPOiteraction) + "\nPOS: ";
    for (int j = 0; j <3; j++){LastEvent = LastEvent + (int)(swarn[particleNumber].pos[j]*100) + " ";}   
    

    randomSeed(analogRead(0));   
    
        //Need to check the target value of the current position
        if (curVal < swarn[particleNumber].PARbestFitness) {
            swarn[particleNumber].PARbestFitness = curVal;
            for (int j = 0; j < 3; j++){swarn[particleNumber].Bpos[j] = swarn[particleNumber].pos[j];};
        }
        if (curVal < bestGlobalFitness) {
            bestGlobalFitness = curVal;
            for (int j = 0; j < 3; j++){bestGlobalPosition[j] = swarn[particleNumber].pos[j];};
            LastEvent = LastEvent + "*********************NEW BEST FOUND*********************"; 
            bestParticle = particleNumber;
            bestIteraction = SPOiteraction;
        }
    
        for (int j = 0; j < 3; j++){
           r1 = random(0, 999);
           r2 = random(0, 999);
           r1 = r1/999;
           r2 = r2/999;
           c1= (c1Min-c1Max)*(SPOiteraction/maxInteractions)+c1Max;
           c2= (c2Max-c2Min)*(SPOiteraction/maxInteractions)+c2Min;
           
           newVel = w*swarn[particleNumber].vel[j] + c1*r1*(swarn[particleNumber].Bpos[j] - swarn[particleNumber].pos[j]) + c2*r2*(bestGlobalPosition[j] - swarn[particleNumber].pos[j]);
           swarn[particleNumber].vel[j] = newVel;
          
          //maxVel = (1-pow((SPOiteraction/maxInteractions), 2))*maxVel;
          // Serial.println(SPOiteraction);
          //Serial.println(maxInteractions);
          
          //Check if vel is allowed
          if (swarn[particleNumber].vel[j] > maxVel){
             swarn[particleNumber].vel[j] = maxVel;//(swarn[particleNumber].vel[j]/abs(swarn[particleNumber].vel[j]))*maxVel;
             LastEvent = LastEvent + ("***Maxvel MAX***"); 
          }
          if (swarn[particleNumber].vel[j] < -maxVel){
             swarn[particleNumber].vel[j] = -maxVel;//(swarn[particleNumber].vel[j]/abs(swarn[particleNumber].vel[j]))*maxVel;
             LastEvent = LastEvent + ("***Maxvel LOWER***"); 
          }
          
          //Now update the position
          if(debugSPO && particleNumber == 3){Serial.print("pos before upd ");Serial.println(swarn[particleNumber].pos[j]);}
          
          newPos = swarn[particleNumber].pos[j] + swarn[particleNumber].vel[j];
          swarn[particleNumber].pos[j] = newPos;
      
          if (swarn[particleNumber].pos[j] < domain[j].minR){
             swarn[particleNumber].pos[j] = domain[j].minR;//random(domain[j].minR,domain[j].maxR);
             LastEvent = LastEvent + ("***OOB LOWER***"); 
          }
          if (swarn[particleNumber].pos[j] > domain[j].maxR){
              swarn[particleNumber].pos[j] = domain[j].maxR;//random(domain[j].minR,domain[j].maxR);  
              LastEvent = LastEvent + ("***OOB MAX***"); 
          }
      
          //for debug we follow a particle
          if(debugSPO == true && particleNumber == 3){
            Serial.print("dim ");Serial.println(j);
            Serial.print("r1 r2 ");Serial.print(r1);Serial.print(" ");Serial.println(r2);
            Serial.print("new vel ");Serial.println(swarn[particleNumber].vel[j]);
            Serial.print("pos after upd ");Serial.println(swarn[particleNumber].pos[j]);
            Serial.println(maxVel);
            
            Serial.println("-------------------------------");
          }

        }      
          //Check if position is allowed
    
    //LastEvent = LastEvent + "\nVEL: "; 
    //LastEvent = LastEvent + String((int)(swarn[particleNumber].vel[0]*100)) + " "+ (int)(swarn[particleNumber].vel[1]*100) + " "+ (int)(swarn[particleNumber].vel[2]*100) + " ";
    LastEvent = LastEvent + ("\nISTE: ") +  (int)(curVal) + " Best " + (int)bestGlobalFitness;
    LastEvent = LastEvent +  " " + (int)(bestGlobalPosition[0]*100) +  " " + (int)(bestGlobalPosition[1]*100) +  " " + (int)(bestGlobalPosition[2]*100);
    LastEvent = LastEvent + " maxVel " + (int)(maxVel*100);
    LastEvent = LastEvent + ("\n________________________"); 
    
   if (debugSPO == true && particleNumber == 3){
    //Serial.print("Vel ");Serial.print(swarn[particleNumber].vel[0]);Serial.print(" ");Serial.print(swarn[particleNumber].vel[1]);Serial.print(" ");Serial.print(swarn[particleNumber].vel[2]);
    //Serial.print(" Pos ");Serial.print(swarn[particleNumber].pos[0]);Serial.print(" ");Serial.print(swarn[particleNumber].pos[1]);Serial.print(" ");Serial.println(swarn[particleNumber].pos[2]);
   
   Serial.println(LastEvent);
 }    
   
   
    //Pass to the next particle and change PID settings
    //Change the robot's Kp, Ki, Kd based on the first particle position
    configuration.anglePIDConKp = swarn[particleNumber].pos[0];
    configuration.anglePIDConKi = swarn[particleNumber].pos[1];
    configuration.anglePIDConKd = swarn[particleNumber].pos[2]; 
    controlConfig();
    curVal = ISTEF(swarn[particleNumber].pos[0], swarn[particleNumber].pos[1], swarn[particleNumber].pos[2]);
   
    
    //Reset ISTE and update the other global varibles
    ISTE=0;
    particleNumber++;
    //If I have looped thorugh all the particles, the cycle starts again in a subsequent interaction
    if (particleNumber == numParticles) {
        SPOiteraction++;
        particleNumber = 0;
    }
    
    if (SPOiteraction == maxInteractions){
        AUTOTUNE = 0;
        SPOiteraction = 0;
        particleNumber = 0;
      
        configuration.anglePIDConKp = bestGlobalPosition[0];
        configuration.anglePIDConKi = bestGlobalPosition[1];
        configuration.anglePIDConKd = bestGlobalPosition[2];
        //Apply new vaues to PID
        controlConfig();
        /*
        Serial.println("*********************RESULTS**********************");
        Serial.println("*********************RESULTS**********************");
        Serial.println("*********************RESULTS**********************");
        for (int j = 0; j <3; j++)Serial.println(bestGlobalPosition[j]);
        Serial.print("Fitness: ");Serial.println(bestGlobalFitness);
        Serial.print("Particle: ");Serial.println(bestParticle);
        Serial.print("Iteraction: ");Serial.println(bestIteraction);
        Serial.println("*********************RESULTS**********************");
        Serial.println("*********************RESULTS**********************");
        Serial.println("*********************RESULTS**********************");
        */
    }    
     Serial.println(curVal);
     if (curVal > 30){
      configuration.anglePIDConKp = bestGlobalPosition[0];
      configuration.anglePIDConKi = bestGlobalPosition[1];
      configuration.anglePIDConKd = bestGlobalPosition[2]; 
      controlConfig();
      
      Serial.println("**********");
    } 
}


