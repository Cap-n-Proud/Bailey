//This script must run as a timed action. Target function (ISTE) is calculate in the main loop and resetted here

/*
For the moment just calculate ISTE, in future define some criteria to ensure the robot doesn't fall
 even in case of wrong parameters, e.g. if dISTE > than Max, reset the particle to its best known pos
 and PID to overall best known pos?
 
 https://github.com/kiuz/Arduino-Statistic-Library
 Need to add the measure using the Statistic library and check if stdev goes big
 As an alternative we can use the derivative ISTE2-ISTE1
 
 */




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

 /* 
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
*/
}

//Call this as a scheduled task with feedbackTime as a trigger. The task is checked only if the varible AUTOTUNE is true. 
//SCMD AUTOTUNE 1
//------------------------------------------

double ISTEF(double x, double y, double z)
{
  //x, y, z are for testing, when you us an analytical function instead of ISTE
  //z = x * exp( -(x^2 + y^2) ) ;The function has a known minimum value of z = -0.4288819 at x = -0.7071068 and y = 0.0.
  //return  x * exp( -(x*x + y*y) ); 
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
 
  randomSeed(analogRead(0));   

  //Need to check the target value of the current position
  if (curVal < swarn[particleNumber].PARbestFitness) {
    swarn[particleNumber].PARbestFitness = curVal;
    for (int j = 0; j < 3; j++){
      swarn[particleNumber].Bpos[j] = swarn[particleNumber].pos[j];
    };
  }
  if (curVal < bestGlobalFitness) {
    bestGlobalFitness = curVal;
    for (int j = 0; j < 3; j++){
      bestGlobalPosition[j] = swarn[particleNumber].pos[j];
    };
    Note = "*********************NEW BEST FOUND*********************"; 
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
       //Check if vel is allowed
    if (swarn[particleNumber].vel[j] > maxVel){
      swarn[particleNumber].vel[j] = maxVel;
      Note = Note + ("***Maxvel MAX*** ") + j; 
    }
    if (swarn[particleNumber].vel[j] < -maxVel){
      swarn[particleNumber].vel[j] = -maxVel;
      Note = Note + ("***Maxvel LOWER*** ") + j; 
    }

    //Now update the position
    newPos = swarn[particleNumber].pos[j] + swarn[particleNumber].vel[j];
    swarn[particleNumber].pos[j] = newPos;

    //Check if position is allowed
    if (swarn[particleNumber].pos[j] < domain[j].minR){
      swarn[particleNumber].pos[j] = random(domain[j].minR,domain[j].maxR); //
      Note = Note + ("***OOB LOWER*** ") + j; 
    }
    if (swarn[particleNumber].pos[j] > domain[j].maxR){
      swarn[particleNumber].pos[j] = random(domain[j].minR,domain[j].maxR);  //domain[j].maxR;
      Note = Note + ("***OOB MAX*** ") + j; 
    }

  }      
  
  LastEventSPO = +  ("\n__________________________________")+ SPACER + "\nInt: " + (int)SPOiteraction + SPACER + dtostrf(maxVel, 10, 3, cbuffer) + (" Particle: ") + particleNumber + SPACER 
              + "\nPosition: " + dtostrf(swarn[particleNumber].pos[0], 10, 3, cbuffer) + SPACER  +  dtostrf(swarn[particleNumber].pos[1], 10, 3, cbuffer) + SPACER  +  dtostrf(swarn[particleNumber].pos[2], 10, 3, cbuffer)
              + (" ISTE: ") + dtostrf(curVal, 10, 5, cbuffer) 
              + "\nParticle best: " + SPACER + dtostrf(swarn[particleNumber].Bpos[0] , 10, 3, cbuffer) + SPACER + dtostrf(swarn[particleNumber].Bpos[1] , 10, 3, cbuffer) + SPACER + dtostrf(swarn[particleNumber].Bpos[2] , 10, 3, cbuffer) + SPACER + dtostrf(swarn[particleNumber].PARbestFitness , 10, 3, cbuffer)
              +  "\nBest gFitness" + dtostrf(bestGlobalFitness, 10, 3, cbuffer) + SPACER + "gPos: " + dtostrf(bestGlobalPosition[0], 10, 3, cbuffer) + SPACER + dtostrf(bestGlobalPosition[1], 10, 3, cbuffer) + SPACER + dtostrf(bestGlobalPosition[2], 10, 3, cbuffer)
              +  "\nBest iteraction: " + (int)bestIteraction + SPACER + "best particle: " + (int)bestParticle 
              + "\n" + Note +("\n__________________________________"); 

  if (debugSPO == true){
     Serial.println(LastEventSPO);
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
  Note = "";
  particleNumber = particleNumber + 1;
  //If I have looped thorugh all the particles, the cycle starts again in a subsequent interaction
  
   if (particleNumber == numParticles) {
    SPOiteraction= SPOiteraction + 1;
    maxVel = 0.05 + (1-exp(-1+SPOiteraction/maxInteractions))*maxVel;
    particleNumber = 0;
  }

//Stop condittions: if I finish the number of cycles OR if the solution seems not to improve for some cycles
  if (SPOiteraction == maxInteractions || (SPOiteraction - bestIteraction) > 5){
    configuration.anglePIDConKp = bestGlobalPosition[0];
    configuration.anglePIDConKi = bestGlobalPosition[1];
    configuration.anglePIDConKd = bestGlobalPosition[2];
    //Apply new vaues to PID
    controlConfig();

    LastEventSPO = ("\n******************************************************************")+ SPACER + "\nInt: " + (int)bestIteraction + (" Particle: ") + (int)bestParticle + SPACER 
              +  "\nBest " + dtostrf(bestGlobalFitness, 10, 3, cbuffer) + SPACER + "Best Global: " + dtostrf(bestGlobalPosition[0], 10, 3, cbuffer) + SPACER + dtostrf(bestGlobalPosition[1], 10, 3, cbuffer) + SPACER + dtostrf(bestGlobalPosition[2], 10, 3, cbuffer)
              + ("\n******************************************************************"); 
   
   if ((SPOiteraction - bestIteraction) > 5)
   {
     LastEventSPO = "\nSoultion seems not to converge further... terminating" + LastEventSPO;
   }
   
   if (debugSPO == true){Serial.println(LastEventSPO);}
  
   AUTOTUNE = 0;
   SPOiteraction = 0;
   particleNumber = 0;
  }    
  
  //Serial.println(curVal);
  if (curVal > 30){
   configuration.anglePIDConKp = bestGlobalPosition[0];
   configuration.anglePIDConKi = bestGlobalPosition[1];
   configuration.anglePIDConKd = bestGlobalPosition[2]; 
   controlConfig();
    }
}



