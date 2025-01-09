/*                                                                                                                                                                                             
 ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ 
|______|______|______|______|______|______|______|______|______|______|______|______|______|______|______|______|______|                                                                                                                                                                                            

 .-----------------. .----------------.  .----------------.  .----------------. 
| .--------------. || .--------------. || .--------------. || .--------------. |
| | ____  _____  | || |  _________   | || | _____  _____ | || |  _________   | |
| ||_   \|_   _| | || | |_   ___  |  | || ||_   _||_   _|| || | |  _   _  |  | |
| |  |   \ | |   | || |   | |_  \_|  | || |  | | /\ | |  | || | |_/ | | \_|  | |
| |  | |\ \| |   | || |   |  _|  _   | || |  | |/  \| |  | || |     | |      | |
| | _| |_\   |_  | || |  _| |___/ |  | || |  |   /\   |  | || |    _| |_     | |
| ||_____|\____| | || | |_________|  | || |  |__/  \__|  | || |   |_____|    | |
| |              | || |              | || |              | || |              | |
| '--------------' || '--------------' || '--------------' || '--------------' |
 '----------------'  '----------------'  '----------------'  '----------------' 

 ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ ______ 
|______|______|______|______|______|______|______|______|______|______|______|______|______|______|______|______|______|    
      __                                     __     __       
|\ | /  \ |\ | __ |__| |  |  |\/|  /\  |\ | /  \ | |  \      
| \| \__/ | \|    |  | \__/  |  | /~~\ | \| \__/ | |__/      
                                                             
 ___      __        __   __       ___  __   __               
|__  \_/ |__) |    /  \ |__)  /\   |  /  \ |__) \ /          
|___ / \ |    |___ \__/ |  \ /~~\  |  \__/ |  \  |           
                                                             
                __   ___  __          __                     
|  |  /\  |\ | |  \ |__  |__) | |\ | / _`                    
|/\| /~~\ | \| |__/ |___ |  \ | | \| \__>                    
                                                             
___  __             ___  __   __                __   __  ___ 
 |  |__)  /\  \  / |__  |__) /__`  /\  |    __ |__) /  \  |  
 |  |  \ /~~\  \/  |___ |  \ .__/ /~~\ |___    |__) \__/  |  
                                                             


(ASCII art text generated @  https://patorjk.com/software/taag/#p=display&f=Blocks&t= )
/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/


The code for this project has been split into 7 segments:
Segment 1: Variables, pins, and constraints 
Segment 2: Sensors
Segment 3: Movement (includes further segregation into sub parts)
Segment 4:End of Maze Celebration
Segment 5: Exploration loop for finding the way to the end of the maze
Segment 6: Mapping and localisation
Segment 7: Setup() and loop()



/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Segment 1*/
/*Initialising variables,pins, and constants.*/

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include "mbed.h"
using namespace mbed;

/*Motors: */

/*Motor A direction 0 is forwards. Motor A is Left Wheel*/
/*Motor B Direction 1 is forwards. Motor B is Right Wheel*/
mbed::DigitalOut MotorADir(P0_4); //Set direction pin for motor A
mbed::DigitalOut MotorBDir(P0_5); //Set direction pin for motor B
mbed::PwmOut MotorAPwm(P0_27); //Set PWM pin for motor A
mbed::PwmOut MotorBPwm(P1_2); //Set PWM pin for motor B


//encoders and distancing:

mbed::InterruptIn EncA(P1_11); //Left wheel encoder pin declaration
long int ShaftRevA; // Counts how many full rotations of motor A there have been
long int EncCountA; // Counts pulses of motor A encoder
mbed::InterruptIn EncB(P1_12); //Left wheel encoder pin declaration
long int ShaftRevB; // Counts how many full rotations of motor B there have been
long int EncCountB; // Counts pulses of motor B encoder
long int BothEncCount; // increments when both wheels have moved equal distance.
float Distance; // used to find the distance travelled of a wheel in the distance function
float TargetDist; // Target Distance for the GoThisFar function.

// speed and squareup related:

float Speed = 0.5f; //general motor speed unless squaring up.
float Ramp = 0.005; //ramp for the ramp up function.
float SquareSpeed = 0.35f; // speed during SquareUp functions.
bool Square; //used to check whether the robot is currently square with a wall.
int StuckCounter = 0; //used to check if the robot is stuck.
int TimeOut = 0; //used to stop the robot from getting stuck in a squareup loop when squaring isn't possible.

//Ultrasonic Sensor Pins.
const int Ultra1Pin = 7;
int Ultra1Dist = 0;
const int Ultra2Pin = 4;
int Ultra2Dist = 0;
const int Ultra3Pin = 5;
int Ultra3Dist = 0;
const int Ultra4Pin = 6;
int Ultra4Dist = 0;

int UltraPin;
int FrontDist = 0;

//IR sensor variables
float OffSet = 0.27; // acceptable varience in sensors for squareup.
mbed::I2C i2c(P0_31, P0_2); // pin declaration for I2C.
const char BackLeft = 0x01;
const char BackRight = 0x02;
const char FrontRight = 0x04;
const char FrontLeft = 0x08;


//Localisation
//class will be used for global variables for localisation so that values can be passed between functions easier.
class GlobalStuff {
      public:
           String GetDirection() {
                 return Direction;
           }

           void SetDirection(String NewDirection) {
                  Direction = NewDirection;
           }
           float GetX(){
              return XCoordinate;
           }
           float GetY(){
              return YCoordinate;
           }
           void UpdateX(float NewX){
              XCoordinate = NewX;
           }
           void UpdateY(float NewY){
              YCoordinate = NewY;
           }
      private:
           String Direction = "West";
           float XCoordinate = 0;
           float YCoordinate = 0;
};

GlobalStuff Localisation;

//Mapping
float MovesToEnd[150] = {}; //array for storing what moves the robot took to get to the end
float MovesBackToStart[150] = {}; //array for reversing the moves taken in order to get back to the start
int CurrentMove = 0; //counts how many moves the robot has taken so far
int MapReversalCounter = 0; //counts how many moves have been reversed so far
// since array is storing floats, unreasonable values will be used to store the commands for a left turn, right turn, square up or reverse. 
// 2000 will be a Left Turn. 3000 will be a right turn. 4000 will be squaring up on the left wall. 5000 will be squaring up on the left wall.
int Left = 2000;
int Right = 3000;
int SquareLeft = 4000;
int SquareRight = SquareRight;


/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Segment 2*/
/*Functions regarding the operation of the sensors (excluding encoders as they are used for movement and as such are a part of the movement section)*/

/*Code in this section includes: 
-Taking readings from IR sensors.
-Averaging many IR readings to mitigate the effacts of a bad reading.
-Taking readings from Ultrasonic sensors.
-Checking the validity of readings from the front sensors via comparison with a second value as the ultrasonics can be unreliable.

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/




// Infrared functions.

float IRSensor(char mux_cmd){
  const char mux_addr = 0xEE; //multiplexer address.
  i2c.write(mux_addr, &mux_cmd, 1);
  char cmd[2]; //character array for IR sensors.
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1); 
  i2c.read(0x80, cmd, 2);
  float distance = (((float)(cmd[0] << 4) + (float)cmd[1])/16.0)/4.0; //bit shift to get actual distances from it. cast as a float to get mm.
  return distance;

}

float AverageIRSensor(char Sensor){ //take 25 readings from the IR sensor and average them. This will help to minimise the impact of any bad readings.
  float Total = 0.0;
  for (int i = 0; i <= 24; i++) { //loop 25 times
    float Reading = IRSensor(Sensor); //take a reading
    Total = Total + Reading; //add that reading to the total of all previous readings.
  }
  float AverageReading = Total / 25.0; //take the average
  return AverageReading;
  
}

/*Ultrasonic sensor code adapted from the arduino ide example code for the Ping ultrasonic sensor (File -> Examples -> Sensors -> Ping)
created 3 Nov 2008 by David A. Mellis
modified 30 Aug 2011 by Tom Igoe

This example code is in the public domain. */

long UltraSensor(int UltraPin){
  // establish variables for duration of the sensor, and the distance result in centimeters:
  long duration, cm;

  // The sensor is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(UltraPin, OUTPUT);
  digitalWrite(UltraPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltraPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(UltraPin, LOW);

  // The same pin is used to read the signal from the sensor: a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(UltraPin, INPUT);
  duration = pulseIn(UltraPin, HIGH);

  // convert the time into a distance
  cm = duration / 29 / 2;
  delay(100);
  return cm;

}

int CheckFront(){
  int FrontLeft = UltraSensor(Ultra1Pin); //check if there is a wall in front using ultrasonic.
  delay(200);
  int FrontRight = UltraSensor(Ultra3Pin); //check if there is a wall in front using ultrasonic
  if(FrontLeft < 20 && FrontRight < 20){
    int FrontDiff = abs(FrontLeft - FrontRight); //see if the values are similar, they are
    if(FrontDiff < 8){
      float FrontDist = (FrontLeft + FrontRight)/2;

      return FrontDist;
    }
    else{
      ReverseThisFar(10.0);
      Serial.println("retry");
      CheckFront();
    }
  }
  else if(FrontLeft < 20 && FrontRight >= 20 && FrontRight <= 900){
    FrontDist = FrontLeft;
    return FrontDist;
  }
  else if(FrontRight < 20 && FrontLeft >= 20 && FrontLeft <= 900){
    FrontDist = FrontRight;
    return FrontDist;
  }
  else{
    FrontDist = FrontLeft;
    return FrontDist;
  }

}


/*----------------------------------------------------------------------------------------------------------------------------------------------------------

/*Segment 3*/
/*Functions regarding the most basic Movement of the robot*/

/*Code in this section includes:

Subsection 1:
-Use of interrupts to find the position of the encoders.
-Use of encoder values for finding the number of revolutions of the wheel and the distance travelled by each wheel.

Subsection 2:
-Using the encoders to travel a specific mm distance in a straight line.
-A failsafe for if the robot gets stuck whilst trying to move in this straight line (generally from having crashed into a wall).
-A function for reversing in a straight line, useful for times when the robot is stuck or has gotten too close to the wall in front.

Subsection 3:
-A function for short stopping both motors at the same time.

Subsection 4:
-A function for turning 90 degrees left (or at least close to, there is some varience due to encoder unreliability.) 
-A function for turning 90 degrees right (or at least close to, there is some varience due to encoder unreliability.)
*Note* these functions also are responsible for updating the direction that the robot is now facing after each turn.

Subsection 5:
-A function for squaring up to the left wall. (functions best out to 12 cm away from the wall, becomes much less precise from there on).
-A function for squaring up to the right wall. (functions best out to 12 cm away from the wall, becomes much less precise from there on).
-A function for deciding whether to square up to the left or right wall.
-A slow ramping of speed to avoid overshooting while squaring up to the wall.


/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------------

/*Subsection 3.1: Encoders*/

/*-----------------------------------------------------------------------------------------------*/

void countPulseA(){ // counts the encoder pulses of wheel A.
   if(MotorADir == 0) //allows encoders to be directional.
     EncCountA++;
   else
     EncCountA--;
  countRevA(); //calls function that tracks revolutions.
}
void countPulseB(){ // same as previous function but for wheel B.
   if(MotorBDir == 1)
     EncCountB++;
   else
     EncCountB--;
  countRevB();
}

void countRevA(){ // Counts revolutions of wheel A.  330 encoder ticks per revolution.
   if(EncCountA%6 == 0){ //Checks if encoder A's value is a multiple of 6.
     if(EncCountA%110 == 0){ // Checks if its a multiple of 100 as well.
       ShaftRevA++; //increments the revolution count.
     }
   }
}
void countRevB(){// same as CountRevA but for wheel B.
   if(EncCountB%6 == 0){
     if(EncCountB%110 == 0){
       ShaftRevB++;
     }
   }
}

float DistanceA(){ //finds the distance travelled by wheel A.
  return Distance = EncCountA / 2.18383082; //660 / (48.1 * pi *2) = 2.18383082 which is the encoder ticks per mm of distance travelled.
}

float DistanceB(){ //same as above but for B.
  return Distance = EncCountB / 2.18383082;
}

float FindDistanceTravelled(){
  float A = DistanceA();
  float B = DistanceB();
  float DistanceForBoth = (A + B) / 2; //find how far the robot has actually moved using encoders
  return DistanceForBoth;
}


/*-----------------------------------------------------------------------------------------------

/*Subsection 3.2: Straight line movements forwards and backwards.*/

/*-----------------------------------------------------------------------------------------------*/
void GoThisFar(float TargetDist){ //function to move in a straight line for a set distance (in millimeters)
  EncCountA = 0; //reset encoder values
  EncCountB = 0;
  BothEncCount = 0;
  while(BothEncCount < (TargetDist * 2.18383082)){ //2.18... is the number of encoder ticks per mm. multiplying by the number of mm the robot should go gives the final encoder count the robot should have
    EncA.rise(&countPulseA); //update encoders
    EncB.rise(&countPulseB);
    MotorADir = 0; //assign wheel directions to go forwards.
    MotorBDir = 1;

    if (EncCountA > EncCountB){ //comparisons to make sure one wheel doesn't travel faster than the other. when one wheel overtakes the other, it is momentarily turned off so the other can catch up.
      MotorAPwm.write(0.0f);
      MotorBPwm.write(Speed);
    }
    else if(EncCountA < EncCountB){
      MotorAPwm.write(Speed);
      MotorBPwm.write(0.0f);
    }
    else if (EncCountA == EncCountB){
      MotorAPwm.write(Speed);
      MotorBPwm.write(Speed);
      BothEncCount = EncCountA; //when both encoder counts are equal, the robot must have travelled that far and can be compared with the target encoder value.
    }
    CheckIfStuck(); //check whether the wheels are actually moving or not.
  }

  stop(); //stop
  float DistanceTravelled = FindDistanceTravelled();
  UpdatePosition(); //Updates the new position of the robot on cartesian coordinate grid.
  AddToMap(DistanceTravelled);
}

void CheckIfStuck(){
  float EncA1 = EncCountA; //take a reading of the encoders
  float EncB1 = EncCountB;
  delay(1); //wait
  EncA.rise(&countPulseA);
  EncB.rise(&countPulseB);
  float EncA2 = EncCountA; //take a second reading of the encoders
  float EncB2 = EncCountB;
  if(EncA2 == EncA1 && EncB2 == EncB1){ //if the encoder values haven't changed
    StuckCounter = StuckCounter + 1; //robot might be stuck so increment stuck counter
  }
  else{ //encoder values have changed
    StuckCounter = 0;  //robot isn't stuck so reset stuck counter.
  }
  //Serial.println(StuckCounter);
  if(StuckCounter >= 1000){ //check whether the stuck counter is over 1000. for it to be this high the robot can't have moved in a while. indicating that it is stuck.
    Serial.println("Snitch I'M STUCK");
    ReverseThisFar(20); //reverse 20 mm
    SquareUp(); //squareup on the left
    StuckCounter = 0; //reset the stuck counter
    Explore(); //resume exploration (maybe not necessary)
  }
}

void ReverseThisFar(float TargetDist){ //same as GoThisFar but in reverse. Directions are switched and desired encoder value is now negative.
  EncCountA = 0;
  EncCountB = 0;
  BothEncCount = 0;
  while(BothEncCount > -(TargetDist * 2.18383082)){
    EncA.rise(&countPulseA);
    EncB.rise(&countPulseB);
    MotorADir = 1;
    MotorBDir = 0;

    if (EncCountA < EncCountB){
      MotorAPwm.write(0.0f);
      MotorBPwm.write(Speed);
    }
    else if(EncCountA > EncCountB){
      MotorAPwm.write(Speed);
      MotorBPwm.write(0.0f);
    }
    else if (EncCountA == EncCountB){
      MotorAPwm.write(Speed);
      MotorBPwm.write(Speed);
      BothEncCount = EncCountA;
    }
  }

  stop(); //stop
  float DistanceTravelled = FindDistanceTravelled();
  Serial.println(DistanceTravelled);
  UpdatePosition(); //Updates the new position of the robot on cartesian coordinate grid.
  AddToMap(DistanceTravelled);
  UpdatePosition();
}

/*-----------------------------------------------------------------------------------------------

/*Subsection 3.3: Stopping*/
/* either set of commands should short stop the motors and thus could be optimised by taking one of the lines out,
however, i like having both to make absolutely certain that the motors will be stopped so i'm fine with 2 lines of inefficiency.*/

/*-----------------------------------------------------------------------------------------------*/

void stop(){ 
  MotorAPwm = 0;
  MotorBPwm = 0;
  MotorAPwm.write(0.0f);
  MotorBPwm.write(0.0f);
}

/*-----------------------------------------------------------------------------------------------

/*Subsection 3.4: Turning.*/

/*-----------------------------------------------------------------------------------------------*/
void TurnLeft(){ //Turns 90 degrees left.
  EncCountA = 0; //resets the encoders
  EncCountB = 0;
  MotorADir = 1; //motor A goes backwards
  MotorBDir = 1; // motor B goes Forwards.

  while(EncCountA > -155){ //90mm between the wheels. want to trace out 1/4 of a circle with radius 45mm. circumferance = 2pi *r. therefore target distance is 70.68583471mm. Multiply by 2.18383082 for desired encoder value (154). Shave some off to account for inertia.

    EncA.rise(&countPulseA); //update encoders
    EncB.rise(&countPulseB);

    if (EncCountA < -EncCountB){ //ensures that both wheels are turning at the same rate in opposite directions. if one has turned farther it stops for a moment to let the other catch up.
      MotorAPwm.write(0.0f);
      MotorBPwm.write(0.50f);
    }
    else if(EncCountA > -EncCountB){
      MotorAPwm.write(0.50f);
      MotorBPwm.write(0.0f);
    }
    else if (EncCountA == -EncCountB){
      MotorAPwm.write(0.50f);
      MotorBPwm.write(0.50f);
      BothEncCount = EncCountA;
    }
  }
  stop();
  //Determine new direction that robot is facing and update global variable.
  if(Localisation.GetDirection() == "West"){
    Localisation.SetDirection("South");
  }
  else if(Localisation.GetDirection() == "North"){
    Localisation.SetDirection("West");
  }
  else if(Localisation.GetDirection() == "East"){
    Localisation.SetDirection("North");
  }
  else if(Localisation.GetDirection() == "South"){
    Localisation.SetDirection("East");
  }
  AddToMap(Left);
}


void TurnRight(){
  EncCountA = 0; //reset encoders.
  EncCountB = 0;
  MotorADir = 0; //motor A goes forwards
  MotorBDir = 0; //motor B goes backwards

  while(EncCountB > -155){ //90mm between the wheels. want to trace out 1/4 of a circle with radius 45mm. circumferance = 2pi *r. therefore target distance is 70.68583471mm. Multiply by 2.18383082 for desired encoder value (154). Shave some off to account for inertia.

    EncA.rise(&countPulseA); //update encoders
    EncB.rise(&countPulseB);

    if (EncCountB < -EncCountA){ //see TurnLeft explaination but in reverse.
      MotorBPwm.write(0.0f);
      MotorAPwm.write(0.50f);
    }
    else if(EncCountB > -EncCountA){
      MotorBPwm.write(0.50f);
      MotorAPwm.write(0.0f);
    }
    else if (EncCountB == -EncCountA){
      MotorBPwm.write(0.50f);
      MotorAPwm.write(0.50f);
      BothEncCount = EncCountA;
    }
  }
  stop();
  //Determine new direction that robot is facing and update global variable.
  if(Localisation.GetDirection() == "West"){
    Localisation.SetDirection("North");
  }
  else if(Localisation.GetDirection() == "North"){
    Localisation.SetDirection("East");
  }
  else if(Localisation.GetDirection() == "East"){
    Localisation.SetDirection("South");
  }
  else if(Localisation.GetDirection() == "South"){
    Localisation.SetDirection("West");
  }
  AddToMap(Right);
}
/*-----------------------------------------------------------------------------------------------

/*Subsection 3.5: Squaring up to walls.*/

/*-----------------------------------------------------------------------------------------------*/

void SquareUpLeft() { //Squares up with left wall. Squares up perfectly so long as is less than 13 cm away from the wall. //maybe add a check for if it is more than a certain distance 
//to avoid those times where it squares instead of turning left
  Square = false;
  TimeOut = 0; //timeout is used to limit the time the robot can spend trying to square up so that it doesn't get caught in a loop when squaring isn't possible.
  while (Square == false) {
    float FL = IRSensor(FrontLeft); //check front left IR sensor.
    float BL = IRSensor(BackLeft); //check back left IR sensor
    //check whether angle is within acceptable varience (if the sensors are within the offset value of each other then its more or less straight)
    if (FL > BL + OffSet) { // check is the front sensor is further from the wall than the back sensor. (if its square then they should be roughly the same)
      EncCountA = 0; 
      EncCountB = 0;
      MotorADir = 1;
      MotorBDir = 1;
      EncA.rise(&countPulseA);
      EncB.rise(&countPulseB);

      if (EncCountA < -EncCountB) { //see TurnLeft explaination.
        RampSpeedB(0.0f);
        RampSpeedA(SquareSpeed);
      } else if (EncCountA > -EncCountB) {
        RampSpeedB(SquareSpeed);
        RampSpeedA(0.0f);
      } else if (EncCountA == -EncCountB) {
        RampSpeedB(SquareSpeed);
        RampSpeedA(SquareSpeed);
        BothEncCount = EncCountA;
      }
    }


    else if (BL > FL + OffSet) { // check is the back sensor is further from the wall than the front sensor. (if its square then they should be roughly the same)
      EncCountA = 0;
      EncCountB = 0;
      MotorADir = 0;
      MotorBDir = 0;
      EncA.rise(&countPulseA);
      EncB.rise(&countPulseB);
      if (EncCountB < -EncCountA) { //see RightTurn explaination.
        RampSpeedA(0.0f);
        RampSpeedB(SquareSpeed);
      } else if (EncCountB > -EncCountA) {
        RampSpeedA(SquareSpeed);
        RampSpeedB(0.0f);
      } else if (EncCountB == -EncCountA) {
        RampSpeedA(SquareSpeed);
        RampSpeedB(SquareSpeed);
        BothEncCount = EncCountA;
      }
    } else { //it is within the acceptable range.
      stop();
      EncCountA = BothEncCount;
      EncCountB = BothEncCount;
      Square = true; //confirm that it is square to exit the while loop
    }
    TimeOut = TimeOut + 1; //for every cycle of the previous code where the while loop isn't resolved, increment timout counter by one.
    //Serial.print(TimeOut);
    if(TimeOut > 500){ //when while loop hasn't resolved fast enough, exit the squareup and attempt to go forwards a little bit to get out of the deadzone before trying again
      GoThisFar(10.0);
      SquareUpLeft();
    }
  }
  AddToMap(SquareLeft);
}

void SquareUpRight() {  //same as SquareUpLeft but on the right. Directions and comparisons are reversed.
  float FR = IRSensor(FrontRight);
  float BR = IRSensor(BackRight);
  Square = false;
  TimeOut = 0; //timeout is used to limit the time the robot can spend trying to square up so that it doesn't get caught in a loop when squaring isn't possible.

  while (Square == false) {
    float FR = IRSensor(FrontRight);
    float BR = IRSensor(BackRight);
    if (FR > BR + OffSet) {
      EncCountA = 0;
      EncCountB = 0;
      MotorADir = 0;
      MotorBDir = 0;

      EncA.rise(&countPulseA);
      EncB.rise(&countPulseB);

      if (EncCountB < -EncCountA) {
        RampSpeedB(0.0f);
        RampSpeedA(SquareSpeed);
      } else if (EncCountB > -EncCountA) {
        RampSpeedB(SquareSpeed);
        RampSpeedA(0.0f);
      } else if (EncCountB == -EncCountA) {
        RampSpeedB(SquareSpeed);
        RampSpeedA(SquareSpeed);
        BothEncCount = EncCountA;
      }
    }


    else if (BR > FR + OffSet) {
      EncCountA = 0;
      EncCountB = 0;
      MotorADir = 1;
      MotorBDir = 1;

      EncA.rise(&countPulseA);
      EncB.rise(&countPulseB);
      if (EncCountA < -EncCountB) {
        RampSpeedA(0.0f);
        RampSpeedB(SquareSpeed);
      } else if (EncCountA > -EncCountB) {
        RampSpeedA(SquareSpeed);
        RampSpeedB(0.0f);
      } else if (EncCountA == -EncCountB) {
        RampSpeedA(SquareSpeed);
        RampSpeedB(SquareSpeed);
        BothEncCount = EncCountA;
      }
    } else {
      MotorAPwm.write(0.0f);
      MotorBPwm.write(0.0f);
      EncCountA = BothEncCount;
      EncCountB = BothEncCount;
      Square = true;
    }

    TimeOut = TimeOut + 1; //for every cycle of the previous code where the while loop isn't resolved, increment timout counter by one.
    if(TimeOut > 500){ //when while loop hasn't resolved fast enough, exit the squareup and attempt to go forwards a little bit to get out of the deadzone before trying again
      GoThisFar(10.0);
      SquareUpLeft();
    }
  }
  AddToMap(SquareRight);
}


void SquareUp(){ //select whether to square up on the left wall or the right wall based on which one is closer.
  int LeftWall = (AverageIRSensor(FrontLeft) + AverageIRSensor(BackLeft))/2;
  int RightWall = (AverageIRSensor(FrontRight) + AverageIRSensor(BackRight))/2;
  if(LeftWall > RightWall){ //if right wall is closer
    SquareUpRight();
  }
  else if(RightWall > LeftWall){ //if left is closer
    SquareUpLeft();
  }
  else{ //if they're equal distance
    SquareUpLeft(); //favour the left hand wall
  }
}

void RampSpeedA(float SetSpeed){ //function to slowly increase the speed of motor A to a desired final speed.
  float CurrentSpeed = MotorAPwm.read(); //find the current PWM
  if(CurrentSpeed < SetSpeed){ //compare whether current speed is above or below the desired value 
    CurrentSpeed = CurrentSpeed + Ramp; // if below desired value, increase PWM by the ramp value.
    MotorAPwm.write(CurrentSpeed); //assign new speed.
  }
  else if(CurrentSpeed > SetSpeed){
    CurrentSpeed = CurrentSpeed - Ramp; //if above desired value, decrease PWM by the ramp value
    MotorAPwm.write(CurrentSpeed); //Assign new speed
  }
  else{
    MotorAPwm.write(CurrentSpeed); //if current speed is already the desired speed, just maintain that PWM.
  }
}

void RampSpeedB(float SetSpeed){ //Same as previous function but for wheel B.
  float CurrentSpeed = MotorBPwm.read();
  if(CurrentSpeed < SetSpeed){
    CurrentSpeed = CurrentSpeed + Ramp;
    MotorBPwm.write(CurrentSpeed);
  }
  else if(CurrentSpeed > SetSpeed){
    CurrentSpeed = CurrentSpeed - Ramp;
    MotorBPwm.write(CurrentSpeed);
  }
  else{
    MotorBPwm.write(CurrentSpeed);
  }
}



/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Segment 4*/
/*Functions for the celebration that signifies that the robot knows it has reached the end of the maze*/

/*Code in this section includes: 
-Doing a big spin :)
-Flashing the built-in RGB lights on the board. It isn't very vision because of how the robot is made but it adds style.

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/


void VictorySpin(){ //calls when reaching the end of the maze to show that the robot knows that it has finished. Does 2 full spins to the right and then 2 to the left.
  for (int i = 0; i <= 8; i++){
    TurnRight();
    LightShow();
  }
  // delay(500);
  // for (int i = 0; i <= 8; i++){
  //   TurnLeft();
  //   LightShow();
  // } //ends facing same direction it started in.
}

void LightShow(){
  digitalWrite(LEDR, LOW); //Turn on Red light
  delay(300);
  digitalWrite(LEDR, HIGH); // Turn off Red light
  digitalWrite(LEDG, LOW); //Turn on Green light
  delay(300);
  digitalWrite(LEDR, HIGH);// Turn off Green light
  digitalWrite(LEDB, LOW); //Turn on Blue light
  delay(300);
  digitalWrite(LEDB, HIGH);// Turn off Blue light
}

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Segment 5*/
/*Main Control loop for getting from the start of the maze to the end of the maze*/

/*Summary of funtion: 
- Checks whether or not the robot has reached the end of the maze.
-if not, squares up with the nearest wall.
-checks the front, left, and right sensors.
-if there not a wall on the left, turn left and go forwards a bit.
-if there is a wall on the left and not one in front, go forwards.
-if there is a wall on the left and in front, turn right.
-if there is a wall on all 3 sides, just turn around fully.
-the code can unfortunately be quite wordy as i had to deal with a lot of edge cases.

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

void Explore(){
  float TestX = Localisation.GetX();
  float TestY = Localisation.GetY();
  Serial.println((String) "X:" + TestX + "   Y:" + TestY);
  Serial.println("Snitch 1");

  while(true){
    if(TestY > 1150){
      Serial.println("Snitch 23");
      break; // breaks out of the while loop if the robot has already reached the end.
    }
    Serial.println("Snitch 2");
    stop();
    float TestX = Localisation.GetX();
    float TestY = Localisation.GetY();
    Serial.println((String) "X:" + TestX + "   Y:" + TestY);
    Serial.println("Snitch 3");
    SquareUp();
    Serial.println("Snitch 4");

    int LeftWall = UltraSensor(Ultra4Pin); //check if there is a wall on the left using ultrasonic.
    int FrontWall = CheckFront();
    int RightWall = UltraSensor(Ultra2Pin); //check if there is a wall on the right using ultrasonic.
    Serial.println("Snitch 5");
    if (FrontWall < 13 || FrontWall > 600){
      Serial.println("Snitch 6");
      ReverseThisFar(10.0);
    }
    else{
      if(LeftWall > 30 && LeftWall < 900){ //check if there is a wall on the left using ultrasonic. there isn't
        Serial.println("Snitch 7");
        float FL = AverageIRSensor(FrontLeft); //check front left ir
        float BL = AverageIRSensor(BackLeft); //check back left ir
        if(FL > 30 && BL > 30){ //check whether both sensors confirm there is no wall
          Serial.println("Snitch 8");
          TurnLeft(); //if there is a wall close in front, do a small turn to avoid driving into it.
          GoThisFar(340.0); //go forwards 10 cm and check again
          Serial.println("Snitch 8.5");
        }
        else if((FL > 20 && BL < 20) || (FL <20 && BL > 20)){ //is there is a wall in front of one sensor but not the other
          if(FrontWall > 10 && FrontWall < 900){ //check if there is a wall close in front
            Serial.println("Snitch 9");
            GoThisFar(50.0); //go 5 cm forwards
          }
          else if(FrontWall <= 13){
            Serial.println("Snitch 10 - uncommon");
            ReverseThisFar(10.0);
          }
          else{
            Serial.println("Snitch 11 - uncommon");
            TurnRight();
          }
        }
        else{ //there actually is a wall on the left, the ultrasonic just failed.
          Serial.println("Snitch 12. Does this ever call?");
          SquareUp();
          if(FrontWall > 13 && FrontWall < 900){ //check if there is a wall close in front
            GoThisFar(100.0f); //go forwards 10 cm and check again
          }
          else if(FrontWall <= 14 || FrontWall > 900){
            TurnRight();
          }
        }
      }
      else if(LeftWall <= 20 || LeftWall >900){ //check if there is a wall on the left using ultrasonic. there is
        Serial.println("Snitch 13");
        if(FrontWall >= 16 && FrontWall < 900){ //check if there is a wall close in front. there isn't
          Serial.println("Snitch 14");
          GoThisFar(100.0f); //go forwards 10 cm and check again
        }
        else if(FrontWall < 16){ //check if there is a wall 16cm in front. there is
          Serial.println("Snitch 15");
          if(RightWall > 20 && RightWall <900){ //check if there is a wall on the Right using ultrasonic. there isn't
            Serial.println("Snitch 16");
            float FR= AverageIRSensor(FrontRight); //check front Right ir
            float BR= AverageIRSensor(BackRight); //check back Right ir
            if(FR> 20 && BR> 20){ //check whether both sensors confirm there is no wall
              Serial.println("Snitch 17");
              TurnRight();
            }
            else if(FR> 20 && BR< 20){ //if there is a wall in front of back sensor but not front. I don't think this can even happen but just in case.
              Serial.println("Snitch 18 if this one pings idek");
              if(FrontWall > 10 && FrontWall < 900){ //check if there is a wall close in front
                GoThisFar(50.0); //go 5 cm forwards
              }
            }
            else if (FR<20 && BR> 20){ //if there is a wall in front of front sensor but not back
              Serial.println("Snitch 19 more likely but still don't think it can happen");
              ReverseThisFar(50.0); 
            }
            else{
              Serial.println("Snitch 20");
              if(FrontWall >= 16 && FrontWall < 900){ //check if there is a wall close in front
                Serial.println("Snitch 21");
                GoThisFar(100.0f); //go forwards 10 cm and check again
                SquareUp(); //Square up with the closest wall
              }
            }
          }
          else{ //there is a wall on the left, the front and the right
            TurnRight();
            TurnRight();
          }
        }
      }
    }
    Serial.println("Snitch 22");
  }
  VictorySpin();
  ReadMap();
  CreateMapBack();
  
}


/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Segment 6*/
/*Mapping and localisation functions*/

/*Code in this section includes: 
-keeping track of the robot's position (x,y).
-adding a move to the the array that keeps track of moves to the end.
-adding a move to the array that plans the route back from the end to the start.
-printing out the map from the start to the end, mostly for debugging purposes.
-creating the map back from the end to the start by reversing the moves taken to get there and removing redundancies (such as areas where reversing was necessary).
-Following the map back to the start by performing the corralating actions.

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Start Facing West
void UpdatePosition(){
  float DistanceTravelled = FindDistanceTravelled();
  float XCurrent = Localisation.GetX(); //get the current x position on cartesian.
  float YCurrent = Localisation.GetY(); //get the current Y position on cartesian.
  //decide whether x or y is being changed depending on what direction the robot is facing. Only moves in straight lines and turn at 90 degrees so only one should change at a time.
  if(Localisation.GetDirection() == "West"){ //if the robot is facing "West" (West is defined as being facing towards the left hand of the maze, not based earth's poles)
    float NewX = XCurrent - DistanceTravelled; //work out the new value of x
    Localisation.UpdateX(NewX); //update global variable tracking x coordinate
  }
  else if(Localisation.GetDirection() == "North"){ //see above
    float NewY = YCurrent + DistanceTravelled;
    Localisation.UpdateY(NewY);
  }
  else if(Localisation.GetDirection() == "East"){ //see above
    float NewX = XCurrent + DistanceTravelled;
    Localisation.UpdateX(NewX);
  }
  if(Localisation.GetDirection() == "South"){ //see above
    float NewY = YCurrent - DistanceTravelled;
    Localisation.UpdateY(NewY);
  }

}

void AddToMap(float NewMove){ //adds a new move to the map on the way to the end.
  Serial.println("Map Snitch 1");
  MovesToEnd[CurrentMove] = NewMove;
  CurrentMove = CurrentMove + 1;
  Serial.println("Map Snitch 2");
}
void AddToReverseMap(float NewMove){ //adds a new move to the the map on the way back to the start.
  Serial.println("Map Snitch 3");
  MovesBackToStart[MapReversalCounter] = NewMove;
  MapReversalCounter = MapReversalCounter + 1;
  Serial.println("Map Snitch 4");
}

void ReadMap(){
  Serial.println("Map Snitch 5");
  for (byte i = 0; i < 149; i = i + 1) { //goes through each move in the array and prints them out on the serial monitor.
    Serial.println("Map Snitch 6");
    if(MovesToEnd[i] == Left){
      Serial.println("Left");
    }
    else if(MovesToEnd[i] == Right){
      Serial.println("Right");
    }
    else if(MovesToEnd[i] == SquareLeft){
      Serial.println("Squareup Left");
    }
    else if(MovesToEnd[i] == SquareRight){
      Serial.println("Squareup Right");
    }
    else if(MovesToEnd[i] < 0){ // reversing is signified by a negative value.
      Serial.print("Reverse");
      Serial.println(MovesToEnd[i]);
    }
    else{
      Serial.print("Forward"); // a positive value that is not equal to Left, Right, SquareLeft or SquareRight is a forwards command.
      Serial.println(MovesToEnd[i]);
    }
  }
}

void CreateMapBack(){ //function that reverses the map from start to end to create a map from end to start.
  Serial.println("Map Snitch 7");
  for (byte i = 149; i > 0; i = i - 1){
    Serial.println("Map Snitch 8");
    if(MovesToEnd[i] == Left){ //if it made a left on the way there
      AddToReverseMap(Right); //make a right on the way back
    }
    else if(MovesToEnd[i] == Right){ //if it made a right on the way there
      AddToReverseMap(Left); //make a left on the way back
    }
    else if(MovesToEnd[i] == SquareLeft){ //when it squares up on the left
      AddToReverseMap(SquareRight); //square up on the right on the way back
    }
    else if(MovesToEnd[i] == Right){ //when it squares up right
      AddToReverseMap(Left); //square up on the left on the way back
    }
    else if(MovesToEnd[i] < 0){ //when it is reversing
      for (byte j = 1; j < 99; j = j + 1){ //this loop finds the next forwards and then subtracts the reverse from it.
        if(MovesToEnd[i-j] != Left && MovesToEnd[i-j] != Right && MovesToEnd[i-j] != SquareLeft && MovesToEnd[i-j] != SquareRight){ 
          MovesToEnd[i-j] = MovesToEnd[i-j] + MovesToEnd[i];
          break; //exits the for loop
        }
      }
    }
    else{ //it's going fowards
      AddToReverseMap(MovesToEnd[i]); //go forwards that far
    }
  }
  FollowMapBack(); //call the function to follow the map once the map is complete.
}

void FollowMapBack(){ //checks each term in the map back to the start and executes relevant command.
  Serial.println("Map Snitch 9 - sureley not???");
  TurnRight();
  TurnRight();
  for (byte i = 0; i < 149; i = i + 1){
    if(MovesBackToStart[i] == Left){
      TurnLeft();
    }
    else if(MovesBackToStart[i] == Right){
      TurnRight();
    }
    else if(MovesBackToStart[i] == SquareLeft){
      SquareUpLeft();
    }
    else if(MovesBackToStart[i] == SquareRight){
      SquareUpRight();
    }
    else{
      GoThisFar(MovesBackToStart[i]);
    }
  }
  VictorySpin();
  while(true){
    sleep();
  }
}


/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Segment 7*/
/*Setup and loop*/

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/



void setup() {
  Serial.begin(9600); //start serial connection, used for debugging.
  Serial.print("Start Test /n");

  MotorAPwm.period_ms(1); //set the motor period for motors A and B.
  MotorBPwm.period_ms(1);  
  MotorAPwm.write(0.0f); //Ensuring the motors start out off.
  MotorBPwm.write(0.0f);

  EncCountA = 0; //Ensuring the encoder counts start at 0.
  EncCountB = 0;
  BothEncCount = 0;


  Localisation.SetDirection("West"); //assign the starting direction (starts facing in the negative x direction) as West.

  pinMode(LEDR, OUTPUT);  // pins for the RGB lights.
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  delay(1000);  

}



void loop() {

  //VictorySpin();
  Explore();

  //delay(5000);
}
