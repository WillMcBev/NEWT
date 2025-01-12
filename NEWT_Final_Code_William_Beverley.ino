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


/*----------------------------------------------------------------------------------------------------------------------------------------------------------*//*

Due to there being issues with Arduino IDE around the beginning of the project, the code has not been split into headers and a cpp file despite that being 
technically better practice. As much as I would have preferred to use headers and have everything more organised, I would prefer even more for things
to actually work. 

Therefore, to try and make everything easier to understand; the code for this project has been split into 7 segments (as I would have split it for header files):
Segment 1: Variables, pins, and constraints 
Segment 2: Sensors
Segment 3: Movement (includes further segregation into sub parts)
Segment 4: End of Maze Celebration
Segment 5: Exploration loop for finding the way to the end of the maze
Segment 6: Mapping and localisation
Segment 7: Setup() and loop()

This code has been shown to work in my bedroom as of the 12/01/2025. I did not manage to fully test the final draft in the maze before the submission time unfortunately
(it was 99% working last time it was in the maze, there was just one bug i had to fix to get it working at home. I see no reason that it wouldn't now work in the maze.)
/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Segment 1*/
/*Initialising variables,pins, and constants.*/

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include "mbed.h"
using namespace mbed;

// Motors:

/* Motor A direction 0 is forwards. Motor A is Left Wheel
   Motor B Direction 1 is forwards. Motor B is Right Wheel*/
mbed::DigitalOut MotorADir(P0_4); // Set direction pin for motor A
mbed::DigitalOut MotorBDir(P0_5); // Set direction pin for motor B
mbed::PwmOut MotorAPwm(P0_27); // Set PWM pin for motor A
mbed::PwmOut MotorBPwm(P1_2); // Set PWM pin for motor B

// Encoders and distancing:

mbed::InterruptIn EncA(P1_11); // Left wheel encoder pin declaration
long int ShaftRevA; // Stores how many full rotations of motor A there have been
long int EncCountA; // Stored number of pulses of motor A encoder

mbed::InterruptIn EncB(P1_12); // Left wheel encoder pin declaration
long int ShaftRevB; // Stored how many full rotations of motor B there have been
long int EncCountB; // Stores numer of pulses of motor B encoder

long int BothEncCount; // Increments when both wheels have moved equal distance.

float Distance; // Used to find the distance travelled of a wheel in the distance function
float TargetDist; // Target Distance for the GoThisFar function.
const float ENCODER_TICKS_PER_MM = 2.18383082; // Every 2.18383082 encoder ticks represents one mm of movement.

// Speed and squareup related:

const float SPEED = 0.5f; // General motor speed unless squaring up.
const float RAMP = 0.005; // Ramp for the ramp up function.
const float SQUARE_SPEED = 0.35f; // Speed during SquareUp functions.
bool Square; // Used to check whether the robot is currently square with a wall.
int StuckCounter = 0; // Used to check if the robot is stuck.
int TimeOut = 0; // Used to stop the robot from getting stuck in a squareup loop when squaring isn't possible.

// Ultrasonic Sensor Pins:

const int ULTRA_1_PIN = 7;
int Ultra1Dist = 0;
const int ULTRA_2_PIN = 4;
int Ultra2Dist = 0;
const int ULTRA_3_PIN = 5;
int Ultra3Dist = 0;
const int ULTRA_4_PIN = 6;
int Ultra4Dist = 0;

int UltraPin;
int FrontDist = 0;

// IR sensor variables:

const float OFFSET = 0.27; // Acceptable varience in sensors for squareup.
mbed::I2C i2c(P0_31, P0_2); // Pin declaration for I2C.
const char BACK_LEFT = 0x01;
const char BACK_RIGHT = 0x02;
const char FRONT_RIGHT = 0x04;
const char FRONT_LEFT = 0x08;

// Localisation:

// Class will be used for global variables for localisation so that values can be passed between functions easier.
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

// Mapping:

float MovesToEnd[200] = {}; // Array for storing what moves the robot took to get to the end
float MovesBackToStart[200] = {}; // Array for reversing the moves taken in order to get back to the start
int CurrentMove = 0; // Stores how many moves the robot has taken so far
int MapReversalCounter = 0; // Stores how many moves have been reversed so far
//  Since array is storing floats, unreasonable values will be used to store the commands for a left turn, right turn, square up or reverse. 
// 2000 will be a Left Turn. 3000 will be a right turn. 4000 will be squaring up on the left wall. 5000 will be squaring up on the right wall.
const int LEFT = 2000;
const int RIGHT = 3000;
const int SQUARE_LEFT = 4000;
const int SQUARE_RIGHT = 5000;
bool Exploring = true;

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
  const char MUX_ADDR = 0xEE; // Multiplexer address.
  i2c.write(MUX_ADDR, &mux_cmd, 1);
  char cmd[2]; // Character array for IR sensors.
  cmd[0] = 0x5E;
  cmd[1] = 0x00;
  i2c.write(0x80, cmd, 1); 
  i2c.read(0x80, cmd, 2);
  float distance = (((float)(cmd[0] << 4) + (float)cmd[1])/16.0)/4.0; // Bit shift to get actual distances from it. cast as a float to get mm.
  return distance;

}

float AverageIRSensor(char Sensor){ // Take 25 readings from the IR sensor and average them. This will help to minimise the impact of any bad readings.
  float Total = 0.0;
  for (int i = 0; i <= 24; i++) { // Loop 25 times
    float Reading = IRSensor(Sensor); // Take a reading
    Total = Total + Reading; // Add that reading to the total of all previous readings.
  }
  float AverageReading = Total / 25.0; // Take the average
  return AverageReading;
  
}

/* Ultrasonic sensor code adapted from the arduino ide example code for the Ping ultrasonic sensor (File -> Examples -> Sensors -> Ping)
created 3 Nov 2008 by David A. Mellis
modified 30 Aug 2011 by Tom Igoe

Also accessible at https://docs.arduino.cc/built-in-examples/sensors/Ping/

This example code is in the public domain. */

long UltraSensor(int UltraPin){
  // Establish variables for duration of the sensor, and the distance result in centimeters:
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
  // Whose duration is the time (in microseconds) from the sending of the ping
  // To the reception of its echo off of an object.
  pinMode(UltraPin, INPUT);
  duration = pulseIn(UltraPin, HIGH);

  // Convert the time into a distance
  cm = duration / 29 / 2;
  delay(100);
  return cm;

}

int CheckFront(){
  int LeftFront = UltraSensor(ULTRA_1_PIN); // Check if there is a wall in front using ultrasonic.
  delay(200);
  int RightFront = UltraSensor(ULTRA_3_PIN); // Check if there is a wall in front using ultrasonic
  if(LeftFront <= 20 && RightFront <= 20){
    int FrontDiff = abs(LeftFront - RightFront); // Find the difference between the 2 values
    if(FrontDiff < 8){ // See if the values are similar, they are (difference betweeen them is low)
      float FrontDist = (LeftFront + RightFront)/2;

      return FrontDist;
    }
    else{ // The values aren't similar so at least one of them is very off
      ReverseThisFar(10.0); // Move a little and try again.
      CheckFront();
    }
  }
  else if(LeftFront < 20 && RightFront > 20 && RightFront <= 900){ // If the front left sensor is saying there is a wall close by and the front right says there isn't
    FrontDist = LeftFront; // Ignore the front right and act as if there is a wall
    return FrontDist;
  }
  else if(RightFront < 20 && LeftFront > 20 && LeftFront <= 900){ // If there is a wall in front of right but not left.
    FrontDist = RightFront; // Ignore front left and act as if there is a wall.
    return FrontDist;
  }
  else{ // Both values are over 20, there isn't a wall nearby 
    FrontDist = LeftFront; // Doesn't matter which one you use since result is the same. Use Left (picked because the robot is more likely to get caught on the left side.)
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

/*-----------------------------------------------------------------------------------------------*/

/*Subsection 3.1: Encoders*/

/*-----------------------------------------------------------------------------------------------*/

void countPulseA(){ // Counts the encoder pulses of wheel A.
   if(MotorADir == 0) // Allows encoders to be directional.
     EncCountA++;
   else
     EncCountA--;
  countRevA(); // Calls function that tracks revolutions.
}
void countPulseB(){ // Same as previous function but for wheel B.
   if(MotorBDir == 1) // Direction is flipped because the motor is facing a different way. Therefore "1" is in opposite directions for each motor.
     EncCountB++;
   else
     EncCountB--;
  countRevB();
}

void countRevA(){ // Counts revolutions of wheel A.  330 encoder ticks per revolution.
   if(EncCountA%6 == 0){ // Checks if encoder A's value is a multiple of 6.
     if(EncCountA%110 == 0){ // Checks if its a multiple of 100 as well.
       ShaftRevA++; // Increments the revolution count.
     }
   }
}
void countRevB(){// Same as CountRevA but for wheel B.
   if(EncCountB%6 == 0){
     if(EncCountB%110 == 0){
       ShaftRevB++;
     }
   }
}

float DistanceA(){ // Finds the distance travelled by wheel A.
  return Distance = EncCountA / ENCODER_TICKS_PER_MM; // 660 / (48.1 * pi *2) = ENCODER_TICKS_PER_MM which is the encoder ticks per mm of distance travelled.
}

float DistanceB(){ // Same as above but for B.
  return Distance = EncCountB / ENCODER_TICKS_PER_MM;
}

float FindDistanceTravelled(){
  float A = DistanceA();
  float B = DistanceB();
  float DistanceForBoth = (A + B) / 2; // Find how far the robot has actually moved using encoders
  return DistanceForBoth;
}

/*-----------------------------------------------------------------------------------------------

/*Subsection 3.2: Straight line movements forwards and backwards.*/

/*GoThisFar and ReverseThisFar could in theory be merged but that may cause issues with mapping and adds a higher level of complication
than I feel is necessary.*/

/*-----------------------------------------------------------------------------------------------*/

void GoThisFar(float TargetDist){ // Function to move in a straight line for a set distance (in millimeters)
  EncCountA = 0; // Reset encoder values
  EncCountB = 0;
  BothEncCount = 0;
  while(BothEncCount < (TargetDist * ENCODER_TICKS_PER_MM)){ // 2.18... is the number of encoder ticks per mm. multiplying by the number of mm the robot should go gives the final encoder count the robot should have
    EncA.rise(&countPulseA); // Update encoders
    EncB.rise(&countPulseB);
    MotorADir = 0; // Assign wheel directions to go forwards.
    MotorBDir = 1;

    if (EncCountA > EncCountB){ // Comparisons to make sure one wheel doesn't travel faster than the other. when one wheel overtakes the other, it is momentarily turned off so the other can catch up.
      MotorAPwm.write(0.0f);
      MotorBPwm.write(SPEED);
    }
    else if(EncCountA < EncCountB){
      MotorAPwm.write(SPEED);
      MotorBPwm.write(0.0f);
    }
    else if (EncCountA == EncCountB){
      MotorAPwm.write(SPEED);
      MotorBPwm.write(SPEED);
      BothEncCount = EncCountA; // When both encoder counts are equal, the robot must have travelled that far and can be compared with the target encoder value.
    }
    if(Exploring == true){
      CheckIfStuck(); // Check whether the wheels are actually moving or not.
    }
  }

  stop(); // Stop
  float DistanceTravelled = FindDistanceTravelled();
  UpdatePosition(); // Updates the new position of the robot on cartesian coordinate grid.
  AddToMap(DistanceTravelled);
}

void CheckIfStuck(){
  float EncA1 = EncCountA; // Take a reading of the encoders
  float EncB1 = EncCountB;
  delay(1); // Wait
  EncA.rise(&countPulseA);
  EncB.rise(&countPulseB);
  float EncA2 = EncCountA; // Take a second reading of the encoders
  float EncB2 = EncCountB;
  if(EncA2 == EncA1 && EncB2 == EncB1){ // If the encoder values haven't changed
    StuckCounter = StuckCounter + 1; // Robot might be stuck so increment stuck counter
  }
  else{ // Encoder values have changed
    StuckCounter = 0;  // Robot isn't stuck so reset stuck counter.
  }
  if(StuckCounter >= 1000){ // Check whether the stuck counter is over 1000. for it to be this high the robot can't have moved in a while. indicating that it is stuck.
    ReverseThisFar(20); // Reverse 20 mm
    SquareUp(); // Squareup on the left
    StuckCounter = 0; // Reset the stuck counter
    Explore(); // Resume exploration
  }
}

void ReverseThisFar(float TargetDist){ // Same as GoThisFar but in reverse. Directions are switched and desired encoder value is now negative.
  // Does not call check if stuck function because reversing is how it gets unstuck.
  EncCountA = 0;
  EncCountB = 0;
  BothEncCount = 0;
  while(BothEncCount > -(TargetDist * ENCODER_TICKS_PER_MM)){
    EncA.rise(&countPulseA);
    EncB.rise(&countPulseB);
    MotorADir = 1;
    MotorBDir = 0;

    if (EncCountA < EncCountB){
      MotorAPwm.write(0.0f);
      MotorBPwm.write(SPEED);
    }
    else if(EncCountA > EncCountB){
      MotorAPwm.write(SPEED);
      MotorBPwm.write(0.0f);
    }
    else if (EncCountA == EncCountB){
      MotorAPwm.write(SPEED);
      MotorBPwm.write(SPEED);
      BothEncCount = EncCountA;
    }
  }

  stop(); // Stop
  float DistanceTravelled = FindDistanceTravelled();
  UpdatePosition(); // Updates the new position of the robot on cartesian coordinate grid.
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

void TurnLeft(){ // Turns 90 degrees left.
  EncCountA = 0; // Resets the encoders
  EncCountB = 0;
  MotorADir = 1; // Motor A goes backwards
  MotorBDir = 1; // Motor B goes Forwards.

  while(EncCountA > -155){ //90mm between the wheels. want to trace out 1/4 of a circle with radius 45mm. circumferance = 2pi *r. therefore target distance is 70.68583471mm. Multiply by ENCODER_TICKS_PER_MM for desired encoder value (154). Shave some off to account for inertia.

    EncA.rise(&countPulseA); // Update encoders
    EncB.rise(&countPulseB);

    if (EncCountA < -EncCountB){ // Ensures that both wheels are turning at the same rate in opposite directions. if one has turned farther it stops for a moment to let the other catch up.
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
  // Determine new direction that robot is facing and update global variable.
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
  AddToMap(LEFT);
}


void TurnRight(){
  EncCountA = 0; // Reset encoders.
  EncCountB = 0;
  MotorADir = 0; // Motor A goes forwards
  MotorBDir = 0; // Motor B goes backwards

  while(EncCountB > -155){ //90mm between the wheels. want to trace out 1/4 of a circle with radius 45mm. circumferance = 2pi *r. therefore target distance is 70.68583471mm. Multiply by ENCODER_TICKS_PER_MM for desired encoder value (154). Shave some off to account for inertia.

    EncA.rise(&countPulseA); // Update encoders
    EncB.rise(&countPulseB);

    if (EncCountB < -EncCountA){ // See TurnLeft explaination but in reverse.
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
  // Determine new direction that robot is facing and update global variable.
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
  AddToMap(RIGHT);
}

/*-----------------------------------------------------------------------------------------------

/*Subsection 3.5: Squaring up to walls.*/

/*-----------------------------------------------------------------------------------------------*/

void SquareUpLeft() { // Squares up with left wall. Squares up perfectly so long as is less than 13 cm away from the wall.
  Square = false;
  TimeOut = 0; // Timeout is used to limit the time the robot can spend trying to square up so that it doesn't get caught in a loop when squaring isn't possible.
  while (Square == false) {
    float FL = IRSensor(FRONT_LEFT); // Check front left IR sensor.
    float BL = IRSensor(BACK_LEFT); // Check back left IR sensor
    if(FL > 20 && BL > 20){ // To avoid it squaring itself around a corner and messing up odometry.
      Square = true;
    }
    else{
      // Check whether angle is within acceptable varience (if the sensors are within the offset value of each other then its more or less straight)
      if (FL > BL + OFFSET) { // Check if the front sensor is further from the wall than the back sensor. (if its square then they should be roughly the same)
        // If the front sensor is further, do a slow left turn until they are equal.
        EncCountA = 0; 
        EncCountB = 0;
        MotorADir = 1;
        MotorBDir = 1;
        EncA.rise(&countPulseA);
        EncB.rise(&countPulseB);

        if (EncCountA < -EncCountB) { // See TurnLeft explaination.
          RampSpeedB(0.0f);
          RampSpeedA(SQUARE_SPEED);
        } else if (EncCountA > -EncCountB) {
          RampSpeedB(SQUARE_SPEED);
          RampSpeedA(0.0f);
        } else if (EncCountA == -EncCountB) {
          RampSpeedB(SQUARE_SPEED);
          RampSpeedA(SQUARE_SPEED);
          BothEncCount = EncCountA;
        }
      }
      else if (BL > FL + OFFSET) { // Check if the back sensor is further from the wall than the front sensor. (if its square then they should be roughly the same)
        // If the back sensor is further, do a slow right turn until they are equal.
        EncCountA = 0;
        EncCountB = 0;
        MotorADir = 0;
        MotorBDir = 0;
        EncA.rise(&countPulseA);
        EncB.rise(&countPulseB);
        if (EncCountB < -EncCountA) { // See RightTurn explaination.
          RampSpeedA(0.0f);
          RampSpeedB(SQUARE_SPEED);
        } else if (EncCountB > -EncCountA) {
          RampSpeedA(SQUARE_SPEED);
          RampSpeedB(0.0f);
        } else if (EncCountB == -EncCountA) {
          RampSpeedA(SQUARE_SPEED);
          RampSpeedB(SQUARE_SPEED);
          BothEncCount = EncCountA;
        }
      } 
      else{ // It is within the acceptable range.
        stop();
        EncCountA = BothEncCount; //Sets both encoder values to be the same just to ensure they both stop.
        EncCountB = BothEncCount;
        Square = true; // Confirm that it is square to exit the while loop
      }
      TimeOut = TimeOut + 1; // For every cycle of the previous code where the while loop isn't resolved, increment timout counter by one.
      if(TimeOut > 500){ // When while loop hasn't resolved fast enough, exit the squareup and attempt to go forwards a little bit to get out of the deadzone before trying again
        GoThisFar(10.0);
        SquareUpLeft();
      }
    }
  }
  AddToMap(SQUARE_LEFT);
}

void SquareUpRight() {  // Same as SquareUpLeft but on the right. Directions and comparisons are reversed.
  float FR = IRSensor(FRONT_RIGHT);
  float BR = IRSensor(BACK_RIGHT);
  Square = false;
  TimeOut = 0; // Timeout is used to limit the time the robot can spend trying to square up so that it doesn't get caught in a loop when squaring isn't possible.

  while (Square == false) {
    float FR = IRSensor(FRONT_RIGHT);
    float BR = IRSensor(BACK_RIGHT);
    if(FR > 20 && BR > 20){
      Square = true;
    }
    else{
      if (FR > BR + OFFSET) {
        EncCountA = 0;
        EncCountB = 0;
        MotorADir = 0;
        MotorBDir = 0;

        EncA.rise(&countPulseA);
        EncB.rise(&countPulseB);

        if (EncCountB < -EncCountA) {
          RampSpeedB(0.0f);
          RampSpeedA(SQUARE_SPEED);
        } else if (EncCountB > -EncCountA) {
          RampSpeedB(SQUARE_SPEED);
          RampSpeedA(0.0f);
        } else if (EncCountB == -EncCountA) {
          RampSpeedB(SQUARE_SPEED);
          RampSpeedA(SQUARE_SPEED);
          BothEncCount = EncCountA;
        }
      }


      else if (BR > FR + OFFSET) {
        EncCountA = 0;
        EncCountB = 0;
        MotorADir = 1;
        MotorBDir = 1;

        EncA.rise(&countPulseA);
        EncB.rise(&countPulseB);
        if (EncCountA < -EncCountB) {
          RampSpeedA(0.0f);
          RampSpeedB(SQUARE_SPEED);
        } else if (EncCountA > -EncCountB) {
          RampSpeedA(SQUARE_SPEED);
          RampSpeedB(0.0f);
        } else if (EncCountA == -EncCountB) {
          RampSpeedA(SQUARE_SPEED);
          RampSpeedB(SQUARE_SPEED);
          BothEncCount = EncCountA;
        }
      } 
      else {
        stop();
        EncCountA = BothEncCount;
        EncCountB = BothEncCount;
        Square = true;
      }
      TimeOut = TimeOut + 1; // For every cycle of the previous code where the while loop isn't resolved, increment timout counter by one.
      if(TimeOut > 500){ // When while loop hasn't resolved fast enough, exit the squareup and attempt to go forwards a little bit to get out of the deadzone before trying again
        GoThisFar(10.0);
        SquareUpRight();
      }
    }
  }
  AddToMap(SQUARE_RIGHT);
}


void SquareUp(){ // Select whether to square up on the left wall or the right wall based on which one is closer.
  int LeftWall = (AverageIRSensor(FRONT_LEFT) + AverageIRSensor(BACK_LEFT))/2;
  int RightWall = (AverageIRSensor(FRONT_RIGHT) + AverageIRSensor(BACK_RIGHT))/2;
  if(LeftWall > RightWall){ // If right wall is closer
    SquareUpRight();
  }
  else if(RightWall > LeftWall){ // If left is closer
    SquareUpLeft();
  }
  else{ // If they're equal distance
    SquareUpLeft(); // Favour the left hand wall
  }
}

void RampSpeedA(float SetSpeed){ // Function to slowly increase the speed of motor A to a desired final speed.
  float CurrentSpeed = MotorAPwm.read(); // Find the current PWM
  if(CurrentSpeed < SetSpeed){ // Compare whether current speed is above or below the desired value 
    CurrentSpeed = CurrentSpeed + RAMP; // If below desired value, increase PWM by the ramp value.
    MotorAPwm.write(CurrentSpeed); // Assign new speed.
  }
  else if(CurrentSpeed > SetSpeed){
    CurrentSpeed = CurrentSpeed - RAMP; // If above desired value, decrease PWM by the ramp value
    MotorAPwm.write(CurrentSpeed); // Assign new speed
  }
  else{
    MotorAPwm.write(CurrentSpeed); // If current speed is already the desired speed, just maintain that PWM.
  }
}

void RampSpeedB(float SetSpeed){ // Same as previous function but for wheel B.
  float CurrentSpeed = MotorBPwm.read();
  if(CurrentSpeed < SetSpeed){
    CurrentSpeed = CurrentSpeed + RAMP;
    MotorBPwm.write(CurrentSpeed);
  }
  else if(CurrentSpeed > SetSpeed){
    CurrentSpeed = CurrentSpeed - RAMP;
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
-Flashing the built-in RGB lights on the board. It isn't very visible because of how the robot is made but it adds style.

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

void Celebrate(){ // Calls when reaching the end of the maze to show that the robot knows that it has finished. Does 2 full spins to the right and then 2 to the left. Then calls LightShow again.
  for (int i = 0; i <= 8; i++){
    TurnRight();
  }
  delay(500);
  for (int i = 0; i <= 8; i++){
  TurnLeft();
  } // Ends facing same direction it started in.
  for (int i = 0; i <= 8; i++){
    LightShow();
  }
}

void LightShow(){
  for (int i = 0; i <= 29; i++){
    digitalWrite(LEDR, LOW); // Turn on Red light
    delay(100);
    digitalWrite(LEDR, HIGH); // Turn off Red light
    digitalWrite(LEDG, LOW); // Turn on Green light
    delay(100);
    digitalWrite(LEDG, HIGH);// Turn off Green light
    digitalWrite(LEDB, LOW); // Turn on Blue light
    delay(100);
    digitalWrite(LEDB, HIGH);// Turn off Blue light
  }
}

void FlashOnce(){
  digitalWrite(LEDR, LOW); // Turn on Red light
  digitalWrite(LEDB, LOW); // Turn on Blue light
  delay(100);
  digitalWrite(LEDR, HIGH); // Turn off Red light
  digitalWrite(LEDR, HIGH); // Turn off Blue. light
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
  Serial.println((String) "X:" + TestX + "   Y:" + TestY); // Prints NEWT's current coordinates. 


  while(true){
    if(TestY > 1150){
      break; // Breaks out of the while loop if the robot has already reached the end.
    }
    stop();
    float TestX = Localisation.GetX();
    float TestY = Localisation.GetY();
    Serial.println((String) "X:" + TestX + "   Y:" + TestY); // Prints NEWT's current coordinates. 
    SquareUp();

    int LeftWall = UltraSensor(ULTRA_4_PIN); // Check if there is a wall on the left using ultrasonic.
    int FrontWall = CheckFront(); // Check if there is a wall in front using ultrasonic. (Seperate function for checking front because it uses 2 sensors to verify each other.)
    int RightWall = UltraSensor(ULTRA_2_PIN); // Check if there is a wall on the right using ultrasonic.
    if (FrontWall < 13 || FrontWall > 600){
      ReverseThisFar(10.0);
    }
    else{
      if(LeftWall > 35 && LeftWall < 900){ // Check if there is a wall on the left using ultrasonic. there isn't
        float FL = AverageIRSensor(FRONT_LEFT); // Check front left ir
        float BL = AverageIRSensor(BACK_LEFT); // Check back left ir
        if(FL > 30 && BL > 30){ // Check whether both sensors confirm there is no wall
          TurnLeft(); // If there is a wall close in front, do a small turn to avoid driving into it.
          GoThisFar(340.0); // Go forwards 10 cm and check again
        }
        else if((FL > 20 && BL < 20) || (FL <20 && BL > 20)){ // Is there is a wall in front of one sensor but not the other
          if(FrontWall > 10 && FrontWall < 900){ // Check if there is a wall close in front, in this case there isn't
            GoThisFar(50.0); // Go forwards a little bit
          }
          else if(FrontWall <= 13){ // In this case there is a wall close in front
            ReverseThisFar(10.0); // Reverse a little
          }
          else{
            TurnRight();
          }
        }
        else{ // There actually is a wall on the left, the ultrasonic just failed.
          SquareUp();
          if(FrontWall > 13 && FrontWall < 900){ // Check if there is a wall close in front. In this case there isn't
            GoThisFar(100.0f); // Go forwards a bit and check again
          }
          else if(FrontWall <= 14 || FrontWall > 900){
            TurnRight();
          }
        }
      }
      else if(LeftWall <= 25 || LeftWall >900){ // Check if there is a wall on the left using ultrasonic. there is
        if(FrontWall >= 16 && FrontWall < 900){ // Check if there is a wall close in front. there isn't
          GoThisFar(100.0f); // Go forwards and check again
        }
        else if(FrontWall < 16){ // Check if there is a wall close in front. there is
          if(RightWall > 20 && RightWall <900){ // Check if there is a wall on the Right using ultrasonic. there isn't
            float FR= AverageIRSensor(FRONT_RIGHT); // Check front Right ir
            float BR= AverageIRSensor(BACK_RIGHT); // Check back Right ir
            if(FR> 20 && BR> 20){ // Check whether both sensors confirm there is no wall
              TurnRight();
            }
            else if(FR> 20 && BR< 20){ // If there is a wall in front of back sensor but not front. I don't think this can even happen but just in case.
              if(FrontWall > 10 && FrontWall < 900){ // Check if there is a wall close in front
                GoThisFar(50.0); // Go forwards a little bit
              }
            }
            else if (FR<20 && BR> 20){ // If there is a wall in front of front sensor but not back
              ReverseThisFar(50.0); 
            }
            else{
              if(FrontWall >= 16 && FrontWall < 900){ // Check if there is a wall close in front
                GoThisFar(100.0f); // Go forwards and check again
                SquareUp(); // Square up with the closest wall
              }
            }
          }
          else{ // There is a wall on the left, the front and the right
            TurnRight(); // Turn around 180 degrees
            TurnRight();
          }
        }
      }
    }
    if(TestY > 1150){
      break; // Breaks out of the while loop if the robot has already reached the end.
    }
  }
  LightShow();
  Exploring = false;
  //ReadMap(); // Uncomment if you want to print the array of moves from the Start point to the End point.
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

// Start Facing West

void UpdatePosition(){
  float DistanceTravelled = FindDistanceTravelled();
  float XCurrent = Localisation.GetX(); // Get the current x position on cartesian.
  float YCurrent = Localisation.GetY(); // Get the current Y position on cartesian.
  // Decide whether x or y is being changed depending on what direction the robot is facing. Only moves in straight lines and turn at 90 degrees so only one should change at a time.
  if(Localisation.GetDirection() == "West"){ // If the robot is facing "West" (West is defined as being facing towards the left hand of the maze, not based earth's poles)
    float NewX = XCurrent - DistanceTravelled; // Work out the new value of x
    Localisation.UpdateX(NewX); // Update global variable tracking x coordinate
  }
  else if(Localisation.GetDirection() == "North"){ // See above
    float NewY = YCurrent + DistanceTravelled;
    Localisation.UpdateY(NewY);
  }
  else if(Localisation.GetDirection() == "East"){ // See above
    float NewX = XCurrent + DistanceTravelled;
    Localisation.UpdateX(NewX);
  }
  if(Localisation.GetDirection() == "South"){ // See above
    float NewY = YCurrent - DistanceTravelled;
    Localisation.UpdateY(NewY);
  }

}

void AddToMap(float NewMove){ // Adds a new move to the map on the way to the end.
  MovesToEnd[CurrentMove] = NewMove;
  CurrentMove = CurrentMove + 1;
}
void AddToReverseMap(float NewMove){ // Adds a new move to the the map on the way back to the start.
  MovesBackToStart[MapReversalCounter] = NewMove;
  MapReversalCounter = MapReversalCounter + 1;
}

void ReadMap(){
  for (byte i = 0; i < 199; i = i + 1) { // Goes through each move in the array and prints them out on the Serial monitor.
    if(MovesToEnd[i] == LEFT){
      Serial.println("Left");
    }
    else if(MovesToEnd[i] == RIGHT){
      Serial.println("Right");
    }
    else if(MovesToEnd[i] == SQUARE_LEFT){
      Serial.println("Squareup Left");
    }
    else if(MovesToEnd[i] == SQUARE_RIGHT){
      Serial.println("Squareup Right");
    }
    else if(MovesToEnd[i] < 0){ // Reversing is signified by a negative value.
      Serial.println("Reverse");
      Serial.println(MovesToEnd[i]);
    }
    else{
      Serial.println("Forward"); // A positive value that is not equal to LEFT, RIGHT, SQUARE_LEFT or SQUARE_RIGHT is a forwards command.
      Serial.println(MovesToEnd[i]);
    }
  }
}

void CreateMapBack(){ // Function that reverses the map from start to end to create a map from end to start.
  for (byte i = 199; i > 0; i = i - 1){
    if(MovesToEnd[i] != 0){ // If there is a 0 value
      if(MovesToEnd[i] == LEFT){ // If it made a left on the way there
        AddToReverseMap(RIGHT); // Make a right on the way back
      }
      else if(MovesToEnd[i] == RIGHT){ // If it made a right on the way there;
        AddToReverseMap(LEFT); // Make a left on the way back
      }
      else if(MovesToEnd[i] == SQUARE_LEFT){ // When it squares up on the left
        AddToReverseMap(SQUARE_RIGHT); // Square up on the right on the way back
      }
      else if(MovesToEnd[i] == SQUARE_RIGHT){ // When it squares up right
        AddToReverseMap(SQUARE_LEFT); // Square up on the left on the way back
      }


      else if(MovesToEnd[i] < 0){ // When it is reversing
        for (byte j = 1; j < 99; j = j + 1){ // This loop finds the next forwards and then subtracts the reverse from it.
          if(MovesToEnd[i-j] != LEFT && MovesToEnd[i-j] != RIGHT && MovesToEnd[i-j] != SQUARE_LEFT && MovesToEnd[i-j] != SQUARE_RIGHT){ 
            MovesToEnd[i-j] = MovesToEnd[i-j] + MovesToEnd[i];
            break; // Exits the for loop
          }
        }
      }
      else{ // It's going fowards
        AddToReverseMap(MovesToEnd[i]); // Go forwards that far
      }
    }
    
  }
  //ReadMapBack(); // Uncomment to print the map from the Finish point to the Start point.
  FollowMapBack(); // Call the function to follow the map once the map is complete.
}

void ReadMapBack(){
  for (byte i = 0; i < 199; i = i + 1) { // Goes through each move in the array and prints them out on the Serial monitor.
    if(MovesBackToStart[i] == LEFT){
      Serial.println("Left");
    }
    else if(MovesBackToStart[i] == RIGHT){
      Serial.println("Right");
    }
    else if(MovesBackToStart[i] == SQUARE_LEFT){
      Serial.println("Squareup Left");
    }
    else if(MovesBackToStart[i] == SQUARE_RIGHT){
      Serial.println("Squareup Right");
    }

    else if(MovesBackToStart[i] < 0){ // Reversing is signified by a negative value.
      Serial.println("Reverse");
      Serial.println(MovesBackToStart[i]);
    }
    else{
      Serial.println("Forward"); // A positive value that is not equal to Left, Right, SQUARE_LEFT or SQUARE_RIGHT is a forwards command.
      Serial.println(MovesBackToStart[i]);
    }
  }
}

void FollowMapBack(){ // Checks each term in the map back to the start and executes relevant command.
  TurnRight(); //Turn around 180 degrees
  TurnRight();
  for (byte i = 0; i < 199; i = i + 1){
    if(MovesBackToStart[i] == 0){
      FlashOnce();
    }
    else if(MovesBackToStart[i] == LEFT){
      TurnLeft();
    }
    else if(MovesBackToStart[i] == RIGHT){
      TurnRight();
    }
    else if(MovesBackToStart[i] == SQUARE_LEFT){
      SquareUpLeft();
    }
    else if(MovesBackToStart[i] == SQUARE_RIGHT){
      SquareUpRight();
    }
    else{
      GoThisFar(MovesBackToStart[i]);
    }
  }

  Celebrate();
  while(true){
    sleep();
  }
}

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*Segment 7*/
/*Setup and loop*/

/*----------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup() {
  // Serial.begin(9600); // Start Serial connection, used for debugging.

  MotorAPwm.period_ms(1); // Set the motor period for motors A and B.
  MotorBPwm.period_ms(1);  
  MotorAPwm.write(0.0f); // Ensuring the motors start out off.
  MotorBPwm.write(0.0f);

  EncCountA = 0; // Ensuring the encoder counts start at 0.
  EncCountB = 0;
  BothEncCount = 0;


  Localisation.SetDirection("West"); // Assign the starting direction (starts facing in the negative x direction) as West.

  pinMode(LEDR, OUTPUT);  // Pins for the RGB lights.
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH); // Turn off Red light
  digitalWrite(LEDG, HIGH); // Turn off Green light
  digitalWrite(LEDB, HIGH); // Turn off Blue light

  Exploring = true;

  delay(1000);  

}



void loop() {
  Explore();
}
