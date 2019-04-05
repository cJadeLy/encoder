// comments are not complete for subsystem funciton calls. They will be when I have extensively tested them when I am sure that I am satisfied with encoder behavior
// encoder could stand to be optimized but given our previous complete lack of success we have given it a forgiving threshold.
// results are as expected

// Revisions to come


#include <PID_v1.h>
#include "CytronMotorDriver.h"
#include <Stepper.h>


#define MotEnable 9 //Motor Enamble pin Runs on PWM signal
#define MotDir  10  // Motor Direction pin

#define ENCODER0PINA         20      // this pin needs to support interrupts
#define ENCODER0PINB         17      
#define CLOCKWISE            1       // direction constant
#define COUNTER_CLOCKWISE    2       // direction constant
#define REVOLUTIONS          10      // 30 = a distance of 2 inches
#define RPM                  30      // for carriage motor speed
#define MAPval               45      // forgiving precision for encoder that maps 0 - 2000 to 0 - MAPval
#define CPR                  2000  // Encoder Cycles per revolution.

// next is a control variable that is used as a bounds check for the position array
short next = 0; 
// scale each bolt location to allow a threshold greater than one degree precision 
int bolt1 = map(180,0, 360, 0, MAPval);
int bolt2 = map(270,0, 360, 0, MAPval);
int bolt3 = map(90,0, 360, 0, MAPval);
int bolt4 = map(135,0, 360, 0, MAPval);
int bolt5 = map(315,0, 360, 0, MAPval);
int bolt6 = map(45,0, 360, 0, MAPval);
int bolt7 = map(225,0, 360, 0, MAPval);

// stuff those values into this array to iterate over
int positions[8] = {bolt1,bolt2, bolt3, bolt4, bolt5, bolt6, bolt7, 0};

// when all indices == 2 the COBOT has completed 3 passes. As of 4/04/19 this is unused but it shall be implemented in the next revision
int pass[8] = {0, 0, 0, 0, 0, 0, 0, 1};

// variables modified by interrupt handler must be declared as volatile
volatile long encoder0Position = 0;
volatile long interruptsReceived = 0;
 
// track last position so we know whether it's worth printing new output, used when debugging
int previousPosition = 0;

// logic control
bool locationFound = false;
bool keepLooking = true;

// track direction: 0 = counter-clockwise; 1 = clockwise
short currentDirection = CLOCKWISE;

// Number of steps per output rotation // TODO: should change to a #define and all caps. 
const int stepsPerRevolution = 200;

// PID //TODO: explain constants
double kp = .5 , ki = 1 , kd = .5; //kd prev vals: 1; 0.01;  kp prev vals: = 5, 1 KP = 2 BADD (WHEN KI = 1 AND KD = .5)
// BEST SO FAR might be kp = 1, ki = 1, kd = .5
// see details in ardiono PID library files 
double input = 0, output = 0, setpoint = 0;
// Create Instance of PID library
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT); 

// Configure the motor driver.
// Create Instance of CytronMD library for torque motor
CytronMD torqueMotor(PWM_DIR, 6, 7);  // PWM = Pin 6, DIR = Pin 7.
// Create Instance of CytronMD library for carriage motor
CytronMD carriageMotor(PWM_DIR, 9, 10);  // PWM = Pin 9, DIR = Pin 10.

// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 35, 37, 39, 41);

//void articulationSystemForward()
//{
//  
//  myStepper.setSpeed(200);
//  Serial.println("Articulation System Activated yeeaaaaaa");
//  for(int i = 0; i < REVOLUTIONS; i++)
//  {
//  
//    myStepper.step(stepsPerRevolution);
//    delayMicroseconds(10);
//  }

//activateTorqueMotor();
//}
void articulationSystemBackward()
{
  Serial.println("/n Articulation System Activated /n");
  // set the speed at 60 rpm:
  myStepper.setSpeed(200);
  for(int i = 0; i < REVOLUTIONS; i++)
  {
          myStepper.step(-stepsPerRevolution);
          delayMicroseconds(10);

  }
//startCarriageMotor();

}
void activateTorqueMotor()
{
    Serial.println("\n\n\n");
  Serial.println("Torque...");
  //delay(1000);
  for(int go = 0; go < 5; go++)
  {
//      torqueMotor.setSpeed(50);
Serial.println("Torque...");
    delay(250);
  }

//
  Serial.println("Leaving. /n");
//   torqueMotor.setSpeed(0);
//    delay(250);
    keepLooking = true;
//articulationSystemBackward();
}
void startCarriageMotor()
 {
  Serial.println("\n\n\n");
  Serial.println("About to go look for location of next bolt...");
  //delay(1000);
  carriageMotor.setSpeed(70);
    delay(250);
  Serial.println("Leaving.");
  keepLooking = true;
 }
void setup() {
  TCCR2B = TCCR2B & 0b11111000 | 0x01;  // set 31KHz PWM to prevent motor noise
  // dont remember why I commented this out. will look into later
//  pinMode(ENCODER0PINA, INPUT);
//  pinMode(ENCODER0PINB, INPUT);

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(41, OUTPUT);
  pinMode(MotEnable, OUTPUT);
  pinMode(MotDir, OUTPUT); 

  Serial.begin(9600); //initialize serial communication

  pinMode(ENCODER0PINA, INPUT_PULLUP); 
  pinMode(ENCODER0PINB, INPUT_PULLUP);

  digitalWrite(ENCODER0PINA, HIGH); 
  digitalWrite(ENCODER0PINB, HIGH); 

  attachInterrupt(3, updateEncoder, RISING); 
    
  myPID.SetMode(AUTOMATIC);   //set PID in Auto mode
  myPID.SetSampleTime(1);  // refresh rate of PID controller
  myPID.SetOutputLimits(-RPM, RPM); //here change in value reflect change in speed limits of motor 
 // enable diagnostic output
  Serial.begin (9600);
  Serial.println("\n\n\n");
  Serial.println("Ready.");

   // final code will call articulation system. testing sometimes requires other behavior
 //startCarriageMotor();
// articulationSystemBackward();
 activateTorqueMotor();
}

void loop()
{

if (locationFound)
  {

    Serial.println("FOUND LOCATION?");
//    Serial.print(encoder0Position, DEC);
//    Serial.print("\t");
//    Serial.print(currentDirection == CLOCKWISE ? "clockwise" : "counter-clockwise");

    // did we really find the location? If not let PID handle it, otherwise we got lucky so here we are 
    // Dont' worry Reed there is no offset this is actual location compared to a location PID would find acceptable
    if(abs(map(encoder0Position, 0, 2000, 0, MAPval)) == positions[next])
    {
       // reset location found  and disable other functions that are not desired when we are at a bolt location
       locationFound = false;
       keepLooking = false;
       // debugging and data gathering 
       Serial.println("YES! position that mapped was:  ");
       Serial.println(encoder0Position);
       Serial.println("cycles completed: ");
       Serial.println(interruptsReceived,DEC);

       // dont go out of bounds in our positions array! C be dangerous
       if(next == 7)
       {
        next = 0;
       }
       else
       {
        next++;
       }
       
       // debugging and data gathering 
       Serial.println("bolt location that was found:  ");
       Serial.println(next);
       // final code will call articulation system, but for now we just need to do something other than sit here 
       activateTorqueMotor();
    }
    else
    {
      Serial.print("Nope.. not good enough value: \t");
      Serial.println((map(encoder0Position, 0, 2000, 0, MAPval)));
      Serial.print("actual value (that was mapped): \t");
      Serial.println(encoder0Position);
    }
  }
   // debug stuff
  if (encoder0Position !=  previousPosition && keepLooking == true)
  {

//    Serial.println("Current Position: ");
//    Serial.print(encoder0Position, DEC);
//    Serial.print("\t");
//    Serial.print(currentDirection == CLOCKWISE ? "clockwise" : "counter-clockwise");
//    Serial.print("\t");
//    Serial.println(interruptsReceived, DEC);

//      previousPosition = encoder0Position;

  }

   

  // man I dont remember why this is here so I am leaving it! Ill figure it out later
  input = encoder0Position;            

  myPID.Compute();  // calculate new output
  
// only do this stuff if COBOT is not busy with other stuff
if(keepLooking)
{
  setpoint = positions[next]; 
    pwmOut(output);  
}
  
  
}
void pwmOut(int out)
{                               
  if (out > 0)
  { 
    Serial.println("setting motor speed to: ");
    Serial.println(out);   
    analogWrite(MotEnable, out);         // Enabling motor enable pin to reach the desire angle
    forward();                           // calling motor to move forward
  }
  else
  {
    Serial.println("setting motor speed to: ");
    Serial.println(out); 
    analogWrite(MotEnable, abs(out));                        
    reverse();                            // calling motor to move reverse
  }
  
  

}


 
 



void updateEncoder()
{
// read both inputs
  int a = digitalRead(ENCODER0PINA);
  int b = digitalRead(ENCODER0PINB);
 
  if (a == b )
  {
    // b is leading a (counter-clockwise)
    encoder0Position--;
    currentDirection = COUNTER_CLOCKWISE;
  }
  else
  {
    // a is leading b (clockwise)
    encoder0Position++;
    currentDirection = CLOCKWISE;
  }
 
  // track 0 to 1999
  encoder0Position = encoder0Position % CPR;

// changing positions[next] to setspeed MAY improve accuracy. but I gotta think more. 
if(abs((map(encoder0Position, 0, 2000, 0, MAPval))) == positions[next])
 {
  locationFound = true;
  // This stays
  carriageMotor.setSpeed(0);
  delay(250);
 }

  interruptsReceived++;

}

void forward ()
{
  digitalWrite(MotDir, HIGH); 
  
}

void reverse ()
{
  digitalWrite(MotDir, LOW); 
 
  
}
// I dont even use this plus it doesn't do what I want it to. It stays for now
void finish ()
{
  Serial.print("Stopping");
  digitalWrite(MotDir, LOW); 
 digitalWrite(MotEnable, LOW); 
  
}
