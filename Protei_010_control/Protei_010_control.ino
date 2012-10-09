#include <Servo.h>
#include <Metro.h> //Include Metro library
Servo servo1;
//Servo servo2;
Metro servo = Metro(250); 

unsigned long counter = 0;
unsigned long Channel1Value;
unsigned long Channel2Value;
unsigned long lastgood1;
unsigned long lastgood2;
unsigned long InitialSteer;
unsigned long InitialThrottle;
float ThrottleScaling;
float SteerScaling;
boolean Right;
//motor stuff
int Steer;
int Thrust;
int RightMotor;
int LeftMotor;


const int PD7= 7;
const int PD6=6;
const int PD5=5;
const int PD3=3;

/**************************************************************
 * Subroutine to control right motor= motorRight(speed, direction);
 ***************************************************************/
void motorRight(byte PWM)
{
  if(PWM==0) //If Speed = 0, shut down the motors
  {
//    digitalWrite(6, LOW); 
    digitalWrite(PD3, LOW); 
 
  }  
  else{
//    digitalWrite(7, LOW);
    analogWrite(PD3, PWM); 
    }
}  

/**************************************************************
 * Subroutine to control left motor
 ***************************************************************/
void motorLeft(byte PWM)
{
  if(PWM==0) // If Speed = 0, shut down the motor
  {
//    digitalWrite(4, LOW); These were for the old form, where I could turn the motors both ways
   // digitalWrite(PD5, LOW); 
    servo1.write(90);
  }  
  else{
//    digitalWrite(4, LOW);
servo1.write(PWM);
   // analogWrite(PD5, PWM); 
    }
}  


void setup()
{
//  servo1.attach(14); // connect servo1 to analog pin 0
  //servo1.setMaximumPulse(2000);
  //servo1.setMinimumPulse(700);

//  servo2.attach(15); // connect servo2 to analog pin 1
  Serial.begin(19200);
  Serial.println("Ready");
  pinMode (PD7, INPUT); // connect Rx channel 1 to PD7, which is labled "D7" on the Arduino board
  pinMode (PD6, INPUT); // connect Rx channel 2 to PD6, which is labled "D6" on the Arduino board
  pinMode(PD3, OUTPUT); // Set the Right motor control PWM pin to OUTPUT
  servo1.attach(5);
 // pinMode(PD5, OUTPUT); // Set the Left motor control PWM pin to OUTPUT
  InitialSteer = pulseIn (PD7, HIGH); //read RC channel 1
  SteerScaling = .15;
  lastgood1 = InitialSteer;
  InitialThrottle = pulseIn (PD6, HIGH); //read RC channel 2
  ThrottleScaling = .15;//scaling factor for the "speed"
  lastgood2 = InitialThrottle; 
}

void loop()
{
//  counter++;  // this just increments a counter for benchmarking the impact of the pulseIn's on CPU performance
  Channel1Value = pulseIn (PD7, HIGH, 20000); //read RC channel 1
  
   // if (ledMetro.check() == 1) { // check if the metro has passed its interval .

 // if (Channel1Value == 0) {Channel1Value = lastgood1;} else {lastgood1 = Channel1Value;}
  
  //if been 0 for x time, dont put to lastgood, put it to 90
  
  if (Channel1Value < InitialSteer) {Right = false;} else {Right = true;}
  Serial.print ("Initial Steer: ");
  Serial.print (InitialSteer);
  Serial.println("");
  Serial.print ("Channel 1: ");
  Serial.print (Channel1Value);
  Serial.println("");
//  if (Right) {Steer = Channel1Value - InitialSteer;} else {Steer = InitialSteer - Channel1Value;}
//  Steer = Steer * SteerScaling; // convert to 0-100 range
//  constrain (Steer, 0, 100); //just in case
  Channel2Value = pulseIn (PD6, HIGH, 20000); //read RC channel 2
  Serial.print("Initial Throttle: ");
  Serial.print (InitialThrottle);
  Serial.println("");
  Serial.print("Channel 2: ");
  Serial.print (Channel2Value);
  Serial.println("");
  
    Channel2Value = map(Channel2Value,1020,1867, 0,180);
    
    if(Channel2Value==0){
      servo1.write(200);}
      else{
   servo1.write(Channel2Value); 
      }
   //servo1.write(Channel1Value); 

}
//
//  Thrust = Channel2Value - InitialThrottle;
//  Thrust = Thrust * ThrottleScaling; // convert to 0-100 range
//  constrain (Thrust, 0, 100); //just in case
////  servo1.write(servo1value); // this is all RC servo stuff; ignore for now
////  servo2.write(servo2value);
////  Servo::refresh();
////  Serial.print (channel1value);  // if you turn on your serial monitor you can see the readings.
////  Serial.print ("  ");
////  Serial.print (channel2value);
////  Serial.println("");
////  Serial.println (counter); uncomment if you're benchmarking
//  Serial.print (Steer);
//  Serial.print ("  ");
//  Serial.print (Thrust);
//  Serial.println("");
//
//// Here we combine the direction and thrust into commands for the two motors. We never turn the motors backwards, 
////   because that would mess up the operation of the vertical vectoring thrusters.  
//  
//  if (Right == true) // turn right 
//  {
//    RightMotor = Thrust - Steer; // reduce power to the motor in the direction you want to go 
//    if (RightMotor < 0) {RightMotor = 0;}
//    LeftMotor = Thrust + Steer; // increase power to the motor opposite the direction you want to go
//  }
//  if (Right == false) //turn left
//  {
//    LeftMotor = Thrust - Steer; // reduce power to the motor in the direction you want to go 
//    if (LeftMotor < 0) {LeftMotor = 0;}
//    RightMotor = Thrust + Steer; // increase power to the motor opposite the direction you want to go
//  }
//  constrain (LeftMotor, 0,100);
//  constrain (RightMotor, 0, 100);
//  motorLeft(LeftMotor);
//  motorRight(RightMotor);
//
//}
//
//

