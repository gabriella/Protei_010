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

  Channel2Value = pulseIn (PD6, HIGH, 20000); //read RC channel 2

//  Serial.print("Initial Throttle: ");
//  Serial.print (InitialThrottle);
//  Serial.println("");
//  Serial.print("Channel 2: ");
//  Serial.print (Channel2Value);
//  Serial.println("");

 if(Channel2Value==0){
    servo1.write(90);
    Serial.println(Channel2Value);

  }
  else{
  Channel2Value = map(Channel2Value,1020,1867, 0,180);
    servo1.write(Channel2Value); 
    Serial.println(Channel2Value);

  }

}



