/*Protei_010_control.ino, 
 https://github.com/gabriella/Protei_010.git
 code for Protei_010 arduino control through TGY tx
 through 2 motor controllers: http://www.pololu.com/catalog/product/1376
 and
 two Brush DC geared motors:http://www.pololu.com/catalog/product/1577
 */


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

const int RX3= 16;
const int RX1=14;

//import motor library
#include <MOTOR.h>

const int ledPower = 13;

void setup()
{
  // motor driver initialize 
  motor.begin();

  Serial.begin(19200);
  Serial.println("Ready");
  pinMode(ledPower, OUTPUT);
  digitalWrite(ledPower, HIGH);

  pinMode (RX3, INPUT); // connect Rx channel 1 to PD7, which is labled "D7" on the Arduino board
  pinMode (RX1, INPUT); // connect Rx channel 2 to PD6, which is labled "D6" on the Arduino board

  InitialSteer = pulseIn (RX3, HIGH); //read RC channel 1
  SteerScaling = .15;
  lastgood1 = InitialSteer;
  InitialThrottle = pulseIn (RX1, HIGH); //read RC channel 2
  ThrottleScaling = .15;//scaling factor for the "speed"
  lastgood2 = InitialThrottle; 
}

void loop()
{
  Channel2Value = pulseIn (RX1, HIGH, 20000); //read RC channel 2

  Channel1Value = pulseIn (RX3, HIGH, 20000); //read RC channel 2
  Serial.print("   motor_body :    ");

 motorBody();
  Serial.print("    motor_sail :    ");

  motorSail();
  Serial.println("");

}

void motorBody(){
  if(Channel2Value==0){
 motor.close(B);
    Serial.print(Channel2Value);
    Serial.print("     no signal   ");
  }
  else{
    int motor_body = map(Channel2Value,1020,1875, 0,255);
    Serial.print(motor_body);
    if(motor_body<=100){
      Serial.print("     backwards     ");
      motor_body= map(motor_body, 110,0,0,255);
           motor.set(B, motor_body, REVERSE);

    }
    else if(motor_body>=150){
    
      Serial.print("     forwards    ");
      motor_body= map(motor_body, 150,255,0,255);
                 motor.set(B, motor_body, FORWARD);

    }
    else{
      Serial.print("     STOP    ");
    motor.close(B);
    } 
  }
}

void motorSail(){
  if(Channel1Value==0){
 motor.close(A);
    Serial.print(Channel1Value);
    Serial.print("   no signal   ");
  }
  else{
    int motor_sail = map(Channel1Value,1020,1875, 0,255);
    Serial.print(motor_sail);
    if(motor_sail<=100){
      Serial.print("     backwards     ");
      motor_sail = map(motor_sail, 110,0,0,255);
           motor.set(A, motor_sail, REVERSE);

    }
    else if(motor_sail>=150){
      Serial.print("     forwards    ");
     motor_sail = map(motor_sail, 150,255,0,255);
     motor.set(A, motor_sail, FORWARD);
    }
    else{
      Serial.print("     STOP    ");
     motor.close(A);
    } 
  }
}

//set the limit




