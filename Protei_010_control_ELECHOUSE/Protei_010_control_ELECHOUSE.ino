      /*Protei_010_control.ino, 
      
            /*Protei_010_control.ino, 
      
      Protei — Remote Control and Motor Control
 Copyright (C) 2013  Gabriella Levine,
 
 	This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program (see COPYING).  If not, see <http://www.gnu.org/licenses/>.
 
      
 https://github.com/gabriella/Protei_010.git
 code for Protei_010 arduino control through TGY 6ch tx/rx
 through motor controller : http://www.elechouse.com/elechouse/index.php?main_page=product_info&cPath=100_146&products_id=2179
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

const int RX3= 17;
const int RX1=19;

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
    int motor_body = map(Channel2Value,1100,1800, 0,255);
    Serial.print(constrain(motor_body,0,255));
    if(motor_body<=100){
      Serial.print("     backwards     ");
      motor_body= map(motor_body, 110,0,0,255);
      motor_body = constrain(motor_body, 0,255);
      motor.set(B, motor_body, REVERSE);

    }
    else if(motor_body>=150){

      Serial.print("     forwards    ");
      motor_body= map(motor_body, 150,255,0,255);
      motor_body = constrain(motor_body, 0,255);

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
    int motor_sail = map(Channel1Value,1100,1800, 0,255);
    Serial.print(constrain(motor_sail,0,255));
    if(motor_sail<=100){
      Serial.print("     backwards     ");
      motor_sail = map(motor_sail, 110,0,0,255);
      motor_sail = constrain(motor_sail, 0,255);

      motor.set(A, motor_sail, REVERSE);

    }
    else if(motor_sail>=150){
      Serial.print("     forwards    ");
      motor_sail = map(motor_sail, 150,255,0,255);
      motor_sail = constrain(motor_sail, 0,255);
      motor.set(A, motor_sail, FORWARD);
    }
    else{
      Serial.print("     STOP    ");
      motor.close(A);
    } 
  }
}







