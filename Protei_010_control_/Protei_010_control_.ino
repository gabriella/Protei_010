/*Protei_010_control.ino, 
 https://github.com/gabriella/Protei_010.git
 code for Protei_010 arduino control through TGY tx
 through 2 motor controllers: http://www.pololu.com/catalog/product/1376
 and
 two Brush DC geared motors:http://www.pololu.com/catalog/product/1577
 */


//#include <Servo.h>
//#include <Metro.h> //Include Metro library
//Servo servo1;
//Servo servo2;
//Metro servo = Metro(250); 

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

const int RX3= 7;
const int RX1=6;


const int OUTPUT_1 = 11; // Analog output pin that the Motor1 pwm pin is attached to
const int DIR_PIN_1=13; //digital output for logic direction pin motor 1
const int OUTPUT_2= 10; // Analog output pin that the MOTOR2 pwm pin is attached to
const int DIR_PIN_2 = 12;//digitaloutput for logic direction pin motor2
const int ledPower = 7;


void setup()
{

  Serial.begin(19200);
  Serial.println("Ready");
  pinMode(ledPower, OUTPUT);
  digitalWrite(ledPower, HIGH);

  pinMode(OUTPUT_1,OUTPUT);
  pinMode(OUTPUT_2,OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);

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
  Serial.print("Initial Throttle: ");
  Serial.print (InitialThrottle);
  Serial.print("    ");
  Serial.print("Channel 2: ");

  if(Channel2Value==0){
    digitalWrite(OUTPUT_1, LOW);
    Serial.println(Channel2Value);
  }
  else{
    Channel2Value = map(Channel2Value,1020,1875, 0,255);
    Serial.println(Channel2Value);
    if(Channel2Value<=100){
      digitalWrite(DIR_PIN_1, LOW);
      Serial.print("     backwards     ");
      analogWrite(OUTPUT_1, map(RX1, 110,0,0,255));
    }
    else if(Channel2Value>=150){
      digitalWrite(DIR_PIN_1, HIGH);
      Serial.print("     forwards    ");
      analogWrite(OUTPUT_1, map(RX1, 150,255,0,255));
    }
    else{
      Serial.print("     STOP    ");
      digitalWrite(OUTPUT_1, LOW);
    } 
  }   
}






