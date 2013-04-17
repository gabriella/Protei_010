/*Protei_010_control.ino, 
 https://github.com/gabriella/Protei_010.git
 code for Protei_010 arduino control through TGY 6ch tx/rx
 through motor controller : http://www.elechouse.com/elechouse/index.php?main_page=product_info&cPath=100_146&products_id=2179
 and
 two Brush DC geared motors:http://www.pololu.com/catalog/product/1577
 & two encoders
 */

#define ENC1_A 12
#define ENC1_B 13
#define ENC_PORT1 PINB

#define ENC2_A 18
#define ENC2_B 19
#define ENC_PORT2 PINC



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

const int ledPower = 11;




void setup()
{
  // Setup encoder pins as inputs
  pinMode(ENC1_A, INPUT);
  digitalWrite(ENC1_A, HIGH);
  pinMode(ENC1_B, INPUT);
  digitalWrite(ENC1_B, HIGH);
  //SET UP encoder B pins as inputs: 
  pinMode(ENC2_A, INPUT);
  digitalWrite(ENC2_A, HIGH);
  pinMode(ENC2_B, INPUT);
  digitalWrite(ENC2_B, HIGH);

  Serial.begin(115200);
  Serial.println("Ready");
 
  // motor driver initialize 
  motor.begin();

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
  encoders();

  Channel2Value = pulseIn (RX1, HIGH, 20000); //read RC channel 2

  Channel1Value = pulseIn (RX3, HIGH, 20000); //read RC channel 2
  // Serial.print("   motor_body :    ");

  motorBody();
  //  Serial.print("    motor_sail :    ");

  motorSail();
  // Serial.println("");
  
}

void motorBody(){
  if(Channel2Value==0){
    motor.close(B);
    //Serial.print(Channel2Value);
    // Serial.print("     no signal   ");
  }
  else{
    int motor_body = map(Channel2Value,1100,1800, 0,255);
    //Serial.print(constrain(motor_body,0,255));
    if(motor_body<=100){
      //    Serial.print("     backwards     ");
      motor_body= map(motor_body, 110,0,0,255);
      motor_body = constrain(motor_body, 0,255);
      motor.set(B, motor_body, REVERSE);

    }
    else if(motor_body>=150){

      // Serial.print("     forwards    ");
      motor_body= map(motor_body, 150,255,0,255);
      motor_body = constrain(motor_body, 0,255);

      motor.set(B, motor_body, FORWARD);

    }
    else{
      //  Serial.print("     STOP    ");
      motor.close(B);
    } 
  }
}

void motorSail(){
  if(Channel1Value==0){
    motor.close(A);
    //  Serial.print(Channel1Value);
    //  Serial.print("   no signal   ");
  }
  else{
    int motor_sail = map(Channel1Value,1100,1800, 0,255);
    //  Serial.print(constrain(motor_sail,0,255));
    if(motor_sail<=100){
      //  Serial.print("     backwards     ");
      motor_sail = map(motor_sail, 110,0,0,255);
      motor_sail = constrain(motor_sail, 0,255);

      motor.set(A, motor_sail, REVERSE);

    }
    else if(motor_sail>=150){
      // Serial.print("     forwards    ");
      motor_sail = map(motor_sail, 150,255,0,255);
      motor_sail = constrain(motor_sail, 0,255);
      motor.set(A, motor_sail, FORWARD);
    }
    else{
      //   Serial.print("     STOP    ");
      motor.close(A);
    } 
  }
}


void encoders(){
  static uint8_t counter1 = 0;      //this variable will be changed by encoder input
  int8_t tmpdata1;

  tmpdata1 = read_encoder1();
  if( tmpdata1 ) {
    Serial.print("Counter value1: ");
    Serial.println(counter1, DEC);
    counter1 += tmpdata1;
  }

  static uint8_t counter2 = 0;      //this variable will be changed by encoder input
  int8_t tmpdata2;
  /**/
  tmpdata2 = read_encoder2();
  if( tmpdata2 ) {
  Serial.print("Counter value2: ");
  Serial.println(counter2, DEC);
    counter2 += tmpdata2;
  }  

}

// returns change in encoder state (-1,0,1) */
int8_t read_encoder1()
{
  static int8_t enc_states[] = {
    0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0    };
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                  //remember previous state
  old_AB |= (ENC_PORT1 & 0x30);  //add current state
  uint8_t new_AB=old_AB >> 4;
  return ( enc_states[( new_AB & 0x0f )]);
}

// returns change in encoder state (-1,0,1) */
int8_t read_encoder2()
{
  static int8_t enc_states[] = {
    0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0    };
  static uint8_t old_CD = 0;
  /**/
  old_CD <<= 2;                  //remember previous state
  old_CD |= (ENC_PORT2 & 0x30);  //add current state
  uint8_t new_CD=old_CD >> 4;
  return ( enc_states[( new_CD & 0x0f )]);
}

