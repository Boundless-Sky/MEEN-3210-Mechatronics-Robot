/**************************************************************************************
 Remote Control
 Author: Jusitn Ngo, Dalton Ostler, Scott Ferguenson, Matthew Burns
 ME EN 3210: Mechantronics 2
 March, 2016
 ROBOT V1
 ***************************************************************************************/

/**************************************************************************************
 Hardware Hookup:
  The XBee Shield makes all of the connections you'll need
  between Arduino and XBee. If you have the shield make
  sure the SWITCH IS IN THE "DLINE" POSITION. That will connect
  the XBee's 2 and 3 pins to Arduino pins 19 and 18.
 ***************************************************************************************/

/**************************************************************************************
 Libraries
 ***************************************************************************************/
#include <Servo.h>
/**************************************************************************************
 Global Variables
 ***************************************************************************************/
Servo armServo;
Servo gripServo;
//pins 38 and 40
int gripMotion = 0;
int armMotion = 0;
int lastArm = 0;
int lastGrip = 0;

#define MaxPacketNum 10
byte packets[MaxPacketNum];

// Analog Input Variables
int X; //Joystick left and right movement
int Y; //Joystick up and down movement
int Grip=0;
int Arm=0; 
int threshold = 3; //deadzone parameter

//Any other buttons besides 4, 5 and 6 are just place holders unless later on you decide to add more buttons
//named 0-7 instead of 1-8 is because it is follows bit math.. 
//i.e. 2^7 + 2^6 + 2^5 + 2^4 + 2^3 + 2^2 + 2^1 + 2^0 = 255
//remember 0 = false, 1 = true
bool button0 = false;
bool button1 = false;
bool button2 = false;
bool button3 = false;
bool button4 = false;
bool button5 = false;
bool button6 = false;
bool button7 = false;

byte buttonByte;
const byte START= 255;
const byte STOP= 254;

//Status LED
// ~~~~~~~~~~~~~ THESE BATCH NEEDS PIN NUMBERS ~~~~~~~~~~~~~~~~~~~~~
//const int GreenLED; //Controller is active
//const int BlueLED; //Magnetic Sensor

//Motor pins (digital), Run with motor-driver sheild
//Usage: Motors used has two leads need to switch HIGH or LOW accordingly to get desired direction
// Left Motor:  leftMotor1 HIGH & leftMotor2 LOW =>  FORWARD  
// Left Motor:  leftMotor2 HIGH & leftMotor1 LOW =>  REVERSE
// Right Motor: rightMotor1 HIGH & rightMotor2 LOW =>  FORWARD 
// Right Motor: rightMotor2 HIGH & rightMotor1 LOW =>  REVERSE
#define leftMotor1 2
#define leftMotor2 4
#define rightMotor1 7
#define rightMotor2 8

//PWM pins (analog) on Arduino
//Usage: PWM signal from 0 - 255, 
// where 127 = 50%  power 
//       191 = 75%  power
//       255 = 100% power
#define pwm9 9   //left motor
#define pwm10 10 //right motor

const int power25 = 64;
const int power50 = 127;
const int power75 = 191;
const int power100 = 255;

//Hall Effect Sensor
//int HallEffect;

/**************************************************************************************
 ***********************       SETUP          *****************************************
 ***************************************************************************************/
void setup() {
  // Set up both ports at 9600 baud. This value is most important
  // for the XBee. Make sure the baud rate matches the config
  // setting of your XBee.
  Serial1.begin(9600); //XBee
  Serial.begin(9600); //Terminal
  delay(1000); // Wait for electronics to power up, especially the motors
  allStop(); // motors are all off

  //Status lights initialize
//  pinMode(BlueLED, OUTPUT);
//  pinMode(GreenLED, OUTPUT);

  //Motor pin initialize
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);

  //Setup Servo
  armServo.attach(38);
  gripServo.attach(40);

  //SetServo to zero
  
  
  //PWM pin initialize
  pinMode(pwm9, OUTPUT);
  pinMode(pwm10, OUTPUT);
  
  
}
/**************************************************************************************
 ***********************      MAIN LOOP        *****************************************
 ***************************************************************************************/
void loop()
{
  recieveData();
  motionData();
//  HallEffect = analogRead(A8);
  debug();
}


/**************************************************************************************
 ***********************      FUNCTIONS        *****************************************
 ***************************************************************************************/

void debug() {
    Serial.print("X = ");           Serial.print(X);
    Serial.print(" Y = ");          Serial.print(Y);
    Serial.print(" Grip = ");       Serial.print(Grip);
    Serial.print(" Arm = ");        Serial.print(Arm);
    Serial.print(" D4 = ");         Serial.print(button4);
    Serial.print(" D5 = ");         Serial.print(button5);
    Serial.print(" D6 = ");         Serial.print(button6);
    Serial.print(" buttonByte: ");  Serial.print(buttonByte, DEC);
    Serial.print(" packet[0]: ");   Serial.print(packets[0]);
    Serial.print(" packet[6]: ");   Serial.print(packets[6]);
    Serial.println("");
    Serial.println("");
//Serial.print("HallEffect: ");
//Serial.print(HallEffect);
}


/**************************************************************************************
 Recieve data from the controller and puts it into corresponding variables. The limitations 
 is that this code is "hard-coded". This means that it can't adapt readily to changes. 
 A better way to adapt would be using while loops and counting the number of bytes that are comming in
 and storing them accordingly
 ***************************************************************************************/
void recieveData() {
  if (Serial1.available() > 0) {
    packets[0] = Serial1.read(); 
    if (packets[0] == START){
      packets[1] = Serial1.read();
      packets[2] = Serial1.read(); 
      packets[3] = Serial1.read(); 
      packets[4] = Serial1.read(); 
      packets[5] = Serial1.read(); 
      packets[6] = Serial1.read(); 
      X = map(packets[1], 0, 253, -255, 255);
      Y = map(packets[2], 0, 253, -255, 255);
      Grip = packets[3];
      double alpha=0.9;
      Arm = (1-alpha)*packets[4]+alpha*Arm;
      buttonByte = packets[5];
      DecodeButton(buttonByte, &button7, &button6, &button5, &button4, &button3, &button2, &button1, &button0);
      delay(10);
/**************************************************************************************
  Clear the recieveing data and start a new
      EXTREMELY Crude way to do so, but it works.
      Hopefully my progrommming skills will get better...
      Do not know the effects this method has on the hardware.

      FYI, so the problem that was being ran into without using this method 
      was that the recieving buffer (Serial1) kept on overflowing
      turn on debug function and serial monitor and look at START.
      START will periodically recieve numbers that are not 255
 ***************************************************************************************/
     
      if (packets[6] == STOP) {
       //flushRecieve();
       Serial1.end();
       Serial1.begin(9600);
       delay(10);
      }
    }
    if (packets[0] != START) {
      Serial1.end();
      Serial1.begin(9600);
      delay(10);
    }
  }
}

/**************************************************************************************
 Move/actuate robot
 Note: using the negatives of X & Y since they actually go to 255. 
 ***************************************************************************************/
void motionData() {
  //Correction factor on the postive side since we had limited the receiving data to 253.
  //correction factor will make the positive side add up to 255 after mapping it. 
  const int factor = 3;
  int left = Y;
  int right = Y;
  //Sationary: Center of Joystick
  if ((X > -threshold) && (X < threshold) && (Y > -threshold) && (Y < threshold)){
    allStop();
    Serial.print("STOPPED");
    Serial.println("");
    delay(100);
  }
  //Moving in an ARC
  //Up - Right
  if (X > (threshold+factor) && Y > (threshold+factor)) {
    left = (X+3);
    right = (X+3)/2;
    analogWrite(pwm9, left );
    analogWrite(pwm10, right );
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    Serial.print("UP-RIGHT: "); Serial.print(left); Serial.print(right);
    Serial.println("");
    delay(100);
  }
   //Up - Left
  if (X < (-threshold-factor) && Y > (threshold+factor)) {
    left = -(X-3)/2;
    right = -(X-3);
    analogWrite(pwm9, (left) );
    analogWrite(pwm10, (right) );
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
    Serial.print("UP-LEFT: "); Serial.print(left); Serial.print(right);
    Serial.println("");
    delay(100);
  }
   //Down - Right
  if (X > (threshold+factor) && Y < (-threshold-factor)) {
    left = (X+3);
    right = (X+3)/2;
    analogWrite(pwm9, left );
    analogWrite(pwm10, right );
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    Serial.print("DOWN-RIGHT: "); Serial.print(left); Serial.print(right);
    Serial.println("");
    delay(100);
  }
   //Down - Left
  if (X < (-threshold-factor) && Y < (-threshold-factor)) {
    left = -(X-3)/2;
    right = -(X-3);
    analogWrite(pwm9, (left) );
    analogWrite(pwm10, (right) );
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    Serial.print("DOWN-LEFT: "); Serial.print(left); Serial.print(right);
    Serial.println("");
    delay(100);
  }
  

  //SPIN
  if ((Y > (-threshold-factor)) && (Y < (threshold+factor))) {
    //Clockwise, looking at robot from top
    if (X > (threshold+factor)) {
      analogWrite(pwm9, (X+factor) );
      analogWrite(pwm10, (X+factor) );
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      Serial.print("CW: "); Serial.print(X+factor);
      Serial.println("");
      delay(100);
    }
    //Counter-Clockwise, looking at robot from top
    if (X < (-threshold-factor)) {
      analogWrite(pwm9, -X );
      analogWrite(pwm10, -X );
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      Serial.print("CCW: "); Serial.print(-X);
      Serial.println("");
      delay(100);
    }
  }

  //Drive forward backward
  if ((X > -threshold) && (X < threshold)) {
    //Forward
    if (Y > threshold) {
      analogWrite(pwm9, (Y+factor) );
      analogWrite(pwm10, (Y+factor) );
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      Serial.print("FOWARD"); Serial.print(Y+factor);
      Serial.println("");
      delay(100);
    }
    //Backward
    if (Y < -threshold) {
      analogWrite(pwm9, -Y);
      analogWrite(pwm10, -Y);
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      Serial.print("Reverse"); Serial.print( -Y);
      Serial.println("");
      delay(100);
    }
  }
  servoMotion();
}
/**************************************************************************************
 Actuate servo on the Arm and the gripper
 ***************************************************************************************/
void servoMotion() {
  //gripper motion control
  if ((button6 == true) || (button5 == true) || (button4 == true)) {
    if (button6 == true) {
      gripMotion = 1500;
      gripServo.write(gripMotion);
    }
    if (button5 == true) {
      gripMotion = 1000;
      gripServo.write(gripMotion);
    }
    if (button4 == true) {
      gripMotion = 2000;
      gripServo.write(gripMotion);
    }
   
  }
  //arm motion
  if (armMotion != lastArm+3) {
    armMotion = Arm;
    armServo.write(armMotion);
    gripServo.write(gripMotion);
    lastArm = armMotion;
    delay(15);
  }
}
/**************************************************************************************
 Usage: DecodeButton(buttonByte, button7, button6, button5, button4, button3, button2, button1, button0);
 Note: Dont get bit place and button number confused, two seperate concepts
 basically this decodes starting from the most significant bit (bit 8). Then converts and returns as 0 or 1
 ***************************************************************************************/
void DecodeButton(byte btnByte, bool *bit8, bool *bit7, bool *bit6, bool *bit5, bool *bit4, bool *bit3, bool *bit2, bool *bit1){
  byte sum = 0;

  *bit8 = btnByte >> 7;
  
  sum += (*bit8)*128;
  *bit7 = (btnByte-sum) >> 6;
  
  sum += (*bit7)*64;
  *bit6 = btnByte-sum >> 5;
  
  sum += (*bit6)*32;
  *bit5 = btnByte-sum >> 4;
  
  sum += (*bit5)*16;
  *bit4 = btnByte-sum >> 3;
  
  sum += (*bit4)*8;
  *bit3 = btnByte-sum >> 2;
 
  sum += (*bit3)*4;
  *bit2 = btnByte-sum >> 1;
  
  sum += (*bit2)*2;
  *bit1 = btnByte-sum >> 0;
}
/**************************************************************************************
 Turns off all wheels
 ***************************************************************************************/
void allStop() {
  analogWrite(pwm9, 0);
  analogWrite(pwm10, 0);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
}
/**************************************************************************************
 One way to clear recieving data, most likely not going to use so comment it out (for future reference)
 ***************************************************************************************/
void flushRecieve() {
  while (Serial1.available()){
    Serial1.read(); 
  }
}

