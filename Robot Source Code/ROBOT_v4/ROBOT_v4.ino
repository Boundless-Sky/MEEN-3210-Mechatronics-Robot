/**************************************************************************************
 Remote Control
 Team: Jusitn Ngo, Dalton Ostler, Scott Ferguenson, Matthew Burns
 ME EN 3210: Mechantronics 2
 March, 2016
 ROBOT V4
 ***************************************************************************************/
/**************************************************************************************
 CHANGE LOG:
 Version 1 (Justin Ngo - March, 2016)
 
 Version 2 (Justin Ngo - March 28, 2016)
  - Refine motion control i.e. threshold to changing speed
  - Not using flushRecieve();
  - Changed data capturing. Doesn't turn off XBee to restart, uses while loops and clears
    until get the start command. Reduces the chances for communication to be off causing 
    robot to not recieve correct commands, forcing a hardware reset. See V1 to see the other way
    The while loop increase the reliability of connection
  - Changed to Serial3 on mega, pins 15(rx), 14(tx) which corresponds to pins 2, 3 respectively on XBee
  - fyi, if you want to change Serial3 to any other, run a sepearte test program so that Xbee will 
    "initialize" otherwise when you just change, the XBee on robot will experience a delay when recieving data

 Version 3 (Jusitn Ngo - March 30, 2016)
  - Status Lights (blue and green)
  - Hall effect sensor

 Version 4 (Justin Ngo - April 7, 2016)
 - revamping driving input for smoother contol
 - add Master and slave for forward and driving motion
 - removed stop bit
 - include encoder control
 - removed reciving grip data from pot
 - fixing the gripper jitter
   *NOTE: The terminal serial.begin() interferes with the servo library (timer/interrupt issues)
   *So when debugging uncomment the function debug() and the setup serial.begin()
 ***************************************************************************************/
/**************************************************************************************
 Hardware Hookup:
  The XBee Shield makes all of the connections you'll need
  between Arduino and XBee. If you have the shield make
  sure the SWITCH IS IN THE "DLINE" POSITION. That will connect
  the XBee's 2 and 3 pins to Arduino pins 15 and 14.
 ***************************************************************************************/

/**************************************************************************************
 Libraries
 ***************************************************************************************/
#include <Servo.h> 
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoder.h"
/**************************************************************************************
 Global Variables
 ***************************************************************************************/
Servo armServo;
Servo gripServo;

//pins 38 and 40
int gripMotion;
int armMotion;
int lastArm = 0;
int lastGrip = 0;

byte packets[4];

// Analog Input Variables
int X; //Joystick left and right movement
int Y; //Joystick up and down movement
int Arm=0; 
double alpha=0.9; //For arm PID control

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

//Status LED
const int GreenLED = 44; //Controller is active
const int BlueLED = 46; //Magnetic Sensor attach an interrupt on it...

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

//power pecentages (NOT USED)
const int power25 = 64;
const int power50 = 127;
const int power75 = 191;
const int power100 = 255;

//Hall Effect Sensor
int HallEffect = 46;
bool HallSensed = false;

//driving parameters
int threshold = 3; //deadzone parameter
double driveGain = 0.1;
double encoderDiff = 0;
double slaveSpeed;

//Encoder
Encoder myEncLeft(20,21);
Encoder myEncRight(18,19);
long newLeft, newRight;

/**************************************************************************************
 ***********************       SETUP          *****************************************
 ***************************************************************************************/
void setup() {
  // Set up both ports at 9600 baud. This value is most important
  // for the XBee. Make sure the baud rate matches the config
  // setting of your XBee.
  Serial3.begin(9600); //XBee
  //Serial.begin(9600); //Terminal
//  delay(1000); // Wait for electronics to power up, especially the motors
  allStop(); // motors are all off
  
  //Motor pin initialize
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  
  //Setup Servo
  armServo.attach(38);
  gripServo.attach(40);

  //PWM pin initialize
  pinMode(pwm9, OUTPUT);
  pinMode(pwm10, OUTPUT);

  //Status Light initialize
  pinMode(GreenLED, OUTPUT);
  pinMode(BlueLED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(3), pin_ISR, CHANGE);

  //Set Encoders to 0
  myEncLeft.write(0);
  myEncRight.write(0);
  
}
/**************************************************************************************
 ***********************      MAIN LOOP        *****************************************
 ***************************************************************************************/
void loop()
{
  recieveData();
  motionData();
  //debug();
}


/**************************************************************************************
 ***********************      FUNCTIONS        *****************************************
 ***************************************************************************************/

void debug() {
//    Serial.print("X = ");           Serial.print(X);
//    Serial.print(" Y = ");          Serial.print(Y);
//    Serial.print(" Arm = ");        Serial.print(Arm);
//    Serial.print(" D4 = ");         Serial.print(button4);
//    Serial.print(" D5 = ");         Serial.print(button5);
//    Serial.print(" D6 = ");         Serial.print(button6);
//    Serial.print(" buttonByte: ");  Serial.print(buttonByte, DEC);
//    Serial.println("");
//    Serial.println("");
}
/**************************************************************************************
 pin interrupt service routine
 ***************************************************************************************/
void pin_ISR() {
  digitalWrite(HallEffect, HIGH);
  HallSensed = true;
}
/**************************************************************************************
 Recieve data from the controller and puts it into corresponding variables. The limitations 
 is that this code is "hard-coded". This means that it can't adapt readily to changes. 
 A better way to adapt would be using while loops and counting the number of bytes that are comming in
 and storing them accordingly
 ***************************************************************************************/
void recieveData() {
  delay(20);
  while (Serial3.available() > 4) { //IF recieved packets is atleast 5 packets
    if (Serial3.read() != START) {
      digitalWrite(GreenLED, LOW); //connection status light (green) is off
      continue;
    }
    digitalWrite(GreenLED, HIGH);
    //Note: With C, arrays start at 0
    packets[0] = Serial3.read(); //X
    packets[1] = Serial3.read(); //Y
    packets[2] = Serial3.read(); //Arm
    packets[3] = Serial3.read(); //Button Byte
    X = map(packets[0], 0, 254, -255, 255);
    Y = map(packets[1], 0, 254, -255, 255);
    Arm = (1-alpha)*packets[2]+alpha*Arm; // This is PID control , smoother physical arm motion
    buttonByte = packets[3];
    DecodeButton(buttonByte, &button7, &button6, &button5, &button4, &button3, &button2, &button1, &button0);
    if (HallSensed = true) {
      HallSensed = false;
      digitalWrite(HallEffect, LOW);
    }
  }//end while
}//end recieveData

/**************************************************************************************
 Move/actuate robot
 Note: using the negatives of X & Y since they actually go to 255. 
 ***************************************************************************************/
void motionData() {
  //Sationary: Center of Joystick
  if ((X > -threshold) && (X < threshold) && (Y > -threshold) && (Y < threshold)){
    allStop();
  }
  //SPIN
  if ((Y > (-threshold)) && (Y < (threshold))) {
    //Clockwise, looking at robot from top
    if (X > 0) {
      analogWrite(pwm9, X );
      analogWrite(pwm10, X );
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
    }
    //Counter-Clockwise, looking at robot from top
    if (X < 0) {
      analogWrite(pwm9, -X );
      analogWrite(pwm10, -X );
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
    }
  }//end spin routine

  //Drive forward  or backward
  //Right Motor (10 = Master
  //Left Motor (9) = Slave
  if ((X > -threshold) && (X < threshold)) {
    newRight = myEncRight.read();
    newLeft = myEncLeft.read();
    encoderDiff = newRight - newLeft;
    slaveSpeed = Y + (encoderDiff * driveGain);
    //Forward
    if (Y > 0) {
      analogWrite(pwm9, slaveSpeed );
      analogWrite(pwm10, Y);
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
    }
    myEncRight.write(0);
    myEncLeft.write(0);
  }
  if ((X > -threshold) && (X < threshold)) {
    newRight = myEncRight.read();
    newLeft = myEncLeft.read();
    encoderDiff = newRight - newLeft;
    slaveSpeed = Y + (encoderDiff * driveGain);
    //Backward
    if (Y < 0) {
      analogWrite(pwm9, -slaveSpeed);
      analogWrite(pwm10, -Y);
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
    }
    myEncRight.write(0);
    myEncLeft.write(0);
  }
  
  //Moving in an ARC
  //Up - Right
  if (X > threshold && Y > threshold) {
    analogWrite(pwm9, Y );
    analogWrite(pwm10, 0 );
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, HIGH);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  }
   //Up - Left
  if (X < -threshold && Y > threshold) {
    analogWrite(pwm9, 0 );
    analogWrite(pwm10, Y );
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, HIGH);
  }
   //Down - Right
  if (X > threshold && Y < -threshold) {
    analogWrite(pwm9, -Y );
    analogWrite(pwm10, 0 );
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, LOW);
    digitalWrite(rightMotor2, LOW);
  }
   //Down - Left
  if (X < -threshold && Y < -threshold) {
    analogWrite(pwm9, 0 );
    analogWrite(pwm10, -Y);
    digitalWrite(leftMotor1, LOW);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
  }
  
  servoMotion();
}//end motionData

/**************************************************************************************
 Actuate servo on the Arm and the gripper
 ***************************************************************************************/
void servoMotion() {
  //gripper motion control
  //NOT USING Gripper potentiometer since button use seems easier... may change
  if ((button6 == true) || (button5 == true) || (button4 == true)) {
    // Middle grip
    if (button6 == true) {
      gripMotion = 1700; // 
      gripServo.write(gripMotion);
      delay(20);
    }
    // Open grip
    if (button5 == true) {
      gripMotion = 1000; 
      gripServo.write(gripMotion);
      delay(20);
    }
    // Close Grip
    if (button4 == true) { 
      gripMotion = 2000;
      gripServo.write(gripMotion);
      delay(20);
    }
  }
  
  //Arm Motion, Only moves the arm if the position is different by a margin, prevents accidental 
  //potentiometer movement
  if (armMotion != lastArm+3) {
    armMotion = Arm;
    armServo.write(armMotion);
    lastArm = armMotion;
    delay(20);
  }
}//end servoMotion
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

