/**************************************************************************************
 Remote Control
 Author: Jusitn Ngo, Dalton Ostler, Scott Ferguenson, Matthew Burns
 ME EN 3210: Mechantronics 2
 March, 2016
 REMOTE V1
 ***************************************************************************************/

/**************************************************************************************
 Hardware Hookup:
  The XBee Shield makes all of the connections you'll need
  between Arduino and XBee. If you have the shield make
  sure the SWITCH IS IN THE "DLINE" POSITION. That will connect
  the XBee's DOUT and DIN pins to Arduino pins 2 and 3.
 ***************************************************************************************/

/**************************************************************************************
 Using SoftwareSerial to communicate with the XBee:
 XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
 XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
 ***************************************************************************************/
#include <SoftwareSerial.h>
SoftwareSerial XBee(2,3); // RX, TX

/**************************************************************************************
 Global Variables
 ***************************************************************************************/
// Digitial Variables
const int RedLED = 13; //Power on
const int GreenLED = 12; //Controller is active

// Buttons (D4 - D6)
// Sparkfun Joystick only has button 2,3,4,5,6 which button 2, 3 were not using
const int D4 = 4;
const int D5 = 5;
const int D6 = 6;

// Analog Input Variables
int X; //Joystick left and right movement
int Y; //Joystick up and down movement
int Grip;
int Arm; 

int threshold = 10; //dead zone size in which joystick is at center

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

byte buttonByte = 0;

//note that this works since buttons 4, 5, 6 combined will not equal 254 or 255
const byte START = 255;
const byte STOP = 254;

/**************************************************************************************
 ***********************       SETUP          *****************************************
 ***************************************************************************************/
 
void setup() {
  XBee.begin(9600); //Xbee
  Serial.begin(9600); //Terminal

  //Set lights as ouputs
  pinMode(RedLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  //Power is on
  digitalWrite(RedLED, HIGH); 

/**************************************************************************************
  Intialize button as inputs and enable arduino's interal "pull-up" resistors
  so that means you don't need additional resistors added to the board
      - HIGH = Button not pressed, 1
      - LOW = Button pressed, 0
      *note that this is later switched in captureData()
 ***************************************************************************************/
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  digitalWrite(D4, HIGH);
  digitalWrite(D6, HIGH);
  digitalWrite(D5, HIGH);
}

/**************************************************************************************
 ***********************      MAIN LOOP        *****************************************
 ***************************************************************************************/
void loop() {
  captureData();
  active();
  //debug();
}

/**************************************************************************************
 ***********************      FUNCTIONS        *****************************************
 ***************************************************************************************/
 
/************************************************************************************
 Turns on green LED to indicate that the controller is active as per ME EN 3210 Rules
 ************************************************************************************/
void active() {
  sendData();
  //blink Green LED to indicate that it is active
  digitalWrite(GreenLED, HIGH); delay(10);
  digitalWrite(GreenLED, LOW);  delay(10);
}

/**************************************************************************************
 Transmit data over to the robot
 ***************************************************************************************/
void sendData() {
  byte sendX = map(X, 0, 1023, 0, 253);
  byte sendY = map(Y, 0, 1023, 0, 253);
  byte sendGrip = map(Grip, 0, 1023, 0, 253);
  byte sendArm = map(Arm, 0, 1023, 0, 253);

//  Debug sendData()
  Serial.print(" X = ");        Serial.print(sendX);
  Serial.print(" Y = ");        Serial.print(sendY);
  Serial.print(" Grip = ");     Serial.print(sendGrip);
  Serial.print(" Arm = ");      Serial.print(sendArm);
  Serial.print(" D4 = ");       Serial.print(button4);
  Serial.print(" D5 = ");       Serial.print(button5);
  Serial.print(" D6 = ");       Serial.print(button6);
  Serial.print(" buttonByte: ");Serial.print(buttonByte, DEC);
  Serial.print(" START: ");     Serial.print(START);
  Serial.print(" STOP: ");      Serial.print(STOP);
  Serial.println("");
  Serial.println("");
  
  //remember this is the protocol, so when robot is reciving decipher in the same order
  XBee.write(START);
  XBee.write(sendX);
  XBee.write(sendY);
  XBee.write(sendGrip);
  XBee.write(sendArm);
  XBee.write(buttonByte);
  XBee.write(STOP);
}

/**************************************************************************************
 Get inputs
 ***************************************************************************************/
void captureData() {
  X = analogRead(A0);
  Y = analogRead(A1);
  Grip = analogRead(A4);
  Arm = analogRead(A5);
  

  //sets dead-band zone
  if ((X > (512 - threshold)) && (X < (512 + threshold)))
    { X = 512; }
  if ((Y > (512 - threshold)) && (Y < (512 + threshold)))
    { Y = 512; }
    
  //convert btn from 1 - not pressed to being 0 - not pressed
  if (digitalRead(D4) == 1) {button4 = 0;}
    else {button4 = 1;}
  if (digitalRead(D5) == 1) {button5 = 0;}
    else {button5 = 1;}
  if (digitalRead(D6) == 1) {button6 = 0;}
    else {button6 = 1;}

 /**************************************************************************************
  encode buttons into locations in a byte (8 bits)
  we have 8 bits in a byte, and we have buttons 4, 5, and 6
  so for example if button 4 is pressed we get 0001 0000
  likewise if we get button 6 we get 0100 0000
  see http://blog.oscarliang.net/robotic-remote-controller-protocol-design/
  ***************************************************************************************/
  buttonByte = EncodeButton(button7, button6, button5, button4, button3, button2, button1, button0);
}

/**************************************************************************************
 Convert 8 buttons into binary and then to byte number for transmitting
 bit 1-8 are going to be 0's or 1's
 so if button4 is pressed EncodeButton should return 2^4 = 16 => 
 sum which will be passed as binary as 0001 0000
 if button 4 and 5 is pressed you get 2^5 + 2^4 = 48 => sum = 0011 0000
 Note: serial monitor will remove 0's infront of the first non-zero value
 ***************************************************************************************/
byte EncodeButton(bool bit8, bool bit7, bool bit6, bool bit5, bool bit4, bool bit3, bool bit2, bool bit1){
  byte sum = 0;
  //convert to binary
  sum += bit8 << 7;
  sum += bit7 << 6;
  sum += bit6 << 5;
  sum += bit5 << 4;
  sum += bit4 << 3;
  sum += bit3 << 2;
  sum += bit2 << 1;
  sum += bit1 << 0;
  return sum;
}


/**************************************************************************************
 Prints variable values on serial monitor
 ***************************************************************************************/
void debug() {
    Serial.print("X = ");         Serial.print(X);
    Serial.print(" Y = ");        Serial.print(Y);
    Serial.print(" Grip = ");     Serial.print(Grip);
    Serial.print(" Arm = ");      Serial.print(Arm);
    Serial.print(" D4 = ");       Serial.print(button4);
    Serial.print(" D5 = ");       Serial.print(button5);
    Serial.print(" D6 = ");       Serial.print(button6);
    Serial.print(" buttonByte: ");Serial.print(buttonByte, DEC);
    Serial.println("");
    Serial.println("");
}


