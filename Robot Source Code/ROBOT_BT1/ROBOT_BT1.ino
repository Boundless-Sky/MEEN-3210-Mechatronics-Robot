
/**************************************************************************************
 Robot Control - PS4 Bluetooh Controlled
 August 13, 2016
 ROBOT VERSION: BT1
 ***************************************************************************************/
/**************************************************************************************
 CHANGE LOG:
 BT1: See Robot Control for previous iterations
 - added a library that works with pololu 5019b motor shield board
 - added usb host shield and dongle to work in conjuction with the ps4 controller
 ***************************************************************************************/
 /**************************************************************************************
 Hardware Hookup:
 - requires, ps4 controller, circuits@home usb host shield and generic bluetooth dongle
 ***************************************************************************************/
 /**************************************************************************************
 Libraries
 ***************************************************************************************/
#include <PS4BT.h>
#include <usbhub.h>
#include <Servo.h> //included in arduino IDE
#include <DualVNH5019MotorShield.h>
DualVNH5019MotorShield md; //calls the board as md. Can be other names like bob

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
//PS4BT PS4(&Btd);

Servo armServo;
int armMotion = 0;
int lastArm = 0;
/**************************************************************************************
 Global Variables
 ***************************************************************************************/
bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

//Status LED
const int GreenLED = 44; //Controller is active
const int BlueLED = 46;

int DriveLeft, DriveRight, DriveMode, DriveX, DriveY;



/**************************************************************************************
 ***********************       SETUP          *****************************************
 ***************************************************************************************/
void setup() {
  md.init(); 
  Serial.begin(115200);

  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
    if (Usb.Init() == -1) {
      Serial.print(F("\r\nOSC did not start"));
      while (1); // Halt
    }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  
  //Status Light initialize
  pinMode(GreenLED, OUTPUT); 
  pinMode(BlueLED, OUTPUT);
  armServo.attach(38);
  armMotion = constrain(armMotion, 0, 116);
  armServo.write(armMotion);
}
/**************************************************************************************
 ***********************      FUNCTIONS        *****************************************
 ***************************************************************************************/
void TankDrive() {
  if (PS4.getAnalogHat(LeftHatY) > 137 || PS4.getAnalogHat(LeftHatY) < 117) {
    DriveLeft = map(PS4.getAnalogHat(LeftHatY), 0, 255, -400, 400);
    md.setM1Speed(DriveLeft);
    delay(2);
  }
  if (PS4.getAnalogHat(RightHatY) > 137 || PS4.getAnalogHat(RightHatY) < 117) {
    DriveRight = map(PS4.getAnalogHat(RightHatY), 0, 255, -400, 400);
    md.setM2Speed(DriveRight);
    delay(2);
  }

  if (PS4.getAnalogHat(LeftHatY) < 137 && PS4.getAnalogHat(LeftHatY) > 117) {
    DriveLeft = 0;
    md.setM1Speed(DriveLeft);
  }

  if (PS4.getAnalogHat(RightHatY) < 137 && PS4.getAnalogHat(RightHatY) > 117) {
    DriveRight = 0;
    md.setM2Speed(DriveRight);
  }
 
 if (DriveRight != 0 || DriveLeft != 0){
    PS4.setLed(Green);
 }
 else {
    PS4.setLed(Red);
 }
}// end TankDrive()

void SingleDrive() {
  DriveX = map(PS4.getAnalogHat(LeftHatX), 0, 255, -400, 400);
  DriveY = map(PS4.getAnalogHat(LeftHatY), 0, 255, -400, 400);
  if (DriveY > 5) {
    md.setM1Speed(DriveY - DriveX);
    md.setM2Speed(DriveY + DriveX);
  }
  if (DriveY < -5) {
    md.setM1Speed(DriveY + DriveX);
    md.setM2Speed(DriveY - DriveX);
  }
  if (DriveY > -5 && DriveY < 5) {
    DriveX = 0;
    DriveY = 0;
    md.setM1Speed(0);
    md.setM2Speed(0);
  }
  if (DriveX != 0 || DriveY != 0){
    PS4.setLed(Blue);
  }
  else {
    PS4.setLed(Yellow);
  }
}// end SingleDrive()

void ThrottleDrive() {
  if (PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)) { // These are the only analog buttons on the PS4 controller
    md.setM1Speed(-1*(PS4.getAnalogButton(L2)));
    md.setM2Speed(-1*(PS4.getAnalogButton(R2)));
  }
  if (PS4.getAnalogButton(L2) != oldL2Value || PS4.getAnalogButton(R2) != oldR2Value){ // Only write value if it's different
    PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
    oldL2Value = PS4.getAnalogButton(L2);
    oldR2Value = PS4.getAnalogButton(R2);
  }
}//end throttle drive

void DanceParty(){
  md.setM1Speed(-400);
  md.setM2Speed(400);
  digitalWrite(BlueLED, HIGH);
  PS4.setLedFlash(10, 10); // Set it to blink rapidly
  delay(200);
  digitalWrite(BlueLED,LOW);
  DriveMode = 1;
}//end dance party

/**************************************************************************************
 ***********************      MAIN LOOP        *****************************************
 ***************************************************************************************/
void loop() {
  Usb.Task();

  if (PS4.connected()) {
    digitalWrite(GreenLED, HIGH);
    delay(10);//allow time for robot to actually execute the command

    if (PS4.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS4.disconnect();
      digitalWrite(GreenLED, LOW);
      PS4.setLedOff();
    }
    
    if (PS4.getButtonClick(UP)) {
        DriveMode = 1;
    } 
    if (PS4.getButtonClick(RIGHT)) {
        DriveMode = 2;
    } 
    if (PS4.getButtonClick(DOWN)) {
        DriveMode = 3;
    } 
    if (PS4.getButtonClick(LEFT)) {
        DriveMode = 4;
    }

    if (PS4.getButtonPress(L1)) {
      armMotion -= 1;
      armServo.write(armMotion);
    }
    if (PS4.getButtonPress(R1)) {
      armMotion += 1;
      armServo.write(armMotion);
      Serial.println(armMotion);
    }
    
    switch (DriveMode) {
      case 1: //tank drive
        TankDrive();
        break;
      case 2: //left joystick drive
        SingleDrive();
        break;
      case 3: //throtle drive
        ThrottleDrive();
        break;
      case 4: //Dance party
        DanceParty();
        break;
      default: //tank drive
        TankDrive();
        break;
    }//end switch case
           
    if (PS4.getButtonClick(TRIANGLE)) {
      Serial.print(F("\r\nTraingle"));
      PS4.setRumbleOn(RumbleLow);
    }
    if (PS4.getButtonClick(CIRCLE)) {
      Serial.print(F("\r\nCircle"));
      PS4.setRumbleOn(RumbleHigh);
    }
    if (PS4.getButtonClick(CROSS)) {
      Serial.print(F("\r\nCross"));
      PS4.setLedFlash(10, 10); // Set it to blink rapidly
    }
    if (PS4.getButtonClick(SQUARE)) {
      Serial.print(F("\r\nSquare"));
      PS4.setLedFlash(0, 0); // Turn off blinking
    }
  }//end ps4 connected
}//end loop
