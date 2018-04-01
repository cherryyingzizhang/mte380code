/////////////////////////////////////////////////////////////////////////////////////////////////
//Libraries
/////////////////////////////////////////////////////////////////////////////////////////////////
//Motors
#include <Servo.h>
//XBOX USB
#include <XBOXUSB.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

/////////////////////////////////////////////////////////////////////////////////////////////////
//Initialization of variables
/////////////////////////////////////////////////////////////////////////////////////////////////

//USB
USB Usb;
XBOXUSB Xbox(&Usb);

//Defining motors
#define NUMMOTORS 4
typedef struct MotorDef
{
    Servo   Motor; 
    int     Pin;   // Indicates the Pin this motor is connected to
};
MotorDef Motors[NUMMOTORS];

//ESC Settings
//Stores the settings for all ESC. Medium means "no movement". High/Low are max speed in cw/ccw dir.
typedef struct ESCSettingsDef
{
  int LowSpeed;
  int HighSpeed;
  int MediumSpeed;
};
ESCSettingsDef ESCSettings; 
//Specified baud rates for Afro ESC configuration
#define ESC_HIGH_DEFAULT 1860
#define ESC_LOW_DEFAULT 1060
#define ESC_MED_DEFAULT 1460
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Setup
/////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //Serial Monitor @ 115200 bps
  Serial.begin(115200);

  //Set up the motor pins
  Motors[0].Pin = 2; //z-axis left
  Motors[1].Pin = 3; //z-axis right
  Motors[2].Pin = 4; //left
  Motors[3].Pin = 5; //right
  for(int i = 0; i < NUMMOTORS; i++)
  {
    int pin = Motors[i].Pin;
    Motors[i].Motor.attach(pin);
  }

  //Set ESC settings
  ESCSettings.LowSpeed   = ESC_LOW_DEFAULT;
  ESCSettings.HighSpeed  = ESC_HIGH_DEFAULT;
  ESCSettings.MediumSpeed = ESC_MED_DEFAULT;

  //Set motor speeds to 0
  for (int i = 0; i < NUMMOTORS; i++)
  {
    Motors[i].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  }
  Serial.println("Motors initialized");

  /*if (Usb.Init() == -1) {
    Serial.println("OSC did not start");
    while (1); //halt
  }
  Serial.println("XBOX USB Library Started");*/
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Code for 4 Motor Movements
/////////////////////////////////////////////////////////////////////////////////////////////////
int change = 120;
int handicap = 5;

void Forward() {
  Serial.println("Forward");
  //updateIMUSensor();
  //forwardYaw = latestYaw; //get latest filtered yaw
  //goingForward = true;
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //right
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //z-axis right
}

void Backward() {
  Serial.println("Backward");
  //goingForward = false;
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //right
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //z-axis right
}

void Left() {
  Serial.println("Left");
  //goingForward = false;
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change + 20); //right
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //z-axis right
}

void Right() {
  Serial.println("Right");
  //goingForward = false;
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change - 20); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //right
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //z-axis right
}

void Up() {
  Serial.println("Up");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //z-axis right
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //right
}

void Down() {
  Serial.println("Down");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //z-axis right
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //right
}

void TiltRight() {
  Serial.println("Tilt Right");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //z-axis right
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //right
}

void TiltLeft() {
  Serial.println("Tilt Left");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //z-axis right
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //right
}

void StopLat() {
  Serial.println("Stop Lateral");
  //goingForward = false;
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
}

void StopZAxis() {
  Serial.println("Stop Z-Axis");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Test All Motors
/////////////////////////////////////////////////////////////////////////////////////////////////
void testAllMotors() {
  delay(20000);
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change - 20);
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  delay(3000);
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change - 20);
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  delay(3000);
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change - 20);
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  delay(3000);
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change - 20);
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  delay(3000);
  StopLat();
  StopZAxis();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Xbox Controller
/////////////////////////////////////////////////////////////////////////////////////////////////
void translateXbox() {
  if (Xbox.getButtonClick(B)) { //emergency stop
    StopLat();
    StopZAxis();
    delay(500);
  } else {
    if (Xbox.getAnalogHat(LeftHatX) > 7500) { //RIGHT
      Right();
    }
    else if (Xbox.getAnalogHat(LeftHatX) < -7500) { //LEFT
      Left();
    }
    else if (Xbox.getAnalogHat(LeftHatY) > 7500) { //FORWARD
      Forward();
    }
    else if (Xbox.getAnalogHat(LeftHatY) < -7500) { //BACKWARD
      Backward();
    }
    else 
    /*if (-7500 < Xbox.getAnalogHat(LeftHatX) && Xbox.getAnalogHat(LeftHatX) < 7500 &&
             -7500 < Xbox.getAnalogHat(LeftHatY) && Xbox.getAnalogHat(LeftHatY) < 7500 ) */
             {
      StopLat();
    }
  
    if (Xbox.getAnalogHat(RightHatX) > 7500) { //TILT RIGHT
      TiltRight();
    }
    else if (Xbox.getAnalogHat(RightHatX) < -7500) { //TILT LEFT
      TiltLeft();
    }
    else if (Xbox.getAnalogHat(RightHatY) > 7500) { //UP
      Up();
    }
    else if (Xbox.getAnalogHat(RightHatY) < -7500) { //DOWN
      Down();
    }
    else 
    /*if (-7500 < Xbox.getAnalogHat(RightHatX) && Xbox.getAnalogHat(RightHatX) < 7500 &&
             -7500 < Xbox.getAnalogHat(RightHatY) && Xbox.getAnalogHat(RightHatY) < 7500 ) */
             {
      StopZAxis();
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Loop
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //XBOX
  Usb.Task();
  if (Xbox.Xbox360Connected) { //Have we received an Xbox signal?
    translateXbox();
    delay(100);
  } else {
    StopLat();
    StopZAxis();
  }
  delay(1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
