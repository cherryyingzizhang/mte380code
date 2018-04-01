/////////////////////////////////////////////////////////////////////////////////////////////////
//Libraries
/////////////////////////////////////////////////////////////////////////////////////////////////
//IMU
#include <MPU9250.h>
#include <quaternionFilters.h>
//Motors
#include <Servo.h>
//Remote Control
#include "IRremote.h"
//Pressure Sensor
#include <SparkFun_MS5803_I2C.h>
#include <Wire.h>
//XBOX USB
#include <XBOXUSB.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////
//Initialization of variables
/////////////////////////////////////////////////////////////////////////////////////////////////
void updateIMUSensor(void);

bool turnOffXbox = false;

//USB
USB Usb;
XBOXUSB Xbox(&Usb);

//IR Remote-Controller
int receiver = 7; //pin number for IR receiver
IRrecv irrecv(receiver);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'

//IMU Sensor
MPU9250 imuSensor;
float mDirection;
float mX, mY, mZ;
float northYaw, forwardYaw;

//Pressure sensor
MS5803 pressureSensor(0x76);
//Create variables to store results
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
double base_altitude = 329.0; // Altitude of Waterloo Ontario (m)
float latestPressureDepth;

//Filter Library
#define SMA_LENGTH 5
float yawfilter[SMA_LENGTH]; //magnometer X
float pitchfilter[SMA_LENGTH]; //magnometer Y
float rollfilter[SMA_LENGTH]; //magnometer Z
float latestYaw = 0;

float aXfilter[SMA_LENGTH]; //A X
float aYfilter[SMA_LENGTH]; //A Y
float aZfilter[SMA_LENGTH]; //A Z

float pFilter[SMA_LENGTH]; //pressure

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
  Serial.begin(9600);

  /*if (Usb.Init() == -1) {
    Serial.println("OSC did not start");
    while (1); //halt
  }*/
  Serial.println("XBOX USB Library Started");
  
  irrecv.enableIRIn(); // Start the receiver
  Serial.println("IR receiver enabled");

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

  //Initialize the pressure sensor
  pressureSensor.reset();
  pressureSensor.begin();
  pressure_baseline = pressureSensor.getPressure(ADC_4096);
  Serial.println("Pressure sensor initialized");

  //Initialize IMU sensor
  Wire.begin(); //initialized the arduino board itself as a master
  Wire.begin(0x68); //imu
  Wire.begin(0x76); //pressure
  Serial.println("I2C initialized");

  int intPin = 13;
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  Serial.println("I2C initialized");
  
  //imuSensor.MPU9250SelfTest(imuSensor.selfTest);
  imuSensor.calibrateMPU9250(imuSensor.gyroBias, imuSensor.accelBias);
  imuSensor.initMPU9250();
  imuSensor.initAK8963(imuSensor.factoryMagCalibration);
  imuSensor.getAres();
  imuSensor.getGres();
  imuSensor.getMres();
  imuSensor.magBias[0] = 164.89;
  imuSensor.magBias[1] = 159.51;
  imuSensor.magBias[2] = 167.60;
  imuSensor.magScale[0] = 0.71;
  imuSensor.magScale[1] = 0.82;
  imuSensor.magScale[2] = 2.78;
  imuSensor.factoryMagCalibration[0] = 1.20;
  imuSensor.factoryMagCalibration[1] = 1.20;
  imuSensor.factoryMagCalibration[2] = 1.15;
  Serial.println("Calibrating north...");
  updateIMUSensor();
  northYaw = imuSensor.yaw;
  Serial.println("North Calibrated To Be: " + String(northYaw));
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//IR
/////////////////////////////////////////////////////////////////////////////////////////////////
void translateIR() // takes action based on IR code received
// describing Remote IR codes 
{
  switch(results.value)
  {
  case 0xFF6897: //0
    StopZAxis();   
    break;
  case 0xFF30CF: //1
    Up();
    break;
  case 0xFF18E7: //2
    Forward();
    break;
  case 0xFF7A85: //3
    Down();
    break;
  case 0xFF10EF: //4
    Left();
    break;
  case 0xFF38C7: //5
    StopLat();
    break;
  case 0xFF5AA5: //6
    Right();
    break;
  case 0xFF42BD: //7
    TiltLeft(); 
    break;
  case 0xFF4AB5: //8
    Backward();
    break;
  case 0xFF52AD: //9 
    TiltRight();
    break;
  ///////////////////////////
  case 0xFF9867: //EQ
    /*updateIMUSensor();
    northYaw = yawfilter[SMA_LENGTH-1];
    Serial.println("New North: " + String(northYaw));*/
    startAutonomous();
    break;
  case 0xFFB04F: //ST/REPT
    pointNorth("north", millis() + 10000); 
    break; 
  case 0xFF02FD: //play/pause
    turnOffXbox = true;
    break;
  }// End Case
  delay(100);
  StopLat();
  StopZAxis();
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//FILTER FUNCTION
/////////////////////////////////////////////////////////////////////////////////////////////////
float sma_filter(float current_value, float* history_SMA)
{ 
  float sum=0;
  float average=0;
  int i;

  for(i=1;i<SMA_LENGTH;i++)
  {
    history_SMA[i-1]=history_SMA[i];
  }
  history_SMA[SMA_LENGTH-1]=current_value;
  
  for(i=0;i<SMA_LENGTH;i++)
  {
    sum+=history_SMA[i];
  }
  average=sum/SMA_LENGTH;

  return average;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//IMU
/////////////////////////////////////////////////////////////////////////////////////////////////
float tol = 10;
void pointNorth(String orientation, float millisLimit) {
  bool atNorth = false;
  float val = 0;
  float delta = 0;

  Serial.println("ForwardYaw: " + String(forwardYaw));
  
  while (atNorth == false && millis() < millisLimit) {
    updateIMUSensor();
    val = latestYaw; //get latest filtered yaw
    Serial.println("latestYaw: " + String(val));

    if (orientation == "forward") { //fixes any lateral tilt while in 'forward' mode
      delta = val - forwardYaw;
    } else { //reorient to 'north' of the pool
      delta = val - northYaw; //northYaw is a constant set-up at initial
    }
    
    if (delta < -180) {
      delta = delta + 360;
    } 
    else if (delta > 180) {
      delta = delta - 360;
    }
  
    if (-180 <= delta && delta <= -tol) {
      Left();
    } 
    else if (-tol <= delta && delta <= tol) {
      if (orientation != "forward") { 
        StopLat();
      }
      atNorth = true;
    }
    else if (tol <= delta && delta <= 180) {
      Right();
    }
    delay(10);
  }
  Forward();
}

void updateIMUSensor() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (imuSensor.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) 
  {
    imuSensor.readAccelData(imuSensor.accelCount);
    imuSensor.ax = (float)imuSensor.accelCount[0] * imuSensor.aRes; // - imuSensor.accelBias[0];
    sma_filter(imuSensor.ax, aXfilter);
    imuSensor.ay = (float)imuSensor.accelCount[1] * imuSensor.aRes; // - imuSensor.accelBias[1];
    sma_filter(imuSensor.ay, aYfilter);
    imuSensor.az = (float)imuSensor.accelCount[2] * imuSensor.aRes; // - imuSensor.accelBias[2];
    sma_filter(imuSensor.az, aZfilter);
    imuSensor.readGyroData(imuSensor.gyroCount);
    imuSensor.gx = (float)imuSensor.gyroCount[0] * imuSensor.gRes;
    imuSensor.gy = (float)imuSensor.gyroCount[1] * imuSensor.gRes;
    imuSensor.gz = (float)imuSensor.gyroCount[2] * imuSensor.gRes;
    imuSensor.readMagData(imuSensor.magCount);
    imuSensor.mx = (float)imuSensor.magCount[0] * imuSensor.mRes
               * imuSensor.factoryMagCalibration[0] - imuSensor.magBias[0];
    imuSensor.my = (float)imuSensor.magCount[1] * imuSensor.mRes
               * imuSensor.factoryMagCalibration[1] - imuSensor.magBias[1];
    imuSensor.mz = (float)imuSensor.magCount[2] * imuSensor.mRes
               * imuSensor.factoryMagCalibration[2] - imuSensor.magBias[2];
  } else {
    Serial.println("did not update");
  }
  imuSensor.updateTime();
  MahonyQuaternionUpdate(imuSensor.ax, imuSensor.ay, imuSensor.az, imuSensor.gx * DEG_TO_RAD,
                         imuSensor.gy * DEG_TO_RAD, imuSensor.gz * DEG_TO_RAD, imuSensor.my,
                         imuSensor.mx, imuSensor.mz, imuSensor.deltat);
  imuSensor.delt_t = millis() - imuSensor.count;
  if (imuSensor.delt_t > 50)
  {
    imuSensor.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
    imuSensor.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                  * *(getQ()+2)));
    imuSensor.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                  * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                  * *(getQ()+3));
    imuSensor.pitch *= RAD_TO_DEG;
    imuSensor.yaw   *= RAD_TO_DEG;
    imuSensor.yaw  -= 9.62; //for University of Waterloo
    imuSensor.roll *= RAD_TO_DEG;

    /*Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(imuSensor.yaw, 2);
    Serial.print(", ");
    Serial.print(imuSensor.pitch, 2);
    Serial.print(", ");
    Serial.println(imuSensor.roll, 2);*/

    latestYaw = sma_filter(imuSensor.yaw, yawfilter);
    float SMApitch = sma_filter(imuSensor.pitch, pitchfilter);
    float SMAroll = sma_filter(imuSensor.roll, rollfilter);

    /*Serial.print("SMAYaw, SMAPitch, SMARoll: ");
    Serial.print(latestYaw);
    Serial.print(", ");
    Serial.print(SMApitch);
    Serial.print(", ");
    Serial.println(SMAroll);*/

    imuSensor.count = millis();
    imuSensor.sumCount = 0;
    imuSensor.sum = 0;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//Code for 4 Motor Movements
/////////////////////////////////////////////////////////////////////////////////////////////////
int change = 85;
int handicap = 10;

void Forward() {
  Serial.println("Forward");
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change - handicap); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //right
}

void Backward() {
  Serial.println("Backward");
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change + handicap); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //right
}

void Left() {
  Serial.println("Left");
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change - 40); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change - 10 + handicap); //right
  //Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //left
  //Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //right
}

void Right() {
  Serial.println("Right");
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change + 10); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change + 40 + handicap); //right
  //Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed); //left
  //Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //right
}

void Up() {
  Serial.println("Up");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //z-axis right
}

void Down() {
  Serial.println("Down");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //z-axis right
}

void TiltRight() {
  Serial.println("Tilt Right");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed - change); //z-axis right
}

void TiltLeft() {
  Serial.println("Tilt Left");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed + change); //z-axis right
}

void StopLat() {
  //Serial.println("Stop Lateral");
  Motors[2].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[3].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
}

void StopZAxis() {
  //Serial.println("Stop Z-Axis");
  Motors[0].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
  Motors[1].Motor.writeMicroseconds(ESCSettings.MediumSpeed);
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Pressure Sensor
/////////////////////////////////////////////////////////////////////////////////////////////////
void updatePressureSensor() {
  pressure_abs = pressureSensor.getPressure(ADC_4096);
  //pressure_relative = sealevel(pressure_abs, base_altitude);
  latestPressureDepth = sma_filter(waterDepth(pressure_abs - pressure_baseline), pFilter);

  //Serial.print("Pressure abs (mbar)= ");
  //Serial.println(pressure_abs);

  //Serial.print("SMAPressure abs: ");
  //Serial.println(sma_filter(pressure_abs, pFilter));

  //Serial.print("Pressure relative (mbar)= ");
  //Serial.println(pressure_relative);

  //Serial.print("Water Depth (m) = ");
  //Serial.println(wDepth);
}

double waterDepth(double pRelative) {
  return(pRelative*100/(998*9.81));
}

/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Xbox Controller
/////////////////////////////////////////////////////////////////////////////////////////////////
void translateXbox() {

  /*if (Xbox.getButtonClick(B)) { //emergency stop
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
    else {
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
    else {
      StopZAxis();
    }
  }*/

  if (Xbox.getButtonClick(UP)) {
    Forward();
    delay(100);
    StopLat();
  }
  if (Xbox.getButtonClick(LEFT)) {
    Left();
    delay(100);
    StopLat();
  }
  if (Xbox.getButtonClick(DOWN)) {
    Backward();
    delay(100);
    StopLat();
  }
  if (Xbox.getButtonClick(RIGHT)) {
    Right();
    delay(100);
    StopLat();
  }

  if (Xbox.getButtonClick(A)) {
    Down();
    delay(100);
    StopZAxis();
  }
  if (Xbox.getButtonClick(B)) {
    TiltRight();
    delay(100);
    StopZAxis();
  }
  if (Xbox.getButtonClick(X)) {
    TiltLeft();
    delay(100);
    StopZAxis();
  }
  if (Xbox.getButtonClick(Y)) {
    Up();
    delay(100);
    StopZAxis();
  }

  if (Xbox.getButtonClick(L1)) {
    updateIMUSensor();
    northYaw = yawfilter[SMA_LENGTH-1];
    forwardYaw = northYaw;
    Serial.println("New North and Forward Yaw: " + String(northYaw));
  }
  if (Xbox.getButtonClick(R1)) {
    pointNorth("north", millis() + 10000); 
  }

  if (Xbox.getButtonClick(START)) {
    startAutonomous();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//Start Autonomous
/////////////////////////////////////////////////////////////////////////////////////////////////
//LATERAL SPEED: 60 inches/3.65s = 16.4383561644 inches/sec
//UP SPEED: 
float lateralSpeed = 16.4383561644;
float upSpeed = 14.5498;
float t = 0;
void startAutonomous() {
  //set yaw for north of pool and forward yaw
  updateIMUSensor();
  forwardYaw = latestYaw;
  northYaw = latestYaw;

  t = millis();
  Forward();
  //go under hanging obstacle for 72+18 inches.
  while(millis() - t < (72+18+20)/lateralSpeed*1000) {
    pointNorth("forward", t + (72+18+20)/lateralSpeed*1000);
  }
  
  t = millis();
  Up();
  updatePressureSensor();
  //go over table
  //140-30.5*2.54 //&& (latestPressureDepth - (140-30.5*2.54) + 3) > 0
  while(millis() - t < (22.5/upSpeed*1000) && (latestPressureDepth - (140-30.5*2.54) + 3) > 0) { 
    pointNorth("forward", t + (22.5/upSpeed*1000));
    updatePressureSensor();
  }

  //go through third obstacle
  t = millis();
  StopZAxis();
  while(millis() - t < (72+38+72-20+10)/lateralSpeed*1000-(22.5/upSpeed*1000)) {
    pointNorth("forward", t + (72+38+72-20+10)/lateralSpeed*1000-(22.5/upSpeed*1000));
  }

  StopLat();
  
  //go to landing pad
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Loop
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //XBOX
  Usb.Task();

  //Update Sensor Readings
  updateIMUSensor();

  if (irrecv.decode(&results)) { // have we received an IR signal?
    translateIR();
    irrecv.resume(); // receive the next value
  }
  if (Xbox.Xbox360Connected && !turnOffXbox) { //Have we received an Xbox signal?
    translateXbox();
    delay(100);
  } else {
    Serial.println("Xbox is disconnected");
    Serial.println("TurnOffXbox == True?: " + String(turnOffXbox == true));
  }
  delay(1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
