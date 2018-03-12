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
//SD Card
#include <SPI.h>
#include <SD.h>

/////////////////////////////////////////////////////////////////////////////////////////////////
//Initialization of variables
/////////////////////////////////////////////////////////////////////////////////////////////////
//IMU Sensor
MPU9250 imuSensor;
float mDirection;
float mX, mY, mZ;
float north;

//Pressure sensor
MS5803 pressureSensor(0x76);
//Create variables to store results
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
double base_altitude = 329.0; // Altitude of Waterloo Ontario (m)

//SD Card
const int sdChipSelect = 53;
File recordingFile;
File testFile;
bool isRecording = false;

//Test constants
bool testingMode = false;
int testDuration = 1;

//Filter Library
#define SMA_LENGTH 10
float yawfilter[SMA_LENGTH]; //magnometer X
float pitchfilter[SMA_LENGTH]; //magnometer Y
float rollfilter[SMA_LENGTH]; //magnometer Z

float aXfilter[SMA_LENGTH]; //A X
float aYfilter[SMA_LENGTH]; //A Y
float aZfilter[SMA_LENGTH]; //A Z

float pFilter[SMA_LENGTH]; //pressure

//IR Remote-Controller
int receiver = 8; //pin number for IR receiver
IRrecv irrecv(receiver);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'
void translateIR() // takes action based on IR code received
// describing Remote IR codes 
{
  String buttonPressed = "";
  
  switch(results.value)
  {
  case 0xFF6897: //0
    StopZAxis();   
    buttonPressed = "StopZAxis";
    break;
  case 0xFF30CF: //1
    if (testingMode == true) { 
      testForXDuration("Up"); 
    } else {
      Up();
    }
    buttonPressed = "Up";    
    break;
  case 0xFF18E7: //2
    if (testingMode == true) { 
      testForXDuration("Forward");
    } else {
      Forward();
    }
    buttonPressed = "Forward";    
    break;
  case 0xFF7A85: //3
    if (testingMode == true) { 
      testForXDuration("Down"); 
    } else {
      Down();
    }
    buttonPressed = "Down";
    break;
  case 0xFF10EF: //4
    Left();
    if (testingMode == true) { 
      testForXDuration("Left"); 
    } else {
      Left();
    }
    buttonPressed = "Left";   
    break;
  case 0xFF38C7: //5
    StopLat();
    buttonPressed = "StopLat";
    break;
  case 0xFF5AA5: //6
    if (testingMode == true) { 
      testForXDuration("Right"); 
    } else {
      Right();
    }
    buttonPressed = "Right";
    break;
  case 0xFF42BD: //7
    if (testingMode == true) { 
      testForXDuration("TiltLeft"); 
    } else {
      TiltLeft(); 
    }
    buttonPressed = "TiltLeft";
    break;
  case 0xFF4AB5: //8
    if (testingMode == true) { 
      testForXDuration("Backward"); 
    } else {
      Backward();
    }
    buttonPressed = "Backward";
    break;
  case 0xFF52AD: //9 
    if (testingMode == true) { 
      testForXDuration("TiltRight"); 
    } else {
      TiltRight();
    }
    buttonPressed = "TiltRight";
    break;
  ///////////////////////////
  case 0xFF9867: //EQ
    goToDepth(0.50); 
    buttonPressed = "GoToDepth";
    break;
  case 0xFFB04F: //ST/REPT
    pointNorth(); 
    buttonPressed = "pointNorth";
    break; 
  case 0xFF02FD: //play/pause
    if (isRecording == true) {
      buttonPressed = "STOP RECORDING";
      isRecording = false; 
      stopRecording();
    }
    else {
      buttonPressed = "START RECORDING";
      isRecording = true;
      startRecording();
    }
    break;
  case 0xFFE21D: //FUNC/STOP
    if (testingMode) {
      buttonPressed = "STOP TESTING MODE";
      testingMode = false;
      Serial.println("STOP TESTING MODE");
      testFile.close();
      buzzXTimes(3);
      
    } else {
      buttonPressed = "START TESTING MODE";
      testingMode = true;
      Serial.println("START TESTING MODE");
      testFile = SD.open("t" + String(millis()/1000) + ".txt", FILE_WRITE);
      if (testFile) {
        buzzXTimes(3);
        Serial.println("TEST FILE OPENED SUCCESSFULLY");
      }
      else {
        Serial.println("TEST FILE DID NOT OPEN SUCCESSFULLY");
        buzzXTimes(1);
      }
    }
    break;
  case 0xFF22DD: //FAST BACK
    testDuration = testDuration - 1;
    if (testDuration == 0) {
      testDuration = 5;
    }
    buttonPressed = "CHANGED TEST DURATION: " + String(testDuration);
    Serial.println("New Test Duration: " + String(testDuration));
    buzzXTimes(testDuration);
    break;
  case 0xFFC23D: //FAST FORWARD
    testDuration = (testDuration) % 6 + 1;
    buttonPressed = "CHANGED TEST DURATION: " + String(testDuration);
    Serial.println("New Test Duration: " + String(testDuration));
    buzzXTimes(testDuration);
    break;
  }// End Case

  if (isRecording == true) {
    recordingFile.println(buttonPressed);
    recordingFile.println(String(millis()));
  }
}

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
  int Low;
  int High;
  int Medium;
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
  //Serial Monitor @ 9600 bps
  Serial.begin(9600);

  pinMode(sdChipSelect, OUTPUT); //start the SD card
  if (!SD.begin(sdChipSelect)) {
    Serial.println("SD Card initialization failed!");
  }
  Serial.println("SD Card initialization done.");

  //pinMode(buzzerPin, OUTPUT); //initialize the buzzer
  pinMode(3, OUTPUT); //LED
  
  irrecv.enableIRIn(); // Start the receiver

  //Set up the motor pins
  Motors[0].Pin = 9; //z-axis left
  Motors[1].Pin = 10; //z-axis right
  Motors[2].Pin = 11; //left
  Motors[3].Pin = 12; //right
  for(int i = 0; i < NUMMOTORS; i++)
  {
    int pin = Motors[i].Pin;
    Motors[i].Motor.attach(pin);
  }

  //Set ESC settings
  ESCSettings.Low   = ESC_LOW_DEFAULT;
  ESCSettings.High  = ESC_HIGH_DEFAULT;
  ESCSettings.Medium = ESC_MED_DEFAULT;

  //Set motor speeds to 0
  for (int i = 0; i < NUMMOTORS; i++)
  {
    Motors[i].Motor.writeMicroseconds(ESCSettings.Medium);
  }

  //Initialize the pressure sensor
  pressureSensor.reset();
  pressureSensor.begin();
  pressure_baseline = pressureSensor.getPressure(ADC_4096);

  //Initialize IMU sensor
  Wire.begin(); //initialized the arduino board itself as a master
  Wire.begin(0x68); //imu
  Wire.begin(0x76); //pressure

  int intPin = 13;
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  
  //imuSensor.MPU9250SelfTest(imuSensor.selfTest);
  imuSensor.calibrateMPU9250(imuSensor.gyroBias, imuSensor.accelBias);
  imuSensor.initMPU9250();
  imuSensor.initAK8963(imuSensor.factoryMagCalibration);
  imuSensor.getAres();
  imuSensor.getGres();
  imuSensor.getMres();
  imuSensor.magBias[0] = 53.06;
  imuSensor.magBias[1] = 28.30;
  imuSensor.magBias[2] = -157.34;
  imuSensor.magScale[0] = 0.88;
  imuSensor.magScale[1] = 1.07;
  imuSensor.magScale[2] = 1.08;
  imuSensor.factoryMagCalibration[0] = 1.18;
  imuSensor.factoryMagCalibration[1] = 1.18;
  imuSensor.factoryMagCalibration[2] = 1.14;
  updateIMUSensor();
  north = imuSensor.yaw;
  Serial.println("North Calibrated To Be: " + String(north));
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//FILTER FUNCTION
/////////////////////////////////////////////////////////////////////////////////////////////////
float sma_filter(float current_value, float* history_SMA)
{ 
  float sum=0; /*This constrains SMA_LENGTH*/
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
float tol = 30;
void pointNorth() {
  bool atNorth = false;
  while (atNorth == false) {
    float val = yawfilter[SMA_LENGTH-1]; //get latest filtered yaw
    float delta = val - north;
    
    if (delta < -180) {
      delta = delta + 360;
    } 
    else if (delta > 180) {
      delta = delta - 360;
    }
  
    if (-180 <= delta && delta <= -tol) {
      Right();
    } 
    else if (-tol <= delta && delta <= 0) {
      StopLat();
      atNorth = true;
    } 
    else if (0 <= delta && delta <= tol) {
      StopLat();
      atNorth = true;
    }
    else if (tol <= delta && delta <= 180) {
      Left();
    }
  }
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
  }
  imuSensor.updateTime();
  MahonyQuaternionUpdate(imuSensor.ax, imuSensor.ay, imuSensor.az, imuSensor.gx * DEG_TO_RAD,
                         imuSensor.gy * DEG_TO_RAD, imuSensor.gz * DEG_TO_RAD, imuSensor.my,
                         imuSensor.mx, imuSensor.mz, imuSensor.deltat);
  imuSensor.delt_t = millis() - imuSensor.count;
  if (imuSensor.delt_t > 10)
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

    float SMAyaw = sma_filter(imuSensor.yaw, yawfilter);
    float SMApitch = sma_filter(imuSensor.pitch, pitchfilter);
    float SMAroll = sma_filter(imuSensor.roll, rollfilter);

    /*Serial.print("SMAYaw, SMAPitch, SMARoll: ");
    Serial.print(SMAyaw);
    Serial.print(", ");
    Serial.print(SMApitch);
    Serial.print(", ");
    Serial.println(SMAroll);*/

    if (isRecording == true) {
      recordingFile.println("SMAYaw, SMAPitch, SMARoll: " + String(SMAyaw) + " " + String(SMApitch) + " " + String(SMAroll));
      recordingFile.println("RawYaw, RawPitch, RawRoll: " + String(imuSensor.yaw) + " " + String(imuSensor.pitch) + " " + String(imuSensor.roll));
      recordingFile.println(String(millis()));
    }

    imuSensor.count = millis();
    imuSensor.sumCount = 0;
    imuSensor.sum = 0;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//Code for 4 Motor Movements
/////////////////////////////////////////////////////////////////////////////////////////////////
int change = 125;
int zAxisHandicap = 1;
int lateralLeftHandicap = 1;

void Forward() {
  Serial.println("Forward");
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium + change); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium + change + 5); //right
}

void Backward() {
  Serial.println("Backward");
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium - change - 5); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium - change); //right
}

void Left() {
  Serial.println("Left");
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium + change); //right
}

void Right() {
  Serial.println("Right");
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium + change); //left
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium); //right
}

void Up() {
  Serial.println("Up");
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium - change - 5); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium - change); //z-axis right
}

void Down() {
  Serial.println("Down");
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium + change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium + change + 5); //z-axis right
}

void TiltRight() {
  Serial.println("Tilt Right");
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium + change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium - change); //z-axis right
}

void TiltLeft() {
  Serial.println("Tilt Left");
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium - change); //z-axis left
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium + change); //z-axis right
}

void StopLat() {
  Serial.println("Stop Lateral");
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium);
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium);
}

void StopZAxis() {
  Serial.println("Stop Z-Axis");
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium);
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium);
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Pressure Sensor
/////////////////////////////////////////////////////////////////////////////////////////////////
void updatePressureSensor() {
  pressure_abs = pressureSensor.getPressure(ADC_4096);
  pressure_relative = sealevel(pressure_abs, base_altitude);
  float wDepth = sma_filter(waterDepth(pressure_abs - pressure_baseline), pFilter));

  /*Serial.print("Pressure abs (mbar)= ");
  Serial.println(pressure_abs);

  Serial.print("SMAPressure abs: ");
  Serial.println(sma_filter(pressure_abs, pFilter));

  Serial.print("Pressure relative (mbar)= ");
  Serial.println(pressure_relative);

  Serial.print("Water Depth (m) = ");
  Serial.println(wDepth);*/

  if (isRecording == true) {
    recordingFile.println("Water Depth (m) = " + String(wDepth));
    recordingFile.println(String(millis()));
  }
}

double waterDepth(double pRelative) {
  return(pRelative*100/(998*9.81));
}

/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//SD Card
/////////////////////////////////////////////////////////////////////////////////////////////////
void startRecording() {
  recordingFile = SD.open("r" + String(millis()/1000) + ".txt", FILE_WRITE);
  delay(100);
  if (recordingFile) {
    buzzXTimes(3);
    Serial.println("START RECORDING SUCCESS");
  }
  else {
    buzzXTimes(1);
    Serial.println("START RECORDING FAILURE");
  }
}

void stopRecording() {
  recordingFile.close();
  Serial.println("STOP RECORDING");
  
  buzzXTimes(3);
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Buzzer
/////////////////////////////////////////////////////////////////////////////////////////////////
void buzzXTimes(int times) {
  for(int i = 0; i < times; i++) {
    //digitalWrite(buzzerPin,HIGH);
    digitalWrite(3, HIGH);
    delay(100);
    //digitalWrite(buzzerPin,LOW);
    digitalWrite(3, LOW);
    delay(50);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//Testing Displacement
/////////////////////////////////////////////////////////////////////////////////////////////////
float ax, ay, az, sx, sy, sz, x, y, z, old_ax, old_ay, old_az, old_sx, old_sy, old_sz;

void testForXDuration(String direction) {
  testFile.println("TESTING DISPLACEMENT:" + direction + " AT TIME = " + String(millis()) + " msec FOR DURATION " + String(testDuration) + " sec");
  
  sx = 0;
  sy = 0;
  sz = 0;
  ax = aXfilter[SMA_LENGTH-1];
  ay = aYfilter[SMA_LENGTH-1];
  az = aZfilter[SMA_LENGTH-1];

  double startTime = millis();
  double lastTime = 0;

  switch(direction){
    case "Up":
      Up();
    break;
    case "Forward":
      Forward();
    break;
    case "Down":
      Down();
    break;
    case "Left":
      Left();
    break;
    case "Right":
      Right();
    break;
    case "TiltLeft":
      TiltLeft();
    break;
    case "TiltRight":
      TiltRight();
    break;
    case "Backward":
      Backward();
    break;
  }
  
  while(millis() - startTime < testDuration*1000) {
    old_ax = ax;
    old_ay = ay;
    old_az = az;
    old_sx = sx;
    old_sy = sy;
    old_sz = sz;

    updateIMUSensor();
    ax = aXfilter[SMA_LENGTH-1];
    ay = aYfilter[SMA_LENGTH-1];
    az = aZfilter[SMA_LENGTH-1];

    double delta_t = millis() - lastTime;
    lastTime = millis();
    
    sx = (old_ax+ax) * delta_t / 2.0;
    sy = (old_ay+ay) * delta_t / 2.0;
    sz = (old_az+az) * delta_t / 2.0;
    
    x = (old_sx+sx) * delta_t / 2.0;
    y = (old_sy+sy) * delta_t / 2.0;
    z = (old_sz+sz) * delta_t / 2.0;

    testFile.println("Time: " + String(lastTime) + " (x,y,z): (" + String(x) + ", " + String(y) + ", " + String(z) + ")");
  }
  testFile.println("FINISHED TESTING DISPLACEMENT");
  StopZAxis();
  StopLat();
}

/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//Loop
/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  updateIMUSensor();
  updatePressureSensor();
  
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    translateIR();
    irrecv.resume(); // receive the next value
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
