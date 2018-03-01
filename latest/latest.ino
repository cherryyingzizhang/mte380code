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

/////////////////////////////////////////////////////////////////////////////////////////////////
//Initialization of variables
/////////////////////////////////////////////////////////////////////////////////////////////////
//IMU Sensor
MPU9250 imuSensor;
float mDirection;
float mX, mY, mZ;

//Pressure sensor
#define TOLERANCE 0.01 //1 cm Tolerance
MS5803 pressureSensor(0x76);
//Create variables to store results
double pressure_abs, altitude_delta, pressure_baseline;
double base_altitude = 325.0; // Altitude of Waterloo Ontario (m)


//Filter Library
#define SMA_LENGTH 5
float yawfilter[SMA_LENGTH]; //magnometer X
float pitchfilter[SMA_LENGTH]; //magnometer Y
float rollfilter[SMA_LENGTH]; //magnometer Z
float pFilter[SMA_LENGTH]; //pressure


//IR Remote-Controller
int receiver = 8; //pin number for IR receiver
IRrecv irrecv(receiver);     // create instance of 'irrecv'
decode_results results;      // create instance of 'decode_results'
void translateIR() // takes action based on IR code received
// describing Remote IR codes 
{
  switch(results.value)
  {
  case 0xFF6897: StopZAxis();   Serial.println("0");  break;//0
  case 0xFF30CF: Up();   Serial.println("1");    break;//1
  case 0xFF18E7: Forward();   Serial.println("2");    break;//2
  case 0xFF7A85: Down();   Serial.println("3");    break;//3
  case 0xFF10EF: Left();   Serial.println("4");    break;//4
  case 0xFF38C7: StopLat();   Serial.println("5");    break;//5
  case 0xFF5AA5: Right();   Serial.println("6");    break;//6
  case 0xFF42BD: TiltLeft();   Serial.println("7");    break;//7
  case 0xFF4AB5: Backward();   Serial.println("8");    break;//8
  case 0xFF52AD: TiltRight();   Serial.println("9");    break;//9
  case 0xFF9867: goToDepth(2); Serial.println("EQ"); break;//EQ
  case 0xFFB04F: pointNorth(); Serial.println("ST/REPT"); break;
  }// End Case
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
  // put your setup code here, to run once:
  Serial.println("Setup: Serial port communication at 9600bps");
  Serial.begin(9600);
  
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
  //pressure_baseline = pressureSensor.getPressure(ADC_4096);

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
  imuSensor.magBias[0] = -1.77;
  imuSensor.magBias[1] = 265.32;
  imuSensor.magBias[2] = 222.33;
  imuSensor.magScale[0] = 0.85;
  imuSensor.magScale[1] = 0.89;
  imuSensor.magScale[2] = 1.43;
  imuSensor.factoryMagCalibration[0] = 1.18;
  imuSensor.factoryMagCalibration[1] = 1.18;
  imuSensor.factoryMagCalibration[2] = 1.14;

  //Set up pressure offset via filtering first for 5 sec.
  for(int i = 0; i < 5; i++) {
    pressure_baseline = sma_filter(pressureSensor.getPressure(ADC_4096), pFilter);

    if (i != 4) {
      delay(200);
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//FILTER FUNCTION
/////////////////////////////////////////////////////////////////////////////////////////////////
int sma_filter(int current_value, float* history_SMA)
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
void pointNorth() {
  int yaw = sma_filter(imuSensor.yaw, yawfilter);
  if (yaw > -115) {
    Left();
  }
  else {
    Right();
  }
  delay(200);
  StopLat();
}
/////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////
//Code for 4 Motor Movements
/////////////////////////////////////////////////////////////////////////////////////////////////
int change = 75;
int zAxisHandicap = 1;
int lateralLeftHandicap = 1;

void Forward() {
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium + change + lateralLeftHandicap); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium - change); //right
}

void Backward() {
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium - change - lateralLeftHandicap); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium + change); //right
}

void Left() {
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium - change - lateralLeftHandicap); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium - change); //right
}

void Right() {
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium + change + lateralLeftHandicap); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium + change); //right
}

void Up() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium - change + 10); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium + change + zAxisHandicap - 10); //z-axis right
}

void Down() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium + change - 10); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium - change - zAxisHandicap + 10); //z-axis right
}

void TiltRight() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium + change - 10); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium + change + zAxisHandicap - 10); //z-axis right
}

void TiltLeft() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium - change + 10); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium - change - zAxisHandicap + 10); //z-axis right
}

void StopLat() {
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium);
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium);
}

void StopZAxis() {
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium);
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium);
}
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Pressure Sensor
/////////////////////////////////////////////////////////////////////////////////////////////////
 double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return(P/pow(1-(A/44330.0),5.255));
}

double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return(44330.0*(1-pow(P/P0,1/5.255)));
}

void goToDepth(float WANTED_DEPTH) {
  pressure_abs = sma_filter(pressureSensor.getPressure(ADC_4096), pFilter);
  altitude_delta = altitude(pressure_abs, pressure_baseline);
  if (altitude_delta - WANTED_DEPTH > 0) {
    Up();
  }
  else {
    Down();
  }
  delay(200);
  StopZAxis();
}

//TODO: not finished this function yet. don't use it.
float currentDepth;
void maintainDepth() {
  pressure_abs = sma_filter(pressureSensor.getPressure(ADC_4096), pFilter);
  altitude_delta = altitude(pressure_abs, pressure_baseline);
  if (altitude_delta - currentDepth > 0) {
    Up();
  }
  else {
    Down();
  }
  delay(200);
  StopZAxis(); //Turn vertical thrust motors off
}

/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
//Loop
/////////////////////////////////////////////////////////////////////////////////////////////////
unsigned long refreshInterval = 500; // ms
unsigned long lastRefreshTime = 0;

void loop() {
  //Every X ms this code will run. Change refreshInterval to change X.
  /*if(millis() - lastRefreshTime >= refreshInterval)
  {
    lastRefreshTime += refreshInterval;
    //do blah
  }*/

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (imuSensor.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) 
  {
    imuSensor.readAccelData(imuSensor.accelCount);
    imuSensor.ax = (float)imuSensor.accelCount[0] * imuSensor.aRes; // - imuSensor.accelBias[0];
    imuSensor.ay = (float)imuSensor.accelCount[1] * imuSensor.aRes; // - imuSensor.accelBias[1];
    imuSensor.az = (float)imuSensor.accelCount[2] * imuSensor.aRes; // - imuSensor.accelBias[2];
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
  if (imuSensor.delt_t > 500)
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

    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(imuSensor.yaw, 2);
    Serial.print(", ");
    Serial.print(imuSensor.pitch, 2);
    Serial.print(", ");
    Serial.println(imuSensor.roll, 2);

    Serial.print("SMAYaw, SMAPitch, SMARoll: ");
    Serial.print(sma_filter(imuSensor.yaw, yawfilter));
    Serial.print(", ");
    Serial.print(sma_filter(imuSensor.pitch, pitchfilter));
    Serial.print(", ");
    Serial.println(sma_filter(imuSensor.roll, rollfilter));

    Serial.print("Pressure Raw: ");
    Serial.println(pressureSensor.getPressure(ADC_4096));

    Serial.print("SMAPressure: ");
    Serial.println(sma_filter(pressureSensor.getPressure(ADC_4096), pFilter));

    imuSensor.count = millis();
    imuSensor.sumCount = 0;
    imuSensor.sum = 0;
  }
  
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    translateIR();
    delay(500); 
    irrecv.resume(); // receive the next value
  } 
  //else {
    //maintainDepth();
  //}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
