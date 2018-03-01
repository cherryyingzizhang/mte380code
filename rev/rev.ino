#include <Servo.h>

int CurrentSpeed;
int Step = 10;

#define NUMMOTORS 4
typedef struct MotorDef
{
    Servo   Motor; 
    int     Pin;   // Indicates the Pin this motor is connected to
};
MotorDef Motors[NUMMOTORS];

// Stores the settings for all ESC. This can be made specific to each ESC, but that's not needed
// for a quadcopter project
typedef struct ESCSettingsDef
{
  int Low;
  int High;
  int Medium;
};
ESCSettingsDef ESCSettings; 

#define ESC_HIGH_DEFAULT 1860
#define ESC_LOW_DEFAULT 1060
#define ESC_MED_DEFAULT 1460

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Setup: Serial port communication at 9600bps");
  Motors[0].Pin = 9; //z-axis left
  Motors[1].Pin = 10; //z-axis right
  Motors[2].Pin = 11; //left
  Motors[3].Pin = 12; //right

  for(int i = 0; i < NUMMOTORS; i++)
  {
    int pin = Motors[i].Pin;
    Motors[i].Motor.attach(pin);
  }

  ESCSettings.Low   = ESC_LOW_DEFAULT;
  ESCSettings.High  = ESC_HIGH_DEFAULT;
  ESCSettings.Medium = ESC_MED_DEFAULT;
}

void SetThrottleRange()
{
  Serial.println("In Set Throttle Range mode");
    
  for (int i = 0; i < NUMMOTORS; i++)
  {
    Motors[i].Motor.writeMicroseconds(ESCSettings.High);
  }

  Serial.println("Connect the ESC now. After connecting the ESC, you should hear the ESC startup tones. Shortly afterwards, you should hear two beeps indicating that the ESC has registered the high throttle value. Immediately after hearing the two beeps, push any key. If you don't do so in 5 sec, the ESC will go into program mode");

  // Wait for user input
  while (!Serial.available())
  {
  }
  Serial.read();

  Serial.println("\nSetting the low throttle setting. If this happens successfully, you should hear several beeps indicating the input voltage supplied to the ESC followed by a long beep indicating that the low throttle has been set. After this point, push any key to proceed, your ESC is ready to be used");

  for (int i = 0; i < NUMMOTORS; i++)
  {
    Motors[i].Motor.writeMicroseconds(ESCSettings.Low);
  }

  // Wait for user input
  while (!Serial.available())
  {
  }
  Serial.read();

  for (int i = 0; i < NUMMOTORS; i++)
  {
    Motors[i].Motor.writeMicroseconds(ESCSettings.Medium);
  }

  // Wait for user input
  while (!Serial.available())
  {
  }
  Serial.read();
}

int change = 50;
int zAxisRightHandicap = 1;
int lateralLeftHandicap = 1;

void Forward() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium); //z-axis right
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium + change + lateralLeftHandicap); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium - change); //right
}

void Backward() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium); //z-axis right
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium - change - lateralLeftHandicap); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium + change); //right
}

void Left() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium); //z-axis right
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium - change - lateralLeftHandicap); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium - change); //right
}

void Right() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium); //z-axis right
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium + change + lateralLeftHandicap); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium + change); //right
}

void Up() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium - change); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium + change + zAxisRightHandicap); //z-axis right
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium); //right
}

void Down() {
  //stop
  Motors[0].Motor.writeMicroseconds(ESCSettings.Medium + change); //z-axis left
  //stop
  Motors[1].Motor.writeMicroseconds(ESCSettings.Medium - change - zAxisRightHandicap); //z-axis right
  //forward
  Motors[2].Motor.writeMicroseconds(ESCSettings.Medium); //left
  //forward
  Motors[3].Motor.writeMicroseconds(ESCSettings.Medium); //right
}

void Stop() {
  for (int i = 0; i < NUMMOTORS; i++)
  {
      Motors[i].Motor.writeMicroseconds(ESCSettings.Medium);
  }
}

void RunAllBySerialInput() {
  for (int i = 0; i < NUMMOTORS; i++)
  {
    Motors[i].Motor.writeMicroseconds(ESCSettings.Medium);
  }
  Serial.println("Running ESC");
  Serial.println("Step = ");
  Serial.println(Step);
  Serial.println("\nPress 'u' to increase speed, 'd' to reduce speed, 's' to stop all motors.");

  CurrentSpeed = ESCSettings.Medium;
  while (1) {
    while (!Serial.available())
    {
    }
    char currentChar = Serial.read();
    if (currentChar == 'u')
    {
      Serial.println("\nIncreasing motor speed by step");
      if (CurrentSpeed + Step < ESCSettings.High) {
        CurrentSpeed = CurrentSpeed + Step;
        Serial.println("New speed = ");
        Serial.println(CurrentSpeed);
      }

      else
      {
        Serial.println("\nMax speed reached\n");
      }
    }
    if (currentChar == 'd')
    {
      Serial.println("\nDecreasing motor speed by step\n");
      if (CurrentSpeed - Step >= ESCSettings.Low)
      {
        CurrentSpeed = CurrentSpeed - Step;
        Serial.println("New speed = ");
        Serial.println(CurrentSpeed);
      }

      else
      {
        Serial.println("\nMin speed reached\n");
      }
    }
    if (currentChar == 's')
    {
      Serial.println("\nStopping Motors\n");
      CurrentSpeed = ESCSettings.Medium;
    }
    for (int i = 0; i < NUMMOTORS; i++)
    {
      Motors[i].Motor.writeMicroseconds(CurrentSpeed);
    }
  }
}

void RunDirectionsBySerialInput() {
  for (int i = 0; i < NUMMOTORS; i++)
  {
    Motors[i].Motor.writeMicroseconds(ESCSettings.Medium);
  }
  Serial.println("Running ESC");
  Serial.println("Step = ");
  Serial.println(Step);
  Serial.println("\nPress w, a, s, or d for lateral direction. Press q to go up, e to go down. Press x to stop.");

  CurrentSpeed = ESCSettings.Medium;
  while (1) {

    //Read key when available else just loop/keep motor running in current direction.
    while (!Serial.available())
    {
    }
    char currentChar = Serial.read();
    
    if (currentChar == 'w')
    {
      Serial.println("\nGoing forward");
      Forward();
    }
    if (currentChar == 'a')
    {
      Serial.println("\nGoing left");
      Left();
    }
    if (currentChar == 's')
    {
      Serial.println("\nGoing backward");
      Backward();
    }
    if (currentChar == 'd')
    {
      Serial.println("\nGoing right");
      Right();
    }
    if (currentChar == 'q')
    {
      Serial.println("\nGoing up");
      Up();
    }
    if (currentChar == 'e')
    {
      Serial.println("\nGoing down");
      Down();
    }
    if (currentChar == 'x')
    {
      Serial.println("\nStopping Motors\n");
      Stop();
    }
  }
}

void loop() {
  //SetThrottleRange();
  //RunDirectionsBySerialInput();
  RunDirectionsBySerialInput();
}
