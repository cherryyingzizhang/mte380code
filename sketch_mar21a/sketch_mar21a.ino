#include <SPI.h>
#include <SD.h>

#define pin_MOSI 51
#define pin_MISO 50
#define chipSelectPin 49

void setup() {
  // put your setup code here, to run once:
  pinMode(chipSelectPin, OUTPUT); 

  Serial.begin(9600);
  SPI.begin();
  if (!SD.begin(7))
  {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }
  Serial.println("SUCCESS - SD card initialized.");
}

void loop() {
  // put your main code here, to run repeatedly:

}
