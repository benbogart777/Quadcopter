#include <Wire.h>  //Wire library used by IMU and Dallas temp sensors
#include <SPI.h>  //SPI library used by SD-card reader
#include "SparkFunMPL3115A2.h" //Altimeter Library
#include <SparkFunLSM9DS1.h>  //IMU library
#include <SD.h>  //SD card library


//Pin definitions
#define SDchipSelect BUILTIN_SDCARD   //SD Pin Definition
#define PRINT_CALCULATED
#define PRINT_SPEED 250   // 250 ms between prints 
#define tempLED 24       //LED pin Definitions  
#define magLED 25

//Objects
LSM9DS1 LSM;  //States "imu" is an LSM9DS1 IMU class object
MPL3115A2 MPL; //States "MPL" is an MPL3115A2 Altimeter class object

//Variables
String magData;  //string that will be set to hold all of the IMU magnetometer data
float magX; // magnetometer values
float magY;
float magZ;
float tempF;  //tempature in fahrenheit
float alt;  //alttitude in ft (based off of pressure) 
float baseTemp = 70; //baseline tempature
float dTemp = 8; //amount the tempature needs to change from the baseline the LED to go on

//For SD card
File datalog;                      //File Object for datalogging
char filename[] = "DroneB00.csv";   //Template for file name to save data in SD-card
bool SDactive = false;             //Used to check for SD card before attempting to log data


void setup() { 
    Serial.begin(9600);  //begins serial communications that can be used for troubleshooting at a baud rate of 9600.
    Wire.begin(); //initiates wire library for I2C
    //LED pin outputs
    pinMode(tempLED, OUTPUT); 
    pinMode(magLED, OUTPUT);
    
    //SD Card Setup 
    pinMode(10, OUTPUT);                                  
    pinMode(SDchipSelect, OUTPUT);
    Serial.print("Initializing SD card...");
    if (!SD.begin(SDchipSelect)){                              //Attempt to start SD communication
      Serial.println("Card not inserted or the card is broken.");          //Print out error if failed; remind user to check card
    }
    else {                                                    //If successful, attempt to create file
      Serial.println("Card initialized successfully.\nCreating File...");
      for (byte i = 0; i < 100; i++) {                        //Can create up to 100 files with similar names, but numbered differently
        filename[6] = '0' + i / 10;
        filename[7] = '0' + i % 10;
        if (!SD.exists(filename)) {                           //If a given filename doesn't exist, it's available
          datalog = SD.open(filename, FILE_WRITE);            //Create file with that name
          SDactive = true;                                    //Activate SD logging since file creation was successful
          Serial.println("Logging to: " + String(filename));  //Tell user which file contains the data for this run of the program
          break;                                              //Exit the for loop now that we have a file
        }
      }
      if (!SDactive) {
      Serial.println("No available file names; clear SD card to enable logging");
      } 
    }
  
  //IMU possible start up error message
  if (LSM.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");  //Prints error message on startup if the IMU is not wired correctly.
  }
  

  //MPL set up
  MPL.begin();
  MPL.setModeAltimeter(); // puts the sensor in altimeter mode
  MPL.setOversampleRate(7); // Set Oversample to the recommended 128
  MPL.enableEventFlags(); // Enable all three pressure and temp event flags

  delay(500);
  //Prints header everytime on startup
  String headerLOG = "Time(ms), Mag(x), Mag(y), Mag(z), Temp(F), Alt(ft)";  //Defines "headerLOG" as a string that contains the inscribed text.
  Serial.println(headerLOG);  //Prints the "headerLOG" to the serial monitor
  if(SDactive) {
    datalog.println(headerLOG);  //Prints the "headerLOG" to the SD-card
    datalog.close();  
  }
  Serial.println(headerLOG);
  delay(500);  //waits 500 ms
}


void loop() {
  //IMU loop (gets the new data every loop and redefines the variables and then puts them in the string "magData"
  if (LSM.magAvailable() ){
    LSM.readMag();
  }
  magX = LSM.calcMag(LSM.mx);
  magY = LSM.calcMag(LSM.my);
  magZ = LSM.calcMag(LSM.mz);
  float magVec = sqrt(magX*magX+magY*magY+magZ*magZ);
  
  //Turns on a LED if there is a strong enough magnetic signal
  if(2 < magVec)
  {
    digitalWrite(magLED, HIGH);
  }
  else
  {
    digitalWrite(magLED, LOW);
  }
  magData = String(magX) + ", " + String(magY) + ", " + String(magZ);

  //Altimeter loop (gets the new data every loop and redefines the variables)
  tempF = MPL.readTempF();
  alt = MPL.readAltitudeFt();

  //Turns on a LED if there is diffrent from a baseline temp
  if(baseTemp-tempF > dTemp || baseTemp-tempF < -dTemp)
  {
    digitalWrite(tempLED, HIGH);
  }
  else
  {
    digitalWrite(tempLED, LOW);
  }

  //Creates the string "dataLOG" and includes the device measurements listed, which is then printed to the serial monitor, the xBee monitor, and the SD-card.
  String dataLOG = String(millis()) + ", " + magData + ", " + String(tempF) + ", " + String(alt);
    if(SDactive) {
      datalog = SD.open(filename, FILE_WRITE);
      datalog.println(dataLOG);  //Prints the "dataLOG" to the SD-card
      datalog.close();
    }
  Serial.println(dataLOG);  //prints to the serial monitor so you can see the incoming data
  delay(500);  //waits .5 sec before re-running the loop  
}
