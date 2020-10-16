//Libraries
#include <Wire.h>  //Wire library used for I2C
#include <SPI.h>  //SPI library used by SD-card reader and IMU library
#include "SparkFunMPL3115A2.h" //Altimeter Library
#include <SparkFunLSM9DS1.h>  //IMU library
#include <SD.h>  //SD card library
#include <Servo.h> //library for controlling a servo

//Pin definitions
#define SDchipSelect BUILTIN_SDCARD   //SD Pin Definition
#define PRINT_CALCULATED
#define PRINT_SPEED 250   // 250 ms between prints 
#define tempLED 24       //LED pin Definitions  
#define magLED 25
#define servoPin 2  //pin for the servo
#define feedbackPin 23  //pin for the feedback line

//Objects
LSM9DS1 LSM;  //States "imu" is an LSM9DS1 IMU class object
MPL3115A2 MPL; //States "MPL" is an MPL3115A2 Altimeter class object
Servo myservo;  // create servo object to control a servo

//Variables
String magData;  //string that will be set to hold all of the IMU magnetometer data
float magX; // magnetometer values
float magY;
float magZ;
float tempF;  //tempature in fahrenheit
float alt;  //alttitude in ft (based off of pressure) 
float baseTemp = 73; //baseline tempature
float dTemp = 7; //amount the tempature needs to change from the baseline the LED to go on

//Control variables
double pos = 90;    // variable to store the servo position
bool magnet = false;  //tells the servo loop if the magnetometer pick up a strong magnetic signal in its last reading
int increment = 2;        // increment to move for each interval
int  ServoUpdateInterval = 20;      // interval between servo updates. Modifying this along with increment allows you to adjust the speed. Fine tune these to avoid choppy motion.
unsigned long lastServoUpdate; // last update of position on servo
int  SenUpdateInterval = 500;      // interval between sensor updates
unsigned long lastSenUpdate; // last update of sensors

// Servo calibration variables
int minPos = 10;  //position on the end of the servos range for calibration
int maxPos = 140;  //position on the end of the servos range for calibration
int minFeedback;  //feedback value at the minimum position
int maxFeedback;  //feedback value at the maximum position
int tolerance = 10; // max feedback measurement error. Keep this as low as possible without causing the servo to freeze becouse it can't reach the desired tolerance.

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

  //Servo setup and calibration
  myservo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
  // Move to the minimum position and record the feedback value
  myservo.write(minPos);
  delay(2000); // make sure it has time to get there and settle
  minFeedback = analogRead(feedbackPin);
  
  // Move to the maximum position and record the feedback value
  myservo.write(maxPos);
  delay(2000); // make sure it has time to get there and settle
  maxFeedback = analogRead(feedbackPin);
  myservo.write(pos);
    
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
  delay(500);  //waits 500 ms
}


void loop() {
  //Servo loop
  if((millis() - lastServoUpdate) > ServoUpdateInterval){
    lastServoUpdate = millis();
    if(magnet==true){
      pos = 90;
      myservo.write(pos);
      Serial.println(pos);
      // Start the move...
      // Calculate the target feedback value for the final position
      int target = map(pos, minPos, maxPos, minFeedback, maxFeedback); 
      // Wait until it reaches the target
      while(abs(analogRead(feedbackPin) - target) > tolerance){
        Serial.print("Position:  ");
        Serial.println(pos);
        Serial.print("Target:  ");
        Serial.println(target);
        Serial.print("Feedback:  ");
        Serial.println(analogRead(feedbackPin));
        Serial.print("Feedback position:  ");
        Serial.println(map(analogRead(feedbackPin), minFeedback, maxFeedback, minPos, maxPos)); //print the position according to the feeback line
        Serial.println(" ");
        } // wait...
      //Serial.println(map(analogRead(feedbackPin), minFeedback, maxFeedback, minPos, maxPos)); //print the position according to the feeback line
    } 
    else{
      pos += increment;
      // Start the move...
      myservo.write(pos);
      Serial.println(pos);
      // Calculate the target feedback value for the final position
      int target = map(pos, minPos, maxPos, minFeedback, maxFeedback); 
      // Wait until it reaches the target
      while(abs(analogRead(feedbackPin) - target) > tolerance){} // wait...
      if ((pos >= 130) || (pos <= 40)) // end of sweep
      {
        // reverse direction
        increment = -increment;
      }
    }
  }

  //Sensor and data loggin loop
  if((millis() - lastSenUpdate) > SenUpdateInterval){
    lastSenUpdate = millis();

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
      magnet=true;
    }
    else
    {
      digitalWrite(magLED, LOW);
      magnet=false;
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

    //Creates the string "dataLOG" and includes the device measurements listed, which is then printed to the serial monitor and the SD-card.
    String dataLOG = String(millis()) + ", " + magData + ", " + String(tempF) + ", " + String(alt);
      if(SDactive) {
        datalog = SD.open(filename, FILE_WRITE);
        datalog.println(dataLOG);  //Prints the "dataLOG" to the SD-card
        datalog.close();
      }
    Serial.println(dataLOG);  //prints to the serial monitor so you can see the incoming data  
  }  
}
