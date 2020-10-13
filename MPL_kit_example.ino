//Libraries
#include <Wire.h>  //Wire library allows for I2C communication
#include "SparkFunMPL3115A2.h" //Altimeter Library

//Pin definitions
#define tempLED 24        

//Objects
MPL3115A2 MPL; //States "MPL" is an MPL3115A2 Altimeter class object

//Variables
float tempF;  //tempature in fahrenheit
float alt;  //alttitude in ft (based off of pressure) 
float baseTemp = 70; //baseline tempature
float dTemp = 8; //amount the tempature needs to change from the baseline the LED to go on

void setup() { 
  Serial.begin(9600);  //begins serial communications that can be used for troubleshooting at a baud rate of 9600.
  Wire.begin(); //initiates wire library for I2C
  //LED pin outputs
  pinMode(tempLED, OUTPUT); 
    
  //MPL set up
  MPL.begin();
  MPL.setModeAltimeter(); // puts the sensor in altimeter mode
  MPL.setOversampleRate(7); // Set Oversample to the recommended 128
  MPL.enableEventFlags(); // Enable all three pressure and temp event flags
  delay(500);

  //Prints header everytime on startup
  String headerLOG = "Time(ms), Temp(F), Alt(ft)";  //Defines "headerLOG" as a string that contains the inscribed text.
  Serial.println(headerLOG);
  delay(500);  //waits 500 ms
}


void loop() {
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

  //Creates the string "dataLOG" and includes the device measurements listed, which is then printed to the serial monitor.
  String dataLOG = String(millis()) + "," + String(tempF) + ", " + String(alt);
  Serial.println(dataLOG);  //prints to the serial monitor so you can see the incoming data
  delay(500);  //waits .5 sec before re-running the loop  
}
