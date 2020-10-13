#include <Wire.h>  //Wire library used by IMU and Dallas temp sensors
#include <SPI.h>  //SPI library used by SD-card reader
#include <SparkFunLSM9DS1.h>  //IMU library


//Pin definitions 
#define magLED 25

//Objects
LSM9DS1 LSM;  //States "imu" is an LSM9DS1 IMU class object

//Variables
String magData;  //string that will be set to hold all of the IMU magnetometer data
float magX; // magnetometer values
float magY;
float magZ;

void setup() { 
  Serial.begin(9600);  //begins serial communications that can be used for troubleshooting at a baud rate of 9600.
  Wire.begin(); //initiates wire library for I2C
  //LED pin outputs 
  pinMode(magLED, OUTPUT);
    
  //IMU possible start up error message
  if (LSM.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");  //Prints error message on startup if the IMU is not wired correctly.
  }
  delay(500);
  
  //Prints header everytime on startup
  String headerLOG = "Time(ms), Mag(x), Mag(y), Mag(z)";  //Defines "headerLOG" as a string that contains the inscribed text.
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

  //Creates the string "dataLOG" and includes the device measurements listed, which is then printed to the serial monitor, the xBee monitor, and the SD-card.
  String dataLOG = String(millis()) + ", " + magData;
  Serial.println(dataLOG);  //prints to the serial monitor so you can see the incoming data
  delay(500);  //waits .5 sec before re-running the loop  
}
