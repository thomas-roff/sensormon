/*
_    _______ ______     _     _____  ______  ______   _____  ______  
   | |  (_______)  ___ \   | |   / ___ \(_____ \|  ___ \ / ___ \|  ___ \ 
    \ \  _____  | |   | |   \ \ | |   | |_____) ) | _ | | |   | | |   | |
     \ \|  ___) | |   | |    \ \| |   | (_____ (| || || | |   | | |   | |
 _____) ) |_____| |   | |_____) ) |___| |     | | || || | |___| | |   | |
(______/|_______)_|   |_(______/ \_____/      |_|_||_||_|\_____/|_|   |_|
                                                                         
 AccordionMasterSensor
 For installation in an accordion to receive pressure, distance and orientation
 data from the bellows
*/

#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect

// Pressure Sensor
  const int MPU_addr=0x68;  // I2C address of the MPU-6050
// Gyro Accelerometer
  int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
// Ultrasonic Sensor
  // US Transmitter (both hard synced)
  const int pingPin = 12;
  // US Receiver
  const int rtnPin = 11;
  // US Values
  long duration;
  int maximumRange = 1200; // The max distance observed from the sensor
  int minimumRange = 200; //  The min distance observed from the sensor

/*
int reedSwitch[] = {1, 2, 7, 8}; // Create an array of reedSwitches
int reedOn[] = {1, 1, 1, 1}; //
int reedOff[] = {0, 0, 0, 0}; //
*/

#include <Wire.h>
#include "Adafruit_MPRLS.h"
//#include "SR04.h"
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

void setup()
{

 // Gyro wake-up
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

 //Serial
  Serial.begin(57600); //starts the serial communication via USB

 // Reed switch array
/*
 for (int index = 0; index < 4; index++) {
   pinMode(reedSwitch[index], INPUT);
   digitalWrite(reedSwitch[index], HIGH);
 }
*/

 // Pressure sensor test
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
       delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");

}

void loop()
{

// DATA COLLECTION

// Pressure Sensor
float pressure_hPa = mpr.readPressure();

// Ultrasonic Sensor
  // US Pins
  pinMode(pingPin, OUTPUT);
  pinMode(rtnPin, INPUT);
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  duration = pulseIn(rtnPin, HIGH);

// Gyro Accelerometer
Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


// DATA SEND

// Accelerometer
// Order 1: 2: 3:
Serial.print(AcX); Serial.print(" "); 
Serial.print(AcY); Serial.print(" "); 
Serial.print(AcZ); Serial.print(" ");
// Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
// Gyro
// Order 1: 2: 3:
Serial.print(GyX); Serial.print(" "); 
Serial.print(GyY); Serial.print(" "); 
Serial.print(GyZ); Serial.print(" ");

// Distance (with error handling)
  if (duration >= maximumRange || duration <= minimumRange) {
    Serial.print("-1"); Serial.print(" "); //if nothing observed ouput -1
    //digitalWrite(LEDPin, HIGH); // LED will be on
  } else {
    Serial.print(duration); Serial.print(" "); // prints the distance in CM
    //digitalWrite(LEDPin, LOW); // LED will be off if object detected
  }
  
// Pressure
Serial.print(pressure_hPa); Serial.print(" ");

// Reed Switches
// Order 1:Bottom 2:Front 3:Back 4:Top
/*
 for (int index = 0; index < 4; index++)
 {
   int val = digitalRead(reedSwitch[index]); // read input value
   if (val == LOW) // check if the switch is pressed
   {
     Serial.print(reedOn[index]); Serial.print(" ");
   }
   else {
     Serial.print(reedOff[index]); Serial.print(" ");
   }
 }
Serial.println();
*/

delay(2); // good practice not to overload the serial port

}
