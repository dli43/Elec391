#include "Arduino_BMI270_BMM150.h"
#define Samples 15000.0;

float gx,gy,gz;
float sum, average;
int counter;

void setup() {

  Serial.begin(115200);   // Initialize baud rate
  while (!Serial);      // Wait for serial to start
  Serial.println("Started");

  if (!IMU.begin()) {     // Check if IMU module has initialized
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

}

void loop() {

  sum = 0;
  counter = 0;
  while(counter < Samples){
    
    if(IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(gx, gy, gz);
      sum = sum + gx;
      counter++;
    }
    
  }

  average = sum/Samples;
  Serial.print("The bias of the gyroscope is: ");
  Serial.println(average,4);
  while(1);

}