#include "Arduino_BMI270_BMM150.h"

float ax,ay,az;

void setup() {

  Serial.begin(9600);   // Initialize baud rate
  while (!Serial);      // Wait for serial to start
  Serial.println("Started");

  if (!IMU.begin()) {     // Check if IMU module has initialized
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

}

void loop() {

  // Reads the acceleration values when it is ready to be read
  if(IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    
  }

}