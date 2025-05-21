#include "Arduino_BMI270_BMM150.h"

float ax,ay,az;
float gx,gy,gz;
char userInput;

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

  if(Serial.available() > 0){

    userInput = Serial.read();
    
    // Reads the acceleration values when it is ready to be read and is requested
    if(IMU.accelerationAvailable() && userInput == 'a') {
      IMU.readAcceleration(ax, ay, az);
      Serial.print(ax);
      Serial.print(",");
      Serial.println(ay);
    }
    
    
    if(IMU.gyroscopeAvailable() && userInput == 'g'){
      IMU.readGyroscope(gz, gy, gz);
      Serial.println(gz);
    }

  }

}