#include "Arduino_BMI270_BMM150.h"

float ax,ay,az;
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
    if(IMU.accelerationAvailable() && userInput == 'g') {
      IMU.readAcceleration(ax, ay, az);

      //Serial.print(ax);
      //Serial.print('\t');
      //Serial.print(ay);
      //Serial.print('\t');
      Serial.print(az);
      Serial.print("\n")

    }

  }

}