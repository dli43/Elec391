#include "Arduino_BMI270_BMM150.h"
#include "math.h"

const float pi = 3.14159;
float ax, ay, az;
float theta_rad ,theta_deg;
char userInput;

void setup() {

  Serial.begin(9600);   // Initialize baud rate
  while (!Serial);      // Wait for serial to start
  Serial.print("Started\n");

  if (!IMU.begin()) {     // Check if IMU module has initialized
    Serial.print("Failed to initialize IMU!\n");
    while (1);
  }

}

void loop() {

  if(Serial.available() > 0){         // User input detected

    userInput = Serial.read();
    
    if(IMU.accelerationAvailable() && userInput == 'g') {   // User request

      IMU.readAcceleration(ax, ay, az);       // Read acceleration values into variables
      theta_rad = atan(ay/az);                // Compute theta
      theta_deg = (180/pi)*theta_rad;         // Convert to degrees
      Serial.print(theta_deg);
      Serial.print("\n");
      
    }

  }

}