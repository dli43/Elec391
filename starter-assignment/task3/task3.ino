#include "Arduino_BMI270_BMM150.h"
#include "math.h"

const float pi = 3.14159;
bool reference_angle_computed = false;
float ax,ay,az;
float gx,gy,gz;
float theta;
unsigned long prev_time, present_time;
float elapsed_time;
char userInput;

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

 if(Serial.available() > 0){   // User input detected

    userInput = Serial.read();
    
    if(IMU.accelerationAvailable() && userInput == 'g' && !reference_angle_computed) {    // First theta sample request
      IMU.readAcceleration(ax,ay,az);
      prev_time = micros();             // Mark time when acceleration is sampled
      theta = (180/pi)*atan(ay/az);
      Serial.print(theta);
      Serial.print("\n");
      reference_angle_computed = true;  // Reference angle is computed so that integration can be used       
    }
    
    else if(IMU.gyroscopeAvailable() && userInput == 'g' && reference_angle_computed){    // Gyroscope theta sample request
      IMU.readGyroscope(gx, gy, gz);
      present_time = micros();          
      elapsed_time = (float)(present_time - prev_time)/1e6;       // Calculate time that has passed since last sample read
      prev_time = present_time;
      theta = theta + (-gx)*elapsed_time;                          // Note the minus sign, this is to denote the orientation of the velocity
      Serial.print(theta);
      Serial.print("\n");
    }

  }

}