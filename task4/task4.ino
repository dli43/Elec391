#include "Arduino_BMI270_BMM150.h"
#include "math.h"

const float pi = 3.14159;
const float k = 0.5;
bool reference_angle_computed = false;
float ax,ay,az;
float gx,gy,gz;
float theta_a, theta_g, theta_k;
unsigned long prev_time, present_time;
float elapsed_time;
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

  if(IMU.accelerationAvailable()){
    IMU.readAcceleration(ax, ay, az);       // Read acceleration values into variables
    theta_a = (180/pi)*atan(ay/az);         // Compute new acceleration theta
    
    if(!reference_angle_computed){          // Assign reference values for gyroscope and filtered theta
      prev_time = micros();                 // Mark time when acceleration is sampled
      theta_g = theta_a;
      theta_k = theta_a;
      reference_angle_computed = true;
    }
    
  }

  if(IMU.gyroscopeAvailable()){
    IMU.readGyroscope(gx,gy,gz);            // Read angular velocity values into variables 
    
    if(reference_angle_computed){
      present_time = micros();          
      elapsed_time = (float)(present_time - prev_time)/1e6;           // Calculate time that has passed since last sample read
      prev_time = present_time;
      theta_g = theta_g + (-gx)*elapsed_time;                         // Compute theta_g
      theta_k = k*(theta_k + (-gx)*elapsed_time) + (1-k)*theta_a;     // Compute theta_k
    }

  }


 if(Serial.available() > 0){              // User input detected

    userInput = Serial.read();

    if(userInput == 'g'){                     // Request all thetas
      Serial.print(theta_a);
      Serial.print(",");
      Serial.print(theta_g);
      Serial.print(",");
      Serial.print(theta_k);
      Serial.print("\n");
    }  
    
  }

}