#include "Arduino_BMI270_BMM150.h"
#include "math.h"

const float pi = 3.14159;
const float k = 0.5;
const float cutoff_angle = 30.0;
bool reference_angle_computed = false;
bool new_filtered_angle = false;
float ax,ay,az;
float gx,gy,gz;
float theta_a, theta_g, theta_k;
unsigned long prev_time, present_time;
float elapsed_time;
char userInput;

int A1_MD = 5;       // Set the pin numbers for the h-bridge driver motor 
int A2_MD = 4;   
int B1_MD = 2;   
int B2_MD = 3;
int duty_cycle = 0;

void setup() {

  pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
  pinMode(A2_MD, OUTPUT);
  pinMode(B1_MD, OUTPUT);
  pinMode(B2_MD, OUTPUT);

  if (!IMU.begin()) {     // Check if IMU module has initialized
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
      new_filtered_angle = true;
    }

  }

  if(new_filtered_angle){

    //calculate pwm value based on absolute value of tilt angle
    duty_cycle = 255 - min(int(255*abs(theta_k)/cutoff_angle), 255);
    
    //tilted forward
    if(theta_k > 0){
      digitalWrite(A1_MD, HIGH);
      analogWrite(A2_MD, duty_cycle);
      digitalWrite(B1_MD, HIGH);
      analogWrite(B2_MD, duty_cycle);
    }
    //tilted backward
    else if(theta_k < 0){
      analogWrite(A1_MD, duty_cycle);
      digitalWrite(A2_MD, LOW);
      analogWrite(B1_MD, duty_cycle);
      digitalWrite(B2_MD, HIGH);
    }
    //no tilt
    else{
      digitalWrite(A1_MD, HIGH);
      digitalWrite(A2_MD, HIGH);
      digitalWrite(B1_MD, HIGH);
      digitalWrite(B2_MD, HIGH);
    }
  }
}