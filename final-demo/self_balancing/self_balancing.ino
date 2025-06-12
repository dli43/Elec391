#include "Arduino_BMI270_BMM150.h"
#include "math.h"

const float pi = 3.14159;
const float k = 0.4;
const float cutoff_angle = 30.0;
bool reference_angle_computed = false;
bool new_filtered_angle = false;
float ax,ay,az;
float gx,gy,gz;
float theta_a, theta_g, theta_k, theta_k_prev;
unsigned long prev_time, present_time;
float elapsed_time;
float integral = 0;

int A1_MD = 5;       // Set the pin numbers for the h-bridge driver motor 
int A2_MD = 4;   
int B1_MD = 2;   
int B2_MD = 3;
int duty_cycle = 0;

const float kp = 25;        // PID values
const float ki = 2;
//const float ki = 0.1;
const float kd = 0.15;
float pid_p;
float pid_i = 0;
float pid_d;
float dutycycle_drive;

void setup() {

  pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
  pinMode(A2_MD, OUTPUT);
  pinMode(B1_MD, OUTPUT);
  pinMode(B2_MD, OUTPUT);

  if (!IMU.begin()) {     // Check if IMU module has initialized
    while (1);
  }

  Serial.begin(115200);

}

void loop() {


  if(IMU.accelerationAvailable()){
    IMU.readAcceleration(ax, ay, az);       // Read acceleration values into variables
    theta_a = (180/pi)*atan(ay/az);         // Compute new acceleration theta
    
    if(!reference_angle_computed){          // Assign reference values for gyroscope and filtered theta
      prev_time = micros();                 // Mark time when acceleration is sampled
      theta_g = theta_a;
      theta_k = theta_a;
      theta_k_prev = theta_k;
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
      theta_k = k*(theta_k_prev + (-gx)*elapsed_time) + (1-k)*theta_a;     // Compute theta_k
      new_filtered_angle = true;
    }

  }

  if(new_filtered_angle){

    //calculate pwm value based on absolute value of tilt angle
    pid_d = kd*(theta_k-theta_k_prev)/elapsed_time;
    pid_p = kp*theta_k;
    integral += theta_k*elapsed_time;
    pid_i = constrain(ki*(integral), -255, 255);

    dutycycle_drive = int(pid_d+pid_i+pid_p);
    duty_cycle = min(abs(dutycycle_drive), 255);


    if(theta_k > cutoff_angle){
      analogWrite(A1_MD, 255);
      digitalWrite(A2_MD, LOW);
      analogWrite(B1_MD, 255);
      digitalWrite(B2_MD, LOW);
    }

    else if(theta_k < -cutoff_angle){
      digitalWrite(A1_MD, LOW);
      analogWrite(A2_MD, 255);
      digitalWrite(B1_MD, LOW);
      analogWrite(B2_MD, 255);
    }
    //tilted forward
    else if(dutycycle_drive > 0){
      analogWrite(A1_MD, duty_cycle);
      digitalWrite(A2_MD, LOW);
      analogWrite(B1_MD, duty_cycle);
      digitalWrite(B2_MD, LOW);
    }
    //tilted backward
    else if(dutycycle_drive < 0){
      digitalWrite(A1_MD, LOW);
      analogWrite(A2_MD, duty_cycle);
      digitalWrite(B1_MD, LOW);
      analogWrite(B2_MD, duty_cycle);
    }
    //no tilt
    else{
      digitalWrite(A1_MD, LOW);
      digitalWrite(A2_MD, LOW);
      digitalWrite(B1_MD, LOW);
      digitalWrite(B2_MD, LOW);
    }

    new_filtered_angle = false;

    theta_k_prev = theta_k;
  }
}