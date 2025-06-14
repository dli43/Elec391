#include "Arduino_BMI270_BMM150.h"
#include "math.h"
#include "AS5600.h"

const float pi = 3.14159;
const float k = 0.995;
const int pwm_deadzone = 44;
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
int angle_count = 0;

float kp = 0;        // PID values
float ki = 0;
float kd = 0;
float pid_p;
float pid_i = 0;
float pid_d;
float dutycycle_drive;
static String input_str = "";
char input_char = ' ';
AS5600 as5600; 

void setup() {

  pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
  pinMode(A2_MD, OUTPUT);
  pinMode(B1_MD, OUTPUT);
  pinMode(B2_MD, OUTPUT);

  if (!IMU.begin()) {     // Check if IMU module has initialized
    while (1);
  }
  // Wire.begin();
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
    gx += 0.4352;
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
    angle_count += 1;
    //calculate pwm value based on absolute value of tilt angle
    pid_d = kd*(theta_k-theta_k_prev)/elapsed_time;
    pid_p = kp*theta_k;
    integral += theta_k*elapsed_time;
    pid_i = constrain(ki*(integral), -255, 255);

    dutycycle_drive = int(pid_d+pid_i+pid_p);
    duty_cycle = min(abs(dutycycle_drive), 255);
    duty_cycle = max(duty_cycle, pwm_deadzone);

    //tilted forward
    if(dutycycle_drive == 0){
      digitalWrite(A1_MD, LOW);
      digitalWrite(A2_MD, LOW);
      digitalWrite(B1_MD, LOW);
      digitalWrite(B2_MD, LOW);
    }
    if(dutycycle_drive > 0){
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
    if(angle_count == 10){
      Serial.print("Theta: ");
      Serial.print(theta_k);
      Serial.print(" | dutycylcle_drive: ");
      Serial.print(dutycycle_drive);
      Serial.print(" | PID: ");
      Serial.print(duty_cycle);
      Serial.print(" | P: ");
      Serial.print(pid_p); 
      Serial.print(" | I: ");
      Serial.print(pid_i);
      Serial.print(" | D: ");
      Serial.print(pid_d);
      Serial.print("\n");
      angle_count = 0;
    }
  }

  //reads input 1 char at a time
  if(Serial.available() > 0){
    input_char = (char)Serial.read();

    //if data fully sent
    if(input_char == '\n'){
        int comma1 = input_str.indexOf(',');
        int comma2 = input_str.indexOf(',', comma1+1);

        //if data formatted correctly
        if(comma1 > 0 && comma2 > comma1){
          kp = input_str.substring(0, comma1).toFloat();
          ki = input_str.substring(comma1+1, comma2).toFloat();
          kd = input_str.substring(comma2+1).toFloat();

          //print new values
          Serial.print("kp: "); Serial.print(kp);
          Serial.print(" ki: "); Serial.print(ki);
          Serial.print(" kd: "); Serial.println(kd);
        }
        else{
          Serial.print("Bad Input vlaues");
        }
        //clear input string after reading
        input_str = "";
    }

    //if data still sending
    else{
      input_str += input_char;
    }
  }
}