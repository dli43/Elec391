#include "Arduino_BMI270_BMM150.h"

int A1_MD = 5;       // Set the pin numbers for the h-bridge driver motor 
int A2_MD = 4;   
int B1_MD = 2;   
int B2_MD = 3;
int duty_cycle = 195;     // Set duty cycle value for PWM: 0 (always on), 255(always off)         


void setup() {
  pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
  pinMode(A2_MD, OUTPUT);
  pinMode(B1_MD, OUTPUT);
  pinMode(B2_MD, OUTPUT);

  if(!IMU.begin()){
    while(1);
  }
}

void loop() {

    //25% power for 5s
    duty_cycle = 191;
    analogWrite(A1_MD, duty_cycle);
    digitalWrite(A2_MD, HIGH);
    digitalWrite(B1_MD, HIGH);
    analogWrite(B2_MD, duty_cycle);
    delay(5000);

    //75% power for 5s
    duty_cycle = 63;
    analogWrite(A1_MD, duty_cycle);
    digitalWrite(A2_MD, HIGH);
    digitalWrite(B1_MD, HIGH);
    analogWrite(B2_MD, duty_cycle);
    delay(5000);
}