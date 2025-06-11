//
//    FILE: AS5600_angular_speed.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo
//     URL: https://github.com/RobTillaart/AS5600
//
//  Examples may use AS5600 or AS5600L devices.
//  Check if your sensor matches the one used in the example.
//  Optionally adjust the code.


#include "AS5600.h"


//  Uncomment the line according to your sensor type
//  AS5600L as5600;   //  use default Wire
AS5600 as5600;   //  use default Wire

int A1_MD = 5;       // Set the pin numbers for the h-bridge driver motor 
int A2_MD = 4;   
int B1_MD = 2;   
int B2_MD = 3;
int duty_cycle;      // Set duty cycle value for PWM: 0 (always on), 255(always off)

float rpm_buffer[40];
float rpm_values[511];
float sum;

bool flag = false;


void setup()
{
  while(!Serial);

  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);
  Serial.println();

  Wire.begin();

  Serial.println(as5600.getAddress());

  // as5600.setAddress(0x40);  //  AS5600L only

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
  pinMode(A2_MD, OUTPUT);
  pinMode(B1_MD, OUTPUT);
  pinMode(B2_MD, OUTPUT);

}


void loop()
{
  for(int i = -255; i <= 255; i++){    // Start with maximum reverse speed and end with maximum forward speed

    if(i < 0){    // Reverse rotation
      duty_cycle = 255 + i;
      analogWrite(A1_MD, duty_cycle);
      digitalWrite(A2_MD, HIGH);
      analogWrite(B1_MD, duty_cycle);
      digitalWrite(B2_MD, HIGH);
    }
    else{       // Forward Rotation
      duty_cycle = 255 - i;
      digitalWrite(A1_MD, HIGH);
      analogWrite(A2_MD, duty_cycle);
      digitalWrite(B1_MD, HIGH);
      analogWrite(B2_MD, duty_cycle);
    }

    if(!flag){    // Flag for minimizing bias on first measurement
      delay(5000);
      flag = true;
    }

    sum = 0;          // Average multiple angular speed measurements for each duty cycle
    for(int k = 0; k < 40; k++){
      rpm_buffer[k] = as5600.getAngularSpeed(AS5600_MODE_RPM);
      delay(25);
      sum = sum + rpm_buffer[k];
    }

    int index = i + 255;  // since i goes from -255 to 255
    if (index >= 0 && index < 511) {
      rpm_values[index] = sum / 40;
      Serial.println(rpm_values[index]);
    }


  }

  Serial.println("DONE");   // Finish
  digitalWrite(A1_MD, HIGH);
  digitalWrite(A2_MD, duty_cycle);
  digitalWrite(B1_MD, HIGH);
  digitalWrite(B2_MD, duty_cycle);
  while(1);
  


}


//  -- END OF FILE --
