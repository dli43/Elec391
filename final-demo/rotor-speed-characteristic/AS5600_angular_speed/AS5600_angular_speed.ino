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

int as5600_select = 6;    // Select the address pin for the i2c multiplexer


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

  pinMode(as5600_select, OUTPUT);   // Set the i2c multiplexer select pin as output

  delay(1000);
}


void loop()
{
  duty_cycle = 191;     // 25% opposite wheel turn direction
  digitalWrite(A1_MD, HIGH);
  analogWrite(A2_MD, duty_cycle);
  analogWrite(B1_MD, duty_cycle);
  digitalWrite(B2_MD, HIGH);

  Serial.println(as5600.getAngularSpeed(AS5600_MODE_RPM));
  delay(50);
}


//  -- END OF FILE --
