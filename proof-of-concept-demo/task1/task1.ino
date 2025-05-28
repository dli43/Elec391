
int A1_MD = 5;       // Set the pin numbers for the h-bridge driver motor 
int A2_MD = 4;   
int B1_MD = 2;   
int B2_MD = 3;
int duty_cycle;      // Set duty cycle value for PWM: 0 (always on), 255(always off)         


void setup() {
  pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
  pinMode(A2_MD, OUTPUT);
  pinMode(B1_MD, OUTPUT);
  pinMode(B2_MD, OUTPUT);
}

void loop() {                       // Run motors in forwards mode using pwm control

  duty_cycle = 191;                 // 25% speed
  digitalWrite(A1_MD, HIGH);
  digitalWrite(B1_MD, HIGH);
  analogWrite(A2_MD, duty_cycle);
  analogWrite(B2_MD, duty_cycle);
  delay(5000);

  duty_cycle = 127;                 // 50% speed
  analogWrite(A2_MD, duty_cycle);
  analogWrite(B2_MD, duty_cycle);
  delay(5000);

  duty_cycle = 63;                  // 75% speed
  analogWrite(A2_MD, duty_cycle);
  analogWrite(B2_MD, duty_cycle);
  delay(5000);

  duty_cycle = 0;                   // 100% speed
  analogWrite(A2_MD, duty_cycle);
  analogWrite(B2_MD, duty_cycle);
  delay(5000);

}