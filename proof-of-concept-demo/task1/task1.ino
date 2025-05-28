
int A1_MD = 5;       // Set the pin numbers for the h-bridge driver motor 
int A2_MD = 4;   
int B1_MD = 2;   
int B2_MD = 3;
int duty_cycle = 0;     // Set duty cycle value for PWM: 0 (always off), 255(always on)         


void setup() {
  pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
  pinMode(A2_MD, OUTPUT);
  pinMode(B1_MD, OUTPUT);
  pinMode(B2_MD, OUTPUT);
}

void loop() {
  duty_cycle = 
  digitalWrite(A1_MD, HIGH);
  analogWrite(A2_MD, duty_cycle);
  digitalWrite(B1_MD, HIGH);
  analogWrite(B2_MD, duty_cycle);
  
}