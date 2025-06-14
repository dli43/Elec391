int A1_MD = 5;       // Set the pin numbers for the h-bridge driver motor 
int A2_MD = 4;   
int B1_MD = 2;   
int B2_MD = 3;
int duty_cycle = 0;      // Set duty cycle value for PWM: 0 (always on), 255(always off)
char buffer[3];         


void setup() {
  pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
  pinMode(A2_MD, OUTPUT);
  pinMode(B1_MD, OUTPUT);
  pinMode(B2_MD, OUTPUT);
  digitalWrite(A2_MD,LOW);
  digitalWrite(B2_MD,LOW);
  digitalWrite(A1_MD,LOW);
  digitalWrite(B1_MD,LOW);
}

void loop() {                       // Run motors in forwards mode using pwm control


  if (Serial.available() > 0) {
    int receivedValue = Serial.parseInt();
    Serial.print("You entered: ");
    Serial.println(receivedValue);
    duty_cycle = receivedValue;
    digitalWrite(A1_MD,LOW);
    digitalWrite(B1_MD,LOW);
    analogWrite(A2_MD,duty_cycle);
    analogWrite(B2_MD,duty_cycle);
    }

  delay(500);  

}