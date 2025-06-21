#include "math.h"

#define samples 20

float kernel_test[samples];
float signal_test[samples];
int index_pnt = 3;

void setup() {
  // put your setup code here, to run once:
  while (!Serial);
  delay(1000);
  Serial.begin(115200);

  float sum = 0;
  for(int i = 0; i < samples; i++){
    kernel_test[i] = exp(-4*i/(float)(samples-1));
    sum += kernel_test[i];
  }

  Serial.println("The values of the Kernel are: ");
  for(int i = 0; i < samples; i++){
    kernel_test[i] /= sum;
    Serial.print(kernel_test[i], 6);
    Serial.print(" ");
  }
  Serial.print("\n");

  Serial.println("The values of the signal are: ");
  for(int i = 0; i < samples; i++){
    signal_test[(samples + (index_pnt - i)) % samples] = samples - i;
  }
  for(int i = 0; i < samples; i++){
    Serial.print(signal_test[i]);
    Serial.print(" ");
  }
  Serial.print("\n");

  float final_value = 0;
  Serial.println("The dotted values are : ");
  for(int i = 0; i < samples; i++){
    float product = kernel_test[i]*signal_test[(samples + (index_pnt - i)) % samples];
    Serial.print(product);
    Serial.print(" ");
    final_value += product;
  }
  Serial.print("\n");

  Serial.println("The final value is ");
  Serial.println(final_value);

}

void loop() {
  // put your main code here, to run repeatedly:

}
