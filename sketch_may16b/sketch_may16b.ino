#include "Arduino_BMI270_BMM150.h"

int sampling_period;
float theta, k;

void setup() {

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");
  
  IMU.begin();

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in G's");
  Serial.println("X\tY\tZ");

  theta = 0; //measured angle, initialize to zero

  //sampling period in ms/sample
  sampling_period = 1000/IMU.accelerationSampleRate();

  //weight for complementary filter
  k = 0.99; //still needs tuning
}

void loop() {
  // put your main code here, to run repeatedly:

  //measure accelerometer readings
  float ax,ay,az;
  float gx, gy, gz;
  if(IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);

    //for debugging
    Serial.print("ax = ");
    Serial.println(ax);
    Serial.print("ay = ");
    Serial.println(ay);
    Serial.print("az = ");
    Serial.println(az);

    //accelerometer angle
    float theta_a = atan(ay/ax)*180/3.14;
    Serial.print("accelerometer angle: ");
    Serial.println(theta_a);

    IMU.readGyroscope(gx, gy, gz);

    Serial.print("gx = ");
    Serial.println(gx);
    Serial.print("gy = ");
    Serial.println(gy);
    Serial.print("gz = ");
    Serial.println(gz);
    // calculate gyroscope angle with previous angle measurement
    float theta_g = theta + gz*sampling_period/1000;
    Serial.print("gyroscope angle: ");
    Serial.println(theta_g);

    //complementary filter
    theta = k*theta_g + (1-k)*(theta_a);
    Serial.print("filtered angle: ");
    Serial.println(theta);
  }

}
