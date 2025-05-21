#include "Arduino_BMI270_BMM150.h"

int sampling_period;
float theta, k;
unsigned long prevTime;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  theta = 0;  // Initial filtered angle
  sampling_period = 1000 / IMU.accelerationSampleRate();
  k = 0.99;  // Complementary filter weight
  prevTime = millis();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Time delta (in seconds)
    unsigned long currTime = millis();
    float dt = (currTime - prevTime) / 1000.0;
    prevTime = currTime;

    // Calculate accelerometer angle in degrees (X-Z plane example)
    float theta_a = atan2(ay, ax) * 180.0 / PI;

    // Gyroscope angle integration (Z-axis)
    float theta_g = theta + gz * dt;

    // Complementary filter
    theta = k * theta_g + (1 - k) * theta_a;

    // Output in CSV format: ax, ay, az, filtered angle
    Serial.print(ax, 4); Serial.print(",");
    Serial.print(ay, 4); Serial.print(",");
    Serial.print(az, 4); Serial.print(",");
    Serial.println(theta, 2);
  }

  delay(10); // Slight delay for stability
}
