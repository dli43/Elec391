  #include "Arduino_BMI270_BMM150.h"
  #include "math.h"
  #include "AS5600.h"
  #include <ArduinoBLE.h>
  #include <Arduino_HS_PWM.h>


  const float pi = 3.14159;
  const float k = 0.995;
  const int pwm_deadzone = 44;
  const float gyro_bias = -1.0428;
  bool reference_angle_computed = false;
  bool new_accelerometer_angle = false;
  bool new_gyro_angle = false;

  const char* BLE_name = "TEAM1-BLE-ROBOT";
  const char* service_UUID = "00000000-5EC4-4083-81CD-A10B8D5CF6EC";
  const char* characteristic_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC";
  BLEDevice central;
  bool isConnected = false;
  const int BUFFER_SIZE = 20;
  BLEService customService(service_UUID);
  BLECharacteristic customCharacteristic(characteristic_UUID, BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

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
  
  const int pwmPin = 5;
  HSPWM pwm(pwmPin);

  void setup() {

    pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
    pinMode(A2_MD, OUTPUT);
    pinMode(B1_MD, OUTPUT);
    pinMode(B2_MD, OUTPUT);

    if (!IMU.begin()) {     // Check if IMU module has initialized
      while (1);
    }
    Wire.begin();
    
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    initialize_BLE();

    pwm.begin(10000); // Set to 10 kHz frequency
    pwm.analogWrite(128); // 50% duty cycle      
    //analogWrite(pin, value) with pwm.analogWrite(value) wherever you use PWM in your motor control logic.

  }

  void loop() {

    if(IMU.accelerationAvailable()){
      read_accelerometer(); //update theta_a
      new_accelerometer_angle = true;
    }

    if(IMU.gyroscopeAvailable()){
      IMU.readGyroscope(gx,gy,gz);            // Read angular velocity values into variables 
      gx -= gyro_bias;
      find_elapsed_time();
      new_gyro_angle = true;
    }

    //once both sensors have measured something
    if(new_accelerometer_angle && new_gyro_angle){
      theta_k = k*(theta_k_prev+(-gx)*elapsed_time) + (1-k)*theta_a;
      angle_count += 1;
      calculate_pwm();
      drive_wheels();
      //send info once every 10 angle measurements
      if(angle_count == 10){
        //print_info();
        angle_count = 0;
      }
    }

    //read input data 1 char at a time
    if(Serial.available() > 0){
      input_char = (char)Serial.read();

      //if data fully sent
      if(input_char == '\n'){
        update_pid();
      }

      //if data still sending
      else{
        input_str += input_char;
      }
    }

    //run BLE logic
    runBLE();

    //set previous theta_k value for next loop
    theta_k_prev = theta_k;
    new_gyro_angle = false;
    new_accelerometer_angle = false;
  }

  //reads accelerometer values and stores angle in theta_a
  void read_accelerometer(){
    IMU.readAcceleration(ax, ay, az);       // Read acceleration values into variables
    theta_a = (180/pi)*atan(ay/az);         // Compute new acceleration theta
    
    if(!reference_angle_computed){          // Assign reference values for gyroscope and filtered theta
      prev_time = micros();                 // Mark time when acceleration is sampled
      reference_angle_computed = true;
    }
  }

  // Calculate time that has passed since last sample read
  void find_elapsed_time(){
    present_time = micros();          
    elapsed_time = (float)(present_time - prev_time)/1e6;
    prev_time = present_time;
  }

  void calculate_pwm(){
    //calculate pwm value based on absolute value of tilt angle
    pid_d = kd*(theta_k-theta_k_prev)/elapsed_time;
    pid_p = kp*theta_k;
    integral += theta_k*elapsed_time;
    pid_i = constrain(ki*(integral), -255, 255);

    dutycycle_drive = int(pid_d+pid_i+pid_p);
    duty_cycle = min(abs(dutycycle_drive), 255);
    duty_cycle = max(duty_cycle, pwm_deadzone);
  }

  void drive_wheels(){
    //drive forward
    if(dutycycle_drive > 0){
      analogWrite(A1_MD, duty_cycle);
      digitalWrite(A2_MD, LOW);
      analogWrite(B1_MD, duty_cycle);
      digitalWrite(B2_MD, LOW);
    }
    //drive backward
    else if(dutycycle_drive < 0){
      digitalWrite(A1_MD, LOW);
      analogWrite(A2_MD, duty_cycle);
      digitalWrite(B1_MD, LOW);
      analogWrite(B2_MD, duty_cycle);
    }
    //no drive
    else{
      digitalWrite(A1_MD, LOW);
      digitalWrite(A2_MD, LOW);
      digitalWrite(B1_MD, LOW);
      digitalWrite(B2_MD, LOW);
    }
  }

  void print_info(){
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
  }

  void update_pid(){
    
    int kpIndex = input_str.indexOf('')

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
    //if input was bad
    else{
      Serial.print("Bad Input vlaues");
    }

    //clear input string after reading
    input_str = "";

  
  }

  void initialize_BLE(){
    
    if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1); //jail
    }

    // Set the device name and local name
    BLE.setLocalName(BLE_name);
    BLE.setDeviceName(BLE_name);

    // Add the characteristic to the service
    customService.addCharacteristic(customCharacteristic);

    // Add the service
    BLE.addService(customService);

    // Set an initial value for the characteristic
    customCharacteristic.writeValue("Waiting for data");

    // Start advertising the service
    BLE.advertise();

    Serial.println("Bluetooth® device active, waiting for connections...");
  }


  //BLE communication logic
  void runBLE(){
    // Check for new central connections
    BLEDevice newCentral = BLE.central();

    // Handle new connection
    if (newCentral && !isConnected) {
      central = newCentral;
      isConnected = true;
      Serial.print("Connected to central: ");
      Serial.println(central.address());
      digitalWrite(LED_BUILTIN, HIGH);
    }

    // Handle disconnection
    if (isConnected && !central.connected()) {
      isConnected = false;
      Serial.println("Disconnected from central.");
      digitalWrite(LED_BUILTIN, LOW);
    }

    // If connected, check for incoming data
    if (isConnected && customCharacteristic.written()) {
      int length = customCharacteristic.valueLength();
      const unsigned char* receivedData = customCharacteristic.value();

      char receivedString[length + 1];
      memcpy(receivedString, receivedData, length);
      receivedString[length] = '\0';

      Serial.print("Received data: ");
      Serial.println(receivedString);

      customCharacteristic.writeValue("Data received");
    }

  }