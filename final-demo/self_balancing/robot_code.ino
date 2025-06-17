    #include "Arduino_BMI270_BMM150.h"
    #include "math.h"
    #include "AS5600.h"
    #include <ArduinoBLE.h>

    const float pi = 3.14159;
    const float k = 0.993;
    const int pwm_deadzone = 46;
    float standing_angle = 0;
    const float gyro_bias = 0.0807;
    float integral_constraint = 0.5;
    bool reference_angle_computed = false;
    bool new_gyro_angle = false;
    float encoder_velocity = 0;
    float desired_velocity = 0;
    float tau = 0.5; //time to correct velocity error
    float desired_acceleration = 0;
    float desired_angle = 0;

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

    float kp = 1.5;        // PID values
    float ki = 30;
    float kd = 0.28;
    float pid_p;
    float pid_i = 0;
    float pid_d;
    int dutycycle_drive;
    static String input_str = "";
    char input_char = ' ';
    AS5600 as5600; 
    

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

    }

    void loop() {

      if(IMU.accelerationAvailable()){
        read_accelerometer(); //update theta_a
      }

      if(IMU.gyroscopeAvailable()){
        IMU.readGyroscope(gx,gy,gz);            // Read angular velocity values into variables 
        gx -= gyro_bias;
        find_elapsed_time();
        new_gyro_angle = true;
      }

      //once both sensors have measured something
      if(new_gyro_angle){
        theta_k = k*(theta_k_prev+(-gx)*elapsed_time) + (1-k)*theta_a;
        angle_count += 1;
        measure_velocity(); //use encoders for this

        desired_acceleration = (desired_velocity-encoder_velocity)/tau;
        desired_angle = atan(desired_acceleration/9.807);

        calculate_pwm();
        drive_wheels();
        //send info once every 30 angle measurements
        if(angle_count == 30){
          print_info();
          angle_count = 0;
        }
        new_gyro_angle = false;
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
      //find correction angle
      float correction_angle = theta_k - standing_angle - desired_angle;

      
      integral += correction_angle*elapsed_time;
      integral = constrain(integral, -integral_constraint, integral_constraint);

      //calculate pwm value based on absolute value of tilt angle
      pid_p = kp*correction_angle;
      pid_i = ki*(integral);
      pid_d = kd*(theta_k-theta_k_prev)/elapsed_time;

      dutycycle_drive = int(pid_d+pid_i+pid_p)+0.5;
      duty_cycle = min(abs(dutycycle_drive)+pwm_deadzone, 255);
    }

    void drive_wheels(){

      //drive forward
      if(dutycycle_drive > 0){
        analogWrite(A1_MD, duty_cycle);
        analogWrite(A2_MD, 0);
        analogWrite(B1_MD, duty_cycle);
        analogWrite(B2_MD, 0);
      }
      //drive backward
      else if(dutycycle_drive < 0){
        analogWrite(A1_MD, 0);
        analogWrite(A2_MD, duty_cycle);
        analogWrite(B1_MD, 0);
        analogWrite(B2_MD, duty_cycle);
      }
      //no drive
      else{
        analogWrite(A1_MD, 0);
        analogWrite(A2_MD, 0);
        analogWrite(B1_MD, 0);
        analogWrite(B2_MD, 0);
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
      input_str.toLowerCase();

      if(input_str == "config"){
        Serial.print("kp: "); 
        Serial.println(kp, 4);
        Serial.print("ki: "); 
        Serial.println(ki, 4);
        Serial.print("kd: "); 
        Serial.println(kd, 4);
        Serial.print("standing angle: "); 
        Serial.println(standing_angle, 4);
        Serial.print("integral constraint: "); 
        Serial.println(integral_constraint, 4);
      }

      //debugging
      //Serial.println(input_str);

      int kpIndex = input_str.indexOf("kp");
      int kiIndex = input_str.indexOf("ki");
      int kdIndex = input_str.indexOf("kd");
      int saIndex = input_str.indexOf("sa");
      int icIndex = input_str.indexOf("ic");

      //debugging
      // Serial.println(kpIndex);
      // Serial.println(kiIndex);
      // Serial.println(kdIndex);

      if(kpIndex != -1 && kpIndex < input_str.length()){
        update_value('p', kpIndex+3);
      }
      if(kiIndex != -1 && kiIndex < input_str.length()){
        update_value('i', kiIndex+3);
      }
      if(kdIndex != -1 && kdIndex < input_str.length()){
        update_value('d', kdIndex+3);
      }
      if(saIndex != -1 && saIndex < input_str.length()){
        update_value('s', saIndex+3);
      }
      if(icIndex != -1 && icIndex < input_str.length()){
        update_value('c', icIndex+3);
      }
      input_str = "";

    }

    void update_value(char type, int index){

      float val;

      //find where the inputted value ends
      int endIndex = input_str.indexOf(' ', index);

      //if there is nothing after the value
      if(endIndex == -1){             
        val = input_str.substring(index).toFloat();
      }

      //if there is something else in the string
      else{
        val = input_str.substring(index, endIndex).toFloat();
      }

      //print if there is a possible error
      if(val == 0){
        Serial.println("Possible error detected");
      }

      //decide which k value to update
      if(type == 'p'){
        kp = val;
        Serial.print("kp: "); 
        Serial.println(kp, 4);
      }
      else if(type == 'i'){
        ki = val;
        Serial.print("ki: "); 
        Serial.println(ki, 4);
      }
      else if(type == 'd'){
        kd = val;
        Serial.print("kd: "); 
        Serial.println(kd, 4);
      }
      else if(type == 's'){
        standing_angle = val;
        Serial.print("standing angle: "); 
        Serial.println(standing_angle, 4);
      }
      else if(type == 'c'){
        integral_constraint = val;
        Serial.print("integral constraint: "); 
        Serial.println(integral_constraint, 4);
      }
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