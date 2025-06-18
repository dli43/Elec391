  #include "Arduino_BMI270_BMM150.h"
  #include "math.h"
  #include "AS5600.h"
  #include <ArduinoBLE.h>
  #include "Wire.h"

  #define TCAADDR 0x70
  #define encoder_left 1
  #define encoder_right 0
  #define vel_buf_samples 20 // number of velocity measurements in sensor buffer

  #define print_interval 20
  #define velocity_interval 5

  const float pi = 3.14159;
  const float wheel_radius = 0.04;
  const float k = 0.993;
  const int pwm_deadzone = 46;
  float standing_angle = -1.05;
  const float gyro_bias = 0.0961;
  float integral_constraint = 14;
  bool reference_angle_computed = false;
  bool compute_complementary_angle = false;
  float encoder_velocities_left[vel_buf_samples];
  float encoder_velocities_right[vel_buf_samples];
  float encoder_velocities_average[vel_buf_samples];
  float average_velocity = 0;
  float desired_velocity = 0;
  float tau = 0.5; //time constant to correct velocity error
  float desired_acceleration = 0;
  float desired_angle = 0;
  float max_desired_angle = 3;

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
  // Initialize reset pin
  int tca_reset = 6;
  
  int duty_cycle = 0;
  int print_count = 1;
  int velocity_count = 1;
  int angle_count = 1;
  int vel_buf_tracker = 0;

  float vel_avg_kernel[vel_buf_samples];

  float kp = 3.2;        // PID values
  float ki = 17;
  float kd = 0.3;
  float pid_p;
  float pid_i = 0;
  float pid_d;
  int dutycycle_drive;
  static String input_str = "";
  char input_char = ' ';

  //right and left motor encoders
  AS5600 as5600_left;
  AS5600 as5600_right; 
  

  void setup() {
    Serial.begin(115200);       // Initialize serial communication

    pinMode(A1_MD, OUTPUT);     // Set the h-bridge driver pins as outputs
    pinMode(A2_MD, OUTPUT);
    pinMode(B1_MD, OUTPUT);
    pinMode(B2_MD, OUTPUT);

    if (!IMU.begin()) {     // Check if IMU module has initialized
      Serial.println("IMU failed");
      while (1);
    }

    // Reset I2C multiplexer
    pinMode(tca_reset, OUTPUT);
    digitalWrite(tca_reset, LOW);
    delay(25);
    digitalWrite(tca_reset,HIGH);

    Wire.begin();       // Initialize I2C

    pinMode(LED_BUILTIN, OUTPUT);
    initialize_BLE();

    //populate encoder arrays with zeroes
    for(int i = 0; i < vel_buf_samples; i++){
      encoder_velocities_left[i] = 0;
      encoder_velocities_right[i] = 0;
    }

    // Initialize filter kernels
    initialize_kernel(vel_avg_kernel, vel_buf_samples);
    
  }

  void loop() {

    if(IMU.accelerationAvailable()){
      read_accelerometer(); //update theta_a
      //Serial.println("accel update");
    }

    if(IMU.gyroscopeAvailable()){
      IMU.readGyroscope(gx,gy,gz);            // Read angular velocity values into variables 
      gx -= gyro_bias;
      find_elapsed_time();
      //Serial.println("gyro update");
      compute_complementary_angle = true;
    }

    //once gyro angle has been updated the complementary angle can be computed
    if(compute_complementary_angle){

      //Update complementary tilt angle
      theta_k = k*(theta_k_prev+(-gx)*elapsed_time) + (1-k)*theta_a;

      //update sensor buffers with most recent measurements
      update_sensor_buffers();

      //calculate desired tilt
      if(velocity_count % velocity_interval == 0){ // Error velocity and tilt control angle adjusted every velocity_interval samples
        average_velocity = get_filtered_value(encoder_velocities_average, vel_avg_kernel, vel_buf_samples, vel_buf_tracker);
        desired_acceleration = (desired_velocity-average_velocity)/tau;
        desired_angle = (180/pi)*atan(desired_acceleration/9.807);
        desired_angle = constrain(desired_angle, -max_desired_angle, max_desired_angle);
      }
      calculate_pwm();
      drive_wheels();

      if(print_count % print_interval == 0){
        print_info();
      }

      print_count = (print_count + 1) % print_interval;
      velocity_count = (velocity_count + 1) % velocity_interval;
      vel_buf_tracker = (vel_buf_tracker + 1) % vel_buf_samples;
      compute_complementary_angle = false;

      //set previous theta_k value for next loop
      theta_k_prev = theta_k;
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

    float pid = pid_p+pid_i+pid_d;

    if(pid > 0){
      dutycycle_drive = int(pid+0.5);
    }
    else if(pid < 0){
      dutycycle_drive = int(pid-0.5);
    }
    else{
      dutycycle_drive = 0;
    }

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
    Serial.print("v_avg: ");
    Serial.print(average_velocity);
    Serial.print(" desired acceleration: ");
    Serial.print(desired_acceleration);
    Serial.print(" desired angle: ");
    Serial.print(desired_angle);
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

      parse_BLE_string(String(receivedString));

      Serial.print("Received data: ");
      Serial.println(receivedString);

      customCharacteristic.writeValue("Data received");
    }

  }

  void parse_BLE_string(String BLE_string){
    BLE_string.toLowerCase();

    if(BLE_string.indexOf("forward") != -1){
      desired_velocity = 0.10;
    }
    else if(BLE_string.indexOf("backward") != -1){
      desired_velocity = -0.10;
    }
    else if(BLE_string.indexOf("stop") != -1){
      desired_velocity = 0;
    }
  }

  void update_average_velocity(){
    //remove old measurements
    average_velocity -= encoder_velocities_left[angle_count]/(2*vel_buf_samples);
    average_velocity -= encoder_velocities_right[angle_count]/(2*vel_buf_samples);

    //measure new velocities
    tcaselect(encoder_left);
    encoder_velocities_left[angle_count] = as5600_left.getAngularSpeed(AS5600_MODE_RADIANS);
    encoder_velocities_left[angle_count] *= -wheel_radius;
    tcaselect(encoder_right);
    encoder_velocities_right[angle_count] = as5600_right.getAngularSpeed(AS5600_MODE_RADIANS);
    encoder_velocities_right[angle_count] *= -wheel_radius;

    //god only knows why the fuck i have to multiply by a negative here

    //add new values to average
    average_velocity += encoder_velocities_left[angle_count]/(2*vel_buf_samples);
    average_velocity += encoder_velocities_right[angle_count]/(2*vel_buf_samples);
  }

  // Select channel on multiplexer
  void tcaselect(uint8_t i) {
    if (i > 7) return;
  
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();  
  }

  void update_sensor_buffers(){
    encoder_velocities_left[vel_buf_tracker] = as5600_left.getAngularSpeed(AS5600_MODE_RADIANS);
    encoder_velocities_left[angle_count] *= -wheel_radius;
    encoder_velocities_right[vel_buf_tracker] = as5600_right.getAngularSpeed(AS5600_MODE_RADIANS);
    encoder_velocities_right[angle_count] *= -wheel_radius;
    encoder_velocities_average[vel_buf_tracker] = (encoder_velocities_left[angle_count] + encoder_velocities_right[angle_count])/2;
  }

  float get_filtered_value(float* signal, float* kernel, int length, int index_pnt){
    float filtered_val = 0.0;

    for(int i = 0; i < length; i++){
      filtered_val += kernel[i]*signal[(length + (index_pnt - i)) % length];
    }

    return filtered_val;
  }

  void initialize_kernel(float* kernel, int length){
    float sum = 0.0;

    for(int i = 0; i < length; i++){
      kernel[i] = exp(-4*i/(float)(length-1));
      sum += kernel[i];
    }

    for(int i = 0; i < length; i++){
    kernel[i] /= sum;
    }
  }