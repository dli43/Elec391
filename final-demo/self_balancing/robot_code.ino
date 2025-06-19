  #include "Arduino_BMI270_BMM150.h"
  #include "math.h"
  #include "AS5600.h"
  #include <ArduinoBLE.h>
  #include "Wire.h"

  #define TCAADDR 0x70
  #define encoder_left 1
  #define encoder_right 0
  #define buf_samples_velocity 25 // number of velocity measurements in sensor buffer
  #define buf_samples_position 4  // number of position measurements in sensor buffer

  #define interval_print 20
  #define interval_position 4
  #define interval_velocity 8

  // BLE states
  #define IDLE 0 
  #define DRIVE_FORWARD 1
  #define DRIVE_BACKWARD 2
  #define TURN_LEFT 3
  #define TURN_RIGHT 4

  // Constants and flags
  const float pi = 3.14159;
  const float wheel_radius = 0.04;
  const float k = 0.993;
  const float kphi = 0.202;   // Motor constant
  const int pwm_deadzone = 46;
  float standing_angle = -0.5;
  const float gyro_bias = 0.0961;
  float integral_constraint = 14;
  bool reference_angle_computed = false;
  bool compute_complementary_angle = false;
  
  // Velocity controller 
  int buf_tracker_velocity = 0;
  float encoder_left_velocities[buf_samples_velocity];
  float encoder_right_velocities[buf_samples_velocity];
  float kernel_velocity[buf_samples_velocity];
  float desired_left_velocity = 0;
  float desired_right_velocity = 0;
  float measured_left_velocity = 0;
  float measured_right_velocity = 0;
  float correct_left_velocity = 0;
  float correct_right_velocity = 0;
  float correct_robot_velocity = 0;
  float offset_wheel_velocity = 0;
  float max_correct_velocity = 0.5;

  float tau_velocity = 0.7; //time constant to correct velocity error
  float desired_acceleration = 0;
  float desired_angle = 0;
  float max_desired_angle = 25;
  float drive_speed = 0.3; //speed to drive robot at when given move commands
  float reference_position = 0; // refrence where the robot starts

  // Position controller
  int buf_tracker_position = 0;
  long encoder_left_ticks = 0;
  long encoder_right_ticks = 0;
  float encoder_left_position[buf_samples_position];
  float encoder_right_position[buf_samples_position];
  float kernel_position[buf_samples_position];
  float desired_left_position = 0;
  float desired_right_position = 0;
  float measured_left_position = 0;
  float measured_right_position = 0;
  float correct_left_position = 0;
  float correct_right_position = 0;
  float tau_position = 1.7;
  float k_turn = 10;
  int max_differential_correction = 5;

  // Bluetooth
  const char* BLE_name = "TEAM1-BLE-ROBOT";
  const char* service_UUID = "00000000-5EC4-4083-81CD-A10B8D5CF6EC";
  const char* characteristic_UUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC";
  BLEDevice central;
  bool isConnected = false;
  const int BUFFER_SIZE = 20;
  BLEService customService(service_UUID);
  BLECharacteristic customCharacteristic(characteristic_UUID, BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);
  int ble_state = 0;

  // IMU sensor
  float ax,ay,az;
  float gx,gy,gz;
  float theta_a, theta_g, theta_k, theta_k_prev;
  unsigned long prev_time, present_time;
  float elapsed_time;
  float integral = 0;

  // GPIO pins
  int A1_MD = 5;       // Set the pin numbers for the h-bridge driver motor 
  int A2_MD = 4;   
  int B1_MD = 2;   
  int B2_MD = 3;
  int tca_reset = 6;
  
  // Counters 
  int print_count = 1;
  int position_count = 1;
  int velocity_count = 1;
  
  
  // Tilt Angle controller
  float kp = 3.4;     // PID values
  float ki = 17;
  float kd = 0.3;
  float pid_p = 0;
  float pid_i = 0;
  float pid_d = 0;
  int duty_cycle_left = 0;
  int duty_cycle_right = 0;
  int dutycycle_drive_left = 0;
  int dutycycle_drive_right = 0;
  static String input_str = "";
  char input_char = ' ';

  //right and left motor encoders
  AS5600 as5600_left;
  AS5600 as5600_right; 
  

  void setup() {
    Serial.begin(115200);       // Initialize serial communication

    set_motordrivers();         // Set the h-bridge driver pins 

    if (!IMU.begin()) {     // Check if IMU module has initialized
      Serial.println("IMU failed");
      while (1);
    }

    reset_tca();    // Reset I2C multiplexer
    
    Wire.begin();       // Initialize I2C
    delay(100);

    pinMode(LED_BUILTIN, OUTPUT);
    initialize_BLE();

    //populate encoder arrays with zeroes
    initialize_buffers();

    // Initialize filter kernels
    initialize_kernel(kernel_velocity, buf_samples_velocity);
    initialize_kernel(kernel_position, buf_samples_position);

    // Initialize encoder position
    tcaselect(encoder_left);
    as5600_left.setZPosition(0);
    tcaselect(encoder_right);
    as5600_right.setZPosition(0);
    
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

      measure_wheel_ticks(); //find current position in wheel ticks
      calculate_wheel_positions(); //update position buffers with current position in meters

      //calculate desired position
      if(position_count % interval_position == 0){
        measured_left_position = get_filtered_value(encoder_left_position, kernel_position, buf_samples_position, buf_tracker_position);
        measured_right_position = get_filtered_value(encoder_right_position, kernel_position, buf_samples_position, buf_tracker_position);
        set_ble_state();
        correct_left_position = desired_left_position - measured_left_position;
        correct_right_position = desired_right_position - measured_right_position;
        desired_left_velocity = correct_left_position/tau_position;
        desired_right_velocity = correct_right_position/tau_position;
      }

      //calculate desired velocity and tilt
      if(velocity_count % interval_velocity == 0){ // Error velocity and tilt control angle adjusted every velocity_interval samples
        measured_left_velocity = wheel_radius*get_filtered_value(encoder_left_velocities, kernel_velocity, buf_samples_velocity, buf_tracker_velocity);
        measured_right_velocity = wheel_radius*get_filtered_value(encoder_right_velocities, kernel_velocity, buf_samples_velocity, buf_tracker_velocity);
        correct_left_velocity = desired_left_velocity - measured_left_velocity;
        correct_right_velocity = desired_right_velocity - measured_right_velocity;
        correct_robot_velocity = (correct_left_velocity + correct_right_velocity)/2;
        correct_robot_velocity = constrain(correct_robot_velocity, -max_correct_velocity, max_correct_velocity);
        offset_wheel_velocity = abs(correct_left_velocity - correct_right_velocity)/(2*wheel_radius);
        desired_acceleration = correct_robot_velocity/tau_velocity;
        desired_angle = (180/pi)*atan(desired_acceleration/9.807);
        desired_angle = constrain(desired_angle, -max_desired_angle, max_desired_angle);
      }
      
      calculate_pwm();
      drive_wheels();

      if(print_count % interval_print == 0){
        print_info();
      }

      print_count = (print_count + 1) % interval_print;
      position_count = (position_count + 1) % interval_position;
      velocity_count = (velocity_count + 1) % interval_velocity;
      buf_tracker_position = (buf_tracker_position + 1) % buf_samples_position;
      buf_tracker_velocity = (buf_tracker_velocity + 1) % buf_samples_velocity;
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
    float pid_left;
    float pid_right;

    if(correct_left_velocity - correct_right_velocity > 0){
      pid_left = pid + kphi*offset_wheel_velocity;
      pid_right = pid - kphi*offset_wheel_velocity;
    }
    else if(correct_right_velocity - correct_left_velocity > 0){
      pid_left = pid - kphi*offset_wheel_velocity;
      pid_right = pid + kphi*offset_wheel_velocity;
    }
    else{
      pid_left = pid;
      pid_right = pid;
    }

    float position_mismatch = correct_left_position - correct_right_position; // meters
    float diff_correction = constrain(k_turn * position_mismatch, -max_differential_correction, max_differential_correction);

    pid_left += diff_correction;
    pid_right -= diff_correction;

    dutycycle_drive_left = round_float(pid_left);
    dutycycle_drive_right = round_float(pid_right);

    duty_cycle_left = min(abs(dutycycle_drive_left)+pwm_deadzone, 255);
    duty_cycle_right = min(abs(dutycycle_drive_right)+pwm_deadzone, 255);
  }

  int round_float(float f){
    if(f > 0){
      return int(f+0.5);
    }
    else if(f < 0){
      return int(f-0.5);
    }
    else{
      return 0;
    }
  }

  void drive_wheels(){

    set_wheel_speed(A1_MD, A2_MD, dutycycle_drive_left, duty_cycle_left);
    set_wheel_speed(B1_MD, B2_MD, dutycycle_drive_right, duty_cycle_right);
    
  }

  void set_wheel_speed(int pin1, int pin2, int drive, int duty_cycle){
    //drive forward
    if(drive > 0){
      analogWrite(pin1, duty_cycle);
      analogWrite(pin2, 0);
    }
    //drive backward
    else if(drive < 0){
      analogWrite(pin1, 0);
      analogWrite(pin2, duty_cycle);
    }
    //no drive
    else{
      analogWrite(pin1, 0);
      analogWrite(pin2, 0);
    }
  }

  void print_info(){
    Serial.print("Theta: ");
    Serial.print(theta_k);
    Serial.print(" | P: ");
    Serial.print(pid_p); 
    Serial.print(" | I: ");
    Serial.print(pid_i);
    Serial.print(" | D: ");
    Serial.print(pid_d);
    Serial.print("\n");
    Serial.print("v_error: ");
    Serial.print(correct_robot_velocity);
    Serial.print(" desired acceleration: ");
    Serial.print(desired_acceleration);
    Serial.print(" desired angle: ");
    Serial.print(desired_angle);
    Serial.print("\n");
    // Serial.print("left pos: ");
    // Serial.println(position_left);
    // Serial.print("right pos: ");
    // Serial.println(position_right);
    // Serial.print("BLE State: ");
    // Serial.println(ble_state);
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

    int kpIndex = input_str.indexOf("kp");
    int kiIndex = input_str.indexOf("ki");
    int kdIndex = input_str.indexOf("kd");
    int saIndex = input_str.indexOf("sa");
    int icIndex = input_str.indexOf("ic");
    int ptIndex = input_str.indexOf("pt");

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
    if(ptIndex != -1 && ptIndex < input_str.length()){
      update_value('t', ptIndex+3);
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
    else if(type == 't'){
      tau_position = val;
      Serial.print("pos tau: "); 
      Serial.println(tau_position, 4);
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
      ble_state = DRIVE_FORWARD;
    }
    else if(BLE_string.indexOf("backward") != -1){
      ble_state = DRIVE_BACKWARD;
    }
    else if(BLE_string.indexOf("left") != -1){
      ble_state = TURN_LEFT;
    }
    else if(BLE_string.indexOf("right") != -1){
      ble_state = TURN_RIGHT;
    }
    else if(BLE_string.indexOf("stop") != -1){
      desired_left_position = measured_left_position;
      desired_right_position = measured_right_position;
      ble_state = IDLE;
    }
  }

  // Select channel on multiplexer
  void tcaselect(uint8_t i) {
    if (i > 7) return;
  
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();  
  }

  void update_sensor_buffers(){
    tcaselect(encoder_left);
    encoder_left_velocities[buf_tracker_velocity] = as5600_left.getAngularSpeed(AS5600_MODE_RADIANS);
    tcaselect(encoder_right);
    encoder_right_velocities[buf_tracker_velocity] = as5600_right.getAngularSpeed(AS5600_MODE_RADIANS);
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

  void set_motordrivers(){
    pinMode(A1_MD, OUTPUT);     
    pinMode(A2_MD, OUTPUT);
    pinMode(B1_MD, OUTPUT);
    pinMode(B2_MD, OUTPUT);
  }

  void reset_tca(){
    pinMode(tca_reset, OUTPUT);
    digitalWrite(tca_reset, LOW);
    delay(25);
    digitalWrite(tca_reset,HIGH);
  }

  //measures distance traveled by each wheel in encoder ticks
  void measure_wheel_ticks(){
    tcaselect(encoder_left);
    encoder_left_ticks = as5600_left.getCumulativePosition();
    tcaselect(encoder_right);
    encoder_right_ticks = as5600_right.getCumulativePosition();
  }

  //converts the encoder tick variables to meters
  void calculate_wheel_positions(){
    encoder_left_position[buf_tracker_position] = encoder_left_ticks * (2*pi*wheel_radius)/4096;
    encoder_right_position[buf_tracker_position] = encoder_right_ticks * (2*pi*wheel_radius)/4096;
  }

  void initialize_buffers(){
    for(int i = 0; i < buf_samples_velocity; i++){
      encoder_left_velocities[i] = 0;
      encoder_right_velocities[i] = 0;
    }
    for(int i = 0; i < buf_samples_velocity; i++){
      encoder_left_position[i] = 0;
      encoder_right_position[i] = 0;
    }
  }

  void set_ble_state(){
    if(ble_state == IDLE){

    }
    else if(ble_state == DRIVE_FORWARD){
      desired_left_position = drive_speed*tau_position + measured_left_position;
      desired_right_position = drive_speed*tau_position + measured_right_position;
    }
    else if(ble_state == DRIVE_BACKWARD){
      desired_left_position = -drive_speed*tau_position + measured_left_position;
      desired_right_position = -drive_speed*tau_position + measured_right_position;
    }
    else if(ble_state == TURN_LEFT){
      desired_left_position -= 0.06;
      desired_right_position += 0.06;
    }
    else if(ble_state == TURN_RIGHT){
      desired_left_position += 0.06;
      desired_right_position -= 0.06;
    }

  }