#include "fc_arduino.h"


//logging related 
SdFs sd;
FsFile file;
SdFile root;
char fileName[13] = "xx.csv";
char f_name[20];
char *ret;
char ext[5] = ".csv";
byte lastFileNumber=0;
byte FileNumber=0;
// bool fileStatus = false; //false indicates file is either closed or not created. True indicates it is open
bool startedLogging = false;


/*Object instantiations*/
PWMServo myservo1;
PWMServo myservo2;
PWMServo myservo3;
PWMServo myservo4;
PulsePositionInput ppm;//ppm
float ppmValues[8];


ICM_20948_SPI myICM;
FusionAhrs ahrs;
FusionVector gyroscope, gyroscope_prev;
FusionVector accelerometer_uncal;
FusionVector accelerometer, accelerometer_prev;
FusionVector magnetometer, magnetometer_prev;
float roll_est, pitch_est, yaw_est; //variable for roll, ptich yaw estimates from madgwick filter
float throttle_des, roll_des, pitch_des, yaw_rate_des;
float roll_passthru, pitch_passthru, yaw_passthru;
float dt;

//
bool armed = false;

//Controller variables
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw_rate, error_yaw_rate_prev, integral_yaw_rate, integral_yaw_rate_prev, derivative_yaw_rate, yaw_PID = 0;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 15.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 20.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees)
float maxPitch = 20.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
float maxYaw = 160.0;     //Max yaw rate in deg/sec

// float Kp_roll_angle = 0.2;    //Roll P-gain
// float Ki_roll_angle = 0.3;    //Roll I-gain 
// float Kd_roll_angle = 0.05;   //Roll D-gain
// float Kp_pitch_angle = 0.2;   //Pitch P-gain
// float Ki_pitch_angle = 0.3;   //Pitch I-gain
// float Kd_pitch_angle = 0.05;  //Pitch D-gain

float Kp_roll_angle = 0.3;    //Roll P-gain
float Ki_roll_angle = 0.3;    //Roll I-gain 
float Kd_roll_angle = 0.05;   //Roll D-gain
float Kp_pitch_angle = 0.3;   //Pitch P-gain
float Ki_pitch_angle = 0.3;   //Pitch I-gain
float Kd_pitch_angle = 0.05;  //Pitch D-gain


float Kp_yaw = 0.1;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain



int i=0;
int start_time;
unsigned long current_time, last_blink, prev_time, print_counter, lastFlushTime;
// static unsigned long previousTimestamp = 0;
unsigned long timestamp;
bool led = false;
const int ledPin = 13;

float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM;


//radio related variables
/*
On Flysky 
Channel 1 : Roll 
Channel 2 : Pitch
Channel 3 : Throttle
Channel 4 : Yaw
Channel 5 : SWD
Channel 6 : SWC
Channel 7 : SWA -- Arm, Disarm
Channle 8 : SWB 
*/
unsigned long channel_1, channel_2, channel_3, channel_4, channel_5, channel_6, channel_7;
//unsigned long channel_1_prev, channel_2_prev, channel_3_prev, channel_4_prev;


//gyroscope calibration variables. These values change in the field due to temperature effects. Recommended to calibrate in field
float gyroX_bias = -1.121614;
float gyroY_bias = -0.054426;
float gyroZ_bias = 0.373040;

//accelerometer calibration variables
//calculated using magneto software
float accelX_bias = -0.003354;
float accelY_bias = -0.006880;
float accelZ_bias = -0.030498;
const FusionMatrix accMisalignment = {1.000312, -0.000326, 0.002377, -0.000326, 1.001031, -0.000630, 0.002377, -0.000630, 0.993011};

//magnetometer calibration variables

//level calibration variables
//float roll_bias=-0.56, pitch_bias=3.81;
//float roll_bias=-0.44, pitch_bias=-1.16;
float roll_bias = -1.04, pitch_bias=-0.86;
//madgwick filter variables
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float B_madgwick = 0.1;  //Madgwick filter parameter
// LP filter parameters calculated from formulas below
/*
dt = 1 / sampling_rate --- 500Hz
RC = 1 / (2π * cutoff_freq)  -- 30Hz for gyroscope and 10Hz for accelerometer
alpha = dt / (RC + dt) --alpha is B_gyro or B_accel
*/
float B_accel =  0.112;     //Accelerometer LP filter paramter
float B_gyro = 0.1;       //Gyro LP filter paramter
float B_mag = 1.0;        //Magnetometer LP filter parameter

/*
For ESC calibration put the throttle in full and power up the multicopter. This begins the ESC sequence with sounds. Once you hear beep-beep sound pull the throttle to full
low. 
*/
void esc_calibration(){
  while (true) {
    digitalWrite(ledPin, HIGH); //LED on to indicate in calibration mode

    getRadio(); //Pulls current available radio commands          
    throttle_des = (channel_3 - 1000.0)/1000.0; //Between 0 and 1
    throttle_des = constrain(throttle_des, 0.0, 1.0);
    
    m1_command_scaled = throttle_des;
    m2_command_scaled = throttle_des;
    m3_command_scaled = throttle_des;
    m4_command_scaled = throttle_des;
    
    scaleCommands();

    commandMotors();

    // delay(5000);

    // getRadio(); //Pulls current available radio commands          
    // throttle_des = (channel_3 - 1000.0)/1000.0; //Between 0 and 1
    // throttle_des = constrain(throttle_des, 0.0, 1.0);
    
    // m1_command_scaled = throttle_des;
    // m2_command_scaled = throttle_des;
    // m3_command_scaled = throttle_des;
    // m4_command_scaled = throttle_des;
    
    // scaleCommands();

    // commandMotors();
    // digitalWrite(ledPin, LOW); 
    // delay(10000);
    loopRate(2000);;
  }
}

/*Startup tasks*/
void setup() {

  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println("Debug enabled...");
  #endif
  
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
    while(1);
  }
  root.open("/");
  while(file.openNext(&root, O_RDONLY)){
    file.getName(f_name, 20);
    ret = strstr(f_name, ext);
    if(ret){
      FileNumber = atoi(&f_name[0]);
      if(FileNumber > lastFileNumber){
        lastFileNumber = FileNumber;
      }
    }
    file.close();
  }
  root.close();
  FileNumber = lastFileNumber + 1;
  fileName[0] = (int)((FileNumber/10) % 10) + '0'; //+0 is used to convert to corresponding ASCII
  fileName[1] = (int)(FileNumber % 10) + '0';

  if (sd.exists(fileName)){
    Serial.println("File already exists");;
  }

  Serial.print("Logging to: ");
  Serial.println(fileName);

  if (!file.open(fileName, O_WRITE | O_CREAT)){
    Serial.println("opening of file failed");
  }

  // Write CSV header
  // file.println(
  //   "Time_us, Throttle_des, Roll_des, Pitch_des, Yaw_des, "
  //   "Raw_gyroX, Raw_gyroY, Raw_gyroZ, Raw_accX, Raw_accY, Raw_accZ, "
  //   "Raw_magX, Raw_magY, Raw_magZ, Roll_est, Pitch_est, Yaw_est, "
  //   "Roll_PID, Pitch_PID, Yaw_PID, Motor1_cmd, Motor2_cmd, Motor3_cmd, Motor4_cmd, "
  //   "Radio_channel1, Radio_channel2, Radio_channel3, Radio_channel4, LoopRate"
  // );
    
  file.println("Time_us, LoopRate");
  file.flush();

  ppm.begin(PPM_PIN);

  /*Initializing Magdwick Fusion filter*/
  FusionAhrsInitialise(&ahrs);

  const FusionAhrsSettings settings = {
    .convention = FusionConventionNed,
    .gain = 0.5f,
    .gyroscopeRange = 250.0f, /* replace this with actual gyroscope range in degrees/s */
    .accelerationRejection = 11.0f,
    .magneticRejection = 0.0,
    .recoveryTriggerPeriod = 5 * 500, /* 5 seconds */
  };

  FusionAhrsSetSettings(&ahrs, &settings);


  /*Motor setup*/
  /* Need to change PWMServo library with increasing analog write frequency and updating the line 199 with 1/400 */
  myservo1.attach(MOTOR1, 1000, 2000); 
  myservo2.attach(MOTOR2, 1000, 2000); 
  myservo3.attach(MOTOR3, 1000, 2000); 
  myservo4.attach(MOTOR4, 1000, 2000);

  /*Led setup*/
  pinMode(ledPin, OUTPUT);

  #ifdef CALIBRATE_IMU
    gyr_calibration();
    delay(100);
    acc_calibration();
    delay(100);
    level_calibration();
  #endif

  #ifdef ESC_CALIBRATE
    esc_calibration();
  #endif

  /* IMU setup if done earlier is causing issues with fetching data*/
  SPI_PORT.begin();
  //myICM.enableDebugging();
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(CS_PIN, SPI_PORT);
    #ifdef DEBUG
      Serial.print(F("Initialization of the sensor returned: "));
      Serial.println(myICM.statusString());
    #endif
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      #ifdef DEBUG
        Serial.println("Trying again...");
      #endif
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  //start_time=micros();
  last_blink=micros();

  #ifdef GYRO_CALIBRATE
    calibrateGyro();
  #endif

  #ifdef ACC_CALIBRATE
    calibrateAcc();
  #endif

  #ifdef MAG_CALIBRATE
    calibrateMag();
  #endif

  #ifdef LEVEL_CALIBRATE
    calibrateLevel();
  #endif

}

/*super loop*/
void loop() {

  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  armedStatus();

  estimator();

  //apply level reference
  roll_est = roll_est - roll_bias;
  pitch_est = pitch_est - pitch_bias;

  desAngle();

  controlller();

  controlAllocation();

  scaleCommands();
  
  throttleCut();

  commandMotors();

  getRadio();

  //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //printMotorCommands();
  //printRadioData();
  //printControlAlloc();
  // printAttitudeLoopRate();


  //Log only when drone is armed
  // if (armed){
  //   if(!startedLogging)
  //     startedLogging = true;
    
  //   logData();
  // }
  // else if (channel_6 < 1500 && startedLogging){
  //   file.flush();
  //   file.close();//close file on throttle cut
  //   startedLogging = false;
  // }

  // logData();

  // if (millis() - lastFlushTime >= 10) {
  //   file.flush();  // Write buffered data to SD
  //   lastFlushTime = millis();
  // }

  loopRate(500); //restricts the base loop to 2000 times
}

void loopRate(int freq) {
  //Regulate main loop rate to specified frequency in Hz
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}


//blink delay in terms of seconds. Allowing only integers
void ledTask(int blink_delay){
  if (current_time - last_blink >= blink_delay*1000000){
    //Serial.println("LED task");
    digitalWrite(ledPin, led);
    led = !led;
    //Serial.println(led);
    last_blink=micros();
  }
}

void armedStatus() {
  if (channel_7 > 1500) {
    armed = true;
  }
  else {
    armed = false;
  }
}


void controlAllocation(){
  m1_command_scaled = throttle_des - pitch_PID - roll_PID - yaw_PID; //Front Right
  m2_command_scaled = throttle_des + pitch_PID + roll_PID - yaw_PID; //Back Left
  m3_command_scaled = throttle_des - pitch_PID + roll_PID + yaw_PID; //Front Left
  m4_command_scaled = throttle_des + pitch_PID - roll_PID + yaw_PID; //Back Right
}


void estimator(){
  if (myICM.dataReady())
  {
    myICM.getAGMT();

    gyroscope = {myICM.gyrX()-gyroX_bias, myICM.gyrY()-gyroY_bias, myICM.gyrZ()-gyroZ_bias}; // gyroscope data in degrees/s
    accelerometer_uncal = {myICM.accX()/1000-accelX_bias, myICM.accY()/1000-accelY_bias, myICM.accZ()/1000-accelZ_bias}; //accelerometer data in g
    magnetometer = {myICM.magX(), myICM.magY(), myICM.magZ()}; // magnetometer data in arbitrary units

    accelerometer = FusionMatrixMultiplyVector(accMisalignment, accelerometer_uncal);

    //Low pass filter for gyroscope and accelerometer
    accelerometer = FusionVectorAdd(FusionVectorMultiplyScalar(accelerometer_prev, (1.0 - B_accel)), FusionVectorMultiplyScalar(accelerometer, B_accel));
    accelerometer_prev = accelerometer;

    gyroscope = FusionVectorAdd(FusionVectorMultiplyScalar(gyroscope_prev, (1.0 - B_gyro)), FusionVectorMultiplyScalar(gyroscope, B_gyro));
    gyroscope_prev = gyroscope;

    // timestamp = micros();  // Current time in microseconds
    // float deltaTime = (timestamp - previousTimestamp) / 1000000.0;  // Convert µs to seconds
    // previousTimestamp = timestamp;

    // FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, dt);
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dt);
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    roll_est = euler.angle.roll;
    pitch_est = euler.angle.pitch;
    yaw_est = euler.angle.yaw;
  }
  else
  {
    Serial.println("Waiting for data");
  }
}

void desAngle(){
  if (armed)
    throttle_des = 0.1 + 0.8*((channel_3 - 1000.0)/1000.0); //Between 0 and 1
  else
    throttle_des = (channel_3 - 1000.0)/1000.0;
  roll_des = (channel_1 - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_2 - 1500.0)/500.0; //Between -1 and 1
  yaw_rate_des = (channel_4 - 1500.0)/500.0; //Between -1 and 1
  yaw_rate_des = 0;
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_rate_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  throttle_des = constrain(throttle_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_rate_des = constrain(yaw_rate_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlller() {
  //Roll
  error_roll = roll_des - roll_est;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_3 < 1100) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturation to prevent windup
  derivative_roll = gyroscope.axis.x; //directly applied with velocity to prevent derivative kick. Assumption is body angular velocities are equal to euler rates at small angle. Ref quan quan Eq 5.5 and Eq 11.6 and  Eq 11.9.
  // this is only valid till roll and pitch angles less than 20deg. Gyroscopes give angular velocities in body frame and not euler rates.
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_est;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_3 < 1100) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = gyroscope.axis.y;
  pitch_PID = -1*.01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw_rate = yaw_rate_des - gyroscope.axis.z;
  integral_yaw_rate = integral_yaw_rate_prev + error_yaw_rate*dt;
  if (channel_3 < 1100) {   //Don't let integrator build if throttle is too low
    integral_yaw_rate = 0;
  }
  integral_yaw_rate = constrain(integral_yaw_rate, -i_limit, i_limit); //saturation to prevent windup
  derivative_yaw_rate = (error_yaw_rate - error_yaw_rate_prev)/dt; 
  yaw_PID = -1*.01*(Kp_yaw*error_yaw_rate + Ki_yaw*integral_yaw_rate + Kd_yaw*derivative_yaw_rate); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_rate_prev = error_yaw_rate;
  integral_yaw_rate_prev = integral_yaw_rate;
}


void getRadio(){
    int channels = ppm.available();
    if(channels > 0){
        for (int i = 0; i < channels; i++) {
            ppmValues[i] = ppm.read(i + 1);  // Read valid channel values
        }
    }
    //Max and min values adjusted based on sub-trim values
    //Used Sub-trim -3% for roll, +3% for pitch and 4% for yaw on flysky fsi10
    // channel_1 = map(ppmValues[0], 995, 1995, 1000, 2000);
    // channel_2 = map(ppmValues[1], 1001, 2001, 1000, 2000);
    // channel_3 = map(ppmValues[2], 999, 1999, 1000, 2000);
    // channel_4 = map(ppmValues[3], 1001, 2001, 1000, 2000);
    // channel_5 = map(ppmValues[4], 999, 1999, 1000, 2000);
    // channel_6 = map(ppmValues[5], 999, 1999, 1000, 2000);

    channel_1 = constrain(ppmValues[0], 1000, 2000);
    channel_2 = constrain(ppmValues[1], 1000, 2000);
    channel_3 = constrain(ppmValues[2], 1000, 2000);
    channel_4 = constrain(ppmValues[3], 1000, 2000);
    channel_5 = constrain(ppmValues[4], 1000, 2000);
    channel_6 = constrain(ppmValues[5], 1000, 2000);
    channel_7 = constrain(ppmValues[6], 1000, 2000);

    // channel_1 = ppmValues[0];
    // channel_2 = ppmValues[1];
    // channel_3 = ppmValues[2];
    // channel_4 = ppmValues[3];
    // channel_5 = ppmValues[4];
    // channel_6 = ppmValues[5];

  //Scale channel values

}

void scaleCommands() {
  //Scaled to 0-180 for servo library
  m1_command_PWM = m1_command_scaled*180;
  m2_command_PWM = m2_command_scaled*180;
  m3_command_PWM = m3_command_scaled*180;
  m4_command_PWM = m4_command_scaled*180;

  //Constrain commands to servos within servo library bounds
  m1_command_PWM = constrain(m1_command_PWM, 0, 180);
  m2_command_PWM = constrain(m2_command_PWM, 0, 180);
  m3_command_PWM = constrain(m3_command_PWM, 0, 180);
  m4_command_PWM = constrain(m4_command_PWM, 0, 180);
}

void commandMotors(){
  myservo1.write(m1_command_PWM);
  myservo2.write(m2_command_PWM);
  myservo3.write(m3_command_PWM);
  myservo4.write(m4_command_PWM);
}

void throttleCut(){
  //if ((channel_6 < 1500) || (armed == false)) {
  if (channel_6 < 1500) {
    //armed = false;
    m1_command_PWM = 0;
    m2_command_PWM = 0;
    m3_command_PWM = 0;
    m4_command_PWM = 0;
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("throttle_des:"));
    Serial.print(throttle_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_rate_des:"));
    Serial.println(yaw_rate_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX:"));
    Serial.print(gyroscope.axis.x);
    Serial.print(F(" GyroY:"));
    Serial.print(gyroscope.axis.y);
    Serial.print(F(" GyroZ:"));
    Serial.println(gyroscope.axis.z);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX:"));
    Serial.print(accelerometer.axis.x);
    Serial.print(F(" AccY:"));
    Serial.print(accelerometer.axis.y);
    Serial.print(F(" AccZ:"));
    Serial.println(accelerometer.axis.z);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX:"));
    Serial.print(magnetometer.axis.x);
    Serial.print(F(" MagY:"));
    Serial.print(magnetometer.axis.y);
    Serial.print(F(" MagZ:"));
    Serial.println(magnetometer.axis.z);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_est);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_est);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_est);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("throttle_Des:"));
    Serial.print(throttle_des);
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.println(yaw_PID);
  }
}


void printMotorCommands() {
  if (current_time - print_counter > 1000000) {
    print_counter = micros();
    Serial.print(F("m1_command:"));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command:"));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command:"));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command:"));
    Serial.println(m4_command_PWM);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1:"));
    Serial.print(channel_1);
    Serial.print(F(" CH2:"));
    Serial.print(channel_2);
    Serial.print(F(" CH3:"));
    Serial.print(channel_3);
    Serial.print(F(" CH4:"));
    Serial.print(channel_4);
    Serial.print(F(" CH5:"));
    Serial.print(channel_5);
    Serial.print(F(" CH6:"));
    Serial.println(channel_6);
  }
}

void printControlAlloc(){
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command:"));
    Serial.print(m1_command_scaled);
    Serial.print(F(" m2_command:"));
    Serial.print(m2_command_scaled);
    Serial.print(F(" m3_command:"));
    Serial.print(m3_command_scaled);
    Serial.print(F(" m4_command:"));
    Serial.println(m4_command_scaled);
  }

}

//Calibration of magenetometer is done through Magneto V1.2 software. This function is just to collect raw data in nano Tesla and send it through serial.
//Data from here is stored in a text file which is later uploaded to the Magneto software which estimates the bias. 
void calibrateMag(){
  ;
}

void calibrateAcc(){
  float accelX=0.0, accelY=0.0, accelZ=0.0;
  int sample = 0;
  Serial.println("Starting accelerometer calibration...");
  Serial.println("Hold the quadcopter in static position");
  delay(5000);
  while(true){
    while(sample < 50){
      if (myICM.dataReady())
      {
        myICM.getAGMT();
        accelX = accelX + myICM.accX()/1000;
        accelY = accelY + myICM.accY()/1000;
        accelZ = accelZ + myICM.accZ()/1000;
        sample = sample + 1;
      }
    }

    accelX = accelX/sample;
    accelY = accelY/sample;
    accelZ = accelZ/sample;
    
    String merged = String(accelX, 6) + "," + String(accelY, 6) + "," + String(accelZ, 6);
    Serial.println(merged);
    Serial.println("Change orientation...");
    sample = 0;
    accelX=0.0, accelY=0.0, accelZ=0.0;
    delay(3000);
  }
}

//Collect gyroscope samples at stationary and average it to get the offset/bias along each axis
void calibrateGyro(){
  bool calibration_done = false;
  float gyroX=0.f, gyroY=0.f, gyroZ=0.f;
  int sample = 0;
  Serial.println("Starting gyroscope calibration... Place the drone stationary");
  while(!calibration_done){
    if (myICM.dataReady())
    {
      myICM.getAGMT();
      gyroX = gyroX + myICM.gyrX();
      gyroY = gyroY + myICM.gyrY();
      gyroZ = gyroZ + myICM.gyrZ();
      sample = sample + 1;
      //Serial.println(sample);
      if(sample == 10000)
        calibration_done = true;

    }
    else
    {
      ;//Serial.println("Waiting for data");
    }
  }

  gyroX_bias = gyroX/sample;
  gyroY_bias = gyroY/sample;
  gyroZ_bias = gyroZ/sample; 

  Serial.println("Calibration done");
  String merged = String(gyroX_bias, 6) + "," + String(gyroY_bias, 6) + "," + String(gyroZ_bias, 6);
  Serial.println(merged);
  delay(10000);

}

//Run the madgwick filter when the drone is level and measure the roll, pitch. Apply this to roll and pitch estimates
void calibrateLevel(){
  //waiting for estimator to converge
  unsigned long cur_time=0, p_time=0, start_time=0;
  start_time = micros();
  while(micros()-start_time < 60*1000000UL){
    p_time = cur_time;      
    cur_time = micros();      
    dt = (cur_time - p_time)/1000000.0;
    estimator();
    loopRate(500);
  }

  //calculating the level reference
  float roll_sum = 0, pitch_sum = 0;
  int count = 0;
  start_time = micros();
  while ((micros() - start_time) < 5 * 1000000UL) {
    p_time = cur_time;
    cur_time = micros();
    dt = (cur_time - p_time) / 1000000.0;
    estimator();

    roll_sum += roll_est;
    pitch_sum += pitch_est;
    count++;

    loopRate(500);
  }

  roll_bias = roll_sum / count;
  pitch_bias = pitch_sum / count;

  Serial.println("Level calibrated");
  Serial.print("Roll bias");
  Serial.println(roll_bias);
  Serial.print("Pitch bias");
  Serial.println(pitch_bias);
}

void printAttitudeLoopRate(){
  Serial.println(1/dt);
}

void logData(){
  // char line[512];
  // snprintf(line, sizeof(line),
  //   "%lu,%.3f,%.3f,%.3f,%.3f,"         // Time_us, Throttle_des, Roll_des, Pitch_des, Yaw_des
  //   "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"   // Raw_gyroX–Z, Raw_accX–Z
  //   "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"   // Raw_magX–Z, Roll_est–Yaw_est
  //   "%.3f,%.3f,%.3f,%d,%d,%d,%d,"   // Roll_PID–Yaw_PID, Motor1–4
  //   "%lu,%lu,%lu,%lu,%f\n",           // Radio_channel1–4
  //   current_time, throttle_des, roll_des, pitch_des,  yaw_rate_des,
  //   gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z, accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z,
  //   magnetometer.axis.x, magnetometer.axis.y, magnetometer.axis.z, roll_est, pitch_est, yaw_est,
  //   roll_PID, pitch_PID, yaw_PID, m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM,
  //   channel_1, channel_2, channel_3, channel_4,1/dt);
  char line[30];
  snprintf(line, sizeof(line),
    "%lu,%f\n", current_time, 1/dt);
  file.print(line);
}

