#include <Wire.h> 
#include "MPU6050.h"

////////////ABOUT ACCELEROMETER AND GYRO///////////
// Accelerometer are less sensitive to drift but highly sensitive to vibration
// Gyro is less sensitive to vibration but drifts more by time
///////////////////////////////////////////////

/*
PIN CONNECTION
VCC_IN - 3v3
SCL - PB6
SDA - PB7
*/ 
bool level_calibration_on = false;
bool set_gyro = false;

int16_t acc_x, acc_y, acc_z;
int16_t temperature;
int16_t cal_int;
int16_t manual_acc_pitch = 0;
int16_t manual_acc_roll = 0;
int16_t manual_gyro_roll = 0;
int16_t manual_gyro_pitch = 0;
int16_t manual_gyro_yaw = 0;

int32_t cal_gyro_roll = 0;
int32_t cal_gyro_pitch = 0;
int32_t cal_gyro_yaw = 0;

float roll, pitch, yaw = 0;
int16_t gyro_roll, gyro_pitch, gyro_yaw;  // dont use float
float acc_pitch, acc_roll;

uint32_t loop_timer;
uint32_t acc_total;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);     // high baud rate to reduce loop time since loop time should execute within 4ms
  Wire.begin();
  delay(100);
  Serial.println("Checking...");
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission()!=0){
    Serial.println("No MPU6050");
    while(1);
  }
  Serial.print("Calibrating");
  loop_timer = micros();
  gyro_setup();
  //calibrate_gyro();
}

void loop() {
  // put your main code here, to run repeatedly:
  read_gyro();
  // Gyro angle calculations
  // Gyro give angular velocity in one second
  // but we need angular velocity in 4ms since we are calculating gyro in 250Hz
  // So multiply 0.004 with gyro reading
  // 500dps -> 65.5
  // 0.0000611 = 0.004/65.5
  pitch += (gyro_pitch/65.5) * 0.004;
  roll  += (gyro_roll/65.5)  * 0.004;
  yaw   += (gyro_yaw/65.5)   * 0.004;
  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr)
  roll -= pitch * sin(gyro_yaw*0.000001066);
  pitch += roll * sin(gyro_yaw*0.000001066);
  
  acc_total = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  if (abs(acc_y)<acc_total) {
    // use (float)acc_y because acc_y/acc_total is alwyas <1 (if int rounded off to 0)
    // 57.296=(180/3.14)
    acc_pitch = asin((float)acc_y/acc_total)*57.296; 
    acc_pitch -= -0.6;
  }
  if (abs(acc_x)<acc_total) {
    acc_roll = asin((float)acc_x/acc_total)*57.296;
    acc_roll -= -0.4;
  }

  if (set_gyro) {
    roll = roll*0.9996 + acc_roll*0.0004;
    pitch = pitch*0.9996 + acc_pitch*0.0004;
  } else {
    // Initially if MPU is inclined it cant be detected by gyro so use
    // acclerometer for initial angle;
    roll=acc_roll;
    pitch=acc_pitch;
    set_gyro=true;
  }
  // to reduce drift small part of noisy acc is added to gyro
  

  print_raw_gyro();
  if (micros()-loop_timer > 4050){ 
    Serial.print("Loop time exceeded");
  }
  while(micros()-loop_timer < 4000);      // loop timer is set to previous loop timing (curr_time-prev_time<4000us)
  loop_timer = micros();       // micros() gives time in us
}

// This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
void calibrate_gyro() {
  // Even if gyro is not moving output seems to be fluctuating so take 
  // average of the readings and subract while reading gyro
  cal_int = 0;
  if (cal_int != 2000) {
    for(cal_int=0; cal_int < 2000; cal_int++) {
    //  if (cal_int%25 == 0) digitalWrite(PB4, !digitalRead(PB4));  
      read_gyro();
      cal_gyro_roll += gyro_roll;         // as gyro gives angular velocity (rad/s) integrate to get angular position
      cal_gyro_pitch += gyro_pitch;
      cal_gyro_yaw += gyro_yaw;
      delay(4);
    }
    cal_gyro_roll /= 2000;
    cal_gyro_pitch /= 2000;
    cal_gyro_yaw /= 2000;
    manual_gyro_roll = cal_gyro_roll;
    manual_gyro_pitch = cal_gyro_pitch;
    manual_gyro_yaw = cal_gyro_yaw;
  }
}

//This part reads the raw gyro and accelerometer data from the MPU-6050
void read_gyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  if (Wire.available() == 14) {
    acc_x = Wire.read() << 8 | Wire.read();
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temperature = Wire.read() << 8 | Wire.read();
    gyro_roll = Wire.read() << 8 | Wire.read();
    gyro_pitch = Wire.read() << 8 | Wire.read();
    gyro_yaw = Wire.read() << 8 | Wire.read();
  }
  gyro_pitch *= -1;
  gyro_yaw *= -1;///
  if (level_calibration_on == 0) {
    acc_x -= manual_acc_roll;
    acc_y -= manual_acc_pitch;
  }
  if (cal_int >= 2000) {        // cal_int should satisfy> 2000, as read function is called in calibrating function also
    gyro_roll -= manual_gyro_roll;
    gyro_pitch -= manual_gyro_pitch;
    gyro_yaw -= manual_gyro_yaw;
  }
}

//In this part the various registers of the MPU-6050 are set.
void gyro_setup() {
  Wire.beginTransmission(MPU_ADDR);   // (0x68 << 1) + W/R -> 0/1
  Wire.write(PWR_MGMT_1); // target register address is PWR_MNGT_1
  Wire.write(0x00); 
  Wire.endTransmission(true); 

  Wire.beginTransmission(0x68); // MPU6050
  Wire.write(CONFIG);      
  Wire.write(0x03);             // Set DLPF to 43Hz
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);           // ACCEL_CONFIG register
  Wire.write(0x10);           // ±8g range
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);     // GYRO_CONFIG register
  Wire.write(0x08);     // ±500dps
  Wire.endTransmission(true);
}

void print_raw_gyro() {
  Serial.print(acc_pitch); Serial.print(" ");
  Serial.println(acc_roll); Serial.print(" ");
  //Serial.println(acc_total);
}

void print_raw_acc() {}
