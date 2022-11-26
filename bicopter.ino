#include <Wire.h>
#include <Servo.h>
 
Servo right_prop;
Servo left_prop;
 
/////SETTINGS
#define IMU_CALIBRATE true
 
/////IMU mpu6050
#define MPU 0x68
#define ACCEL 0x3B
#define GYRO 0x43
float temp;
float AccRawX1, AccRawY1, AccRawZ1, GyRawX1, GyRawY1, GyRawZ1;
float AccRawX2, AccRawY2, AccRawZ2, GyRawX2, GyRawY2, GyRawZ2;
float AccAngleX, AccAngleY, AccAngleZ, GyAngleX, GyAngleY, GyAngleZ;
float roll, pitch, yaw;
float mpuTimer, mpuDTime;
//--calibration
float AccRawXc, AccRawYc, AccRawZc, GyRawXc, GyRawYc, GyRawZc;
 
/////kalman
float kalmanR;
float kalmanP;
float Xt, Xt_update, Xt_prev;
float Pt, Pt_update, Pt_prev;
float Kt;
float R, Q;
 
float elapsedTimes, time, timePrev;
int i;
 
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
/////////////////PID CONSTANTS/////////////////
double kp = 3.4;  //3.55
double ki = 0.0000000001; //0.003
double kd = 2.72;     //2.05
 
double throttleR = 1150;
double throttleL = 1290; //initial value of throttle to the motors
float desired_angle = 0;
 
void setup() {
  mpu6050_setup();
  calibrate_mpu6050();
  Serial.begin(19200);
  R = 10;
  Q = 0.1;
  Pt_prev = 1;
  // put your setup code here, to run once:
  time = millis();      
  left_prop.attach(10);
  right_prop.attach(3);
  left_prop.writeMicroseconds(2000);
  right_prop.writeMicroseconds(2000);
  delay(2000);
  left_prop.writeMicroseconds(1000);
  right_prop.writeMicroseconds(1000);
  delay(2000);
}
 
void loop() {
  // put your main code here, to run repeatedly:
  calculate_angle();
  kalmanR = kalman_filter(roll);
  kalmanP = kalman_filter(pitch);
 
  timePrev = time;  
  time = millis();  
  elapsedTimes = (time - timePrev) / 1000;
 
  error = desired_angle - kalmanP;
  pid_p = kp * error;
 
  if (-3 < error < 3) {
  pid_i = pid_i + (ki * error);
  }
 
  pid_d = kd * ((error - previous_error) / elapsedTimes);
 
  PID = pid_p + pid_i + pid_d;
  if (PID < -1000) {
    PID = -1000;
  }
  if (PID > 1000) {
    PID = 1000;
  }
 
  pwmLeft = throttleL + PID;
  pwmRight = throttleR - PID;
 
  if (pwmRight < 1150) {

    pwmRight = 1150;
  }
  if (pwmRight > 1475) {
    pwmRight = 1475;
  }
  //Left
  if (pwmLeft < 1290) {
    pwmLeft = 1290;
  }
  if (pwmLeft > 1500) {
    pwmLeft = 1500;
  }
 
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);
  previous_error = error;

  Serial.print(" \terror : ");
  Serial.print(error);
  Serial.print(" \tkalmanP : ");
  Serial.print(kalmanP);
  Serial.print(" \tPWM RIGHT : ");
  Serial.print(pwmRight);
  Serial.print(" \tPWM LEFT : ");
  Serial.print(pwmLeft);
  Serial.print(" \tPID : ");
  Serial.print(PID);
  Serial.println(" ");
}
 
void mpu6050_setup() {
  Wire.beginTransmission(MPU);  
  Wire.write(MPU);
  Wire.write(0b00000111);
  Wire.endTransmission();
  //mpu reset
  Wire.beginTransmission(0x68);
  Wire.write(MPU);
  Wire.write(0b00000000);
  Wire.endTransmission();
  // low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0b00000001);
  Wire.endTransmission();
  // powe management / reset
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // acelero dan high pass filter
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0b00000111);
  Wire.endTransmission();
  // gyro reset
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
}
 
void read_mpu6050_accel() {
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 8);
  AccRawX1 = Wire.read() << 8 | Wire.read();  //high and low accel data x
  AccRawY1 = Wire.read() << 8 | Wire.read();  //high and low accel data y
  AccRawZ1 = Wire.read() << 8 | Wire.read();  //high and low accel data z
}
 
void read_mpu6050_gyro() {
  Wire.beginTransmission(MPU);
  Wire.write(GYRO);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);
  GyRawX1 = Wire.read() << 8 | Wire.read();
  GyRawY1 = Wire.read() << 8 | Wire.read();
  GyRawZ1 = Wire.read() << 8 | Wire.read();
}
 
void calibrate_mpu6050() {
  Serial.print("calibrating");
 
  for (int i = 0; i < 2000; i++) {
    read_mpu6050_accel();
    read_mpu6050_gyro();
    AccRawXc += AccRawX1;
    AccRawYc += AccRawY1;
    AccRawZc += AccRawZ1;
    GyRawXc += GyRawX1;
    GyRawYc += GyRawY1;
    GyRawZc += GyRawZ1;
 
    if (i % 100 == 0) Serial.print(".");
  }
  AccRawXc /= 2000;
  AccRawYc /= 2000;
  AccRawZc /= 2000;
  GyRawXc /= 2000;
  GyRawYc /= 2000;
  GyRawZc /= 2000;
  Serial.println(AccRawXc);
  Serial.println(AccRawYc);
  Serial.println(AccRawZc);
}
 
void calculate_angle() {
  read_mpu6050_accel();
 
  AccRawX2 = (AccRawX1 + (-1 * AccRawXc)) / 16384.0;
  AccRawY2 = (AccRawY1 + (-1 * AccRawYc)) / 16384.0;
  AccRawZ2 = (AccRawZ1 + (8192 - AccRawZc)) / 16384.0;

 
  AccAngleX = (atan2(AccRawX2, AccRawZ2)) * 57.2957795;
  AccAngleY = (atan2(AccRawY2, AccRawZ2)) * 57.2957795;
 
  for (int i = 0; i < 250; i++) {
    mpuDTime = (millis() - mpuTimer) / 1000;
    read_mpu6050_gyro();
 
    GyRawX2 = GyRawX1 / 131.0;
    GyRawY2 = GyRawY1 / 131.0;
 
    GyAngleX += GyRawX2 * mpuDTime;
    GyAngleY += GyRawY2 * mpuDTime;
  }
  GyAngleX = (GyAngleX + (-1 * GyRawXc)) / 250;
  GyAngleY = (GyAngleY + (-1 * GyRawYc)) / 250;
 
  roll = 0.9998 * AccAngleX + 0.0002 * GyAngleX;
  pitch = 0.9998 * AccAngleY + 0.0002 * GyAngleY;
}
 
float kalman_filter(float data) {
  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  Kt = Pt_update / (Pt_update + R);
  Xt = Xt_update + (Kt * (data - Xt_update));\
  Pt = (1 - Kt) * Pt_update;
 
  Xt_prev = Xt;
  Pt_prev = Pt;
 
  return Xt;
}