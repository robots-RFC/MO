//youtube.com/TARUNKUMARDAHAKE
//facebook.com/TARUNKUMARDAHAKE
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

int MIN_ABS_SPEED = 70;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// Kalman Filter Variables
float Q_angle = 0.01;   // Process noise variance for the angle
float Q_bias = 0.005;    // Process noise variance for the gyro bias
float R_measure = 0.1;  // Measurement noise variance (from accelerometer)
float angle = 0;         // Estimated angle (output)
float bias = 0;          // Gyro bias estimate
float P[2][2] = { {0, 0}, {0, 0} }; // Error covariance matrix
float dt = 0.01; // Sampling time (10ms)

// orientation/motion vars
Quaternion q;         // [w, x, y, z] quaternion container
VectorFloat gravity;  // [x, y, z] gravity vector
float ypr[3];         // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 182.5;
double setpoint = originalSetpoint;

int MotorAspeed, MotorBspeed;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 1000;
unsigned long last_move = 0;
unsigned long last_change = 0;w

float MOTORSLACK_A = 70;  // Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B = 70;
#define BALANCE_PID_MIN -255  // Define PID limits to match PWM max in reverse and foward
#define BALANCE_PID_MAX 255

//adjust these values to fit your own design
double Kp = 200;
double Kd = 0;
double Ki = 0;

#define RKp 50   //Set this first
#define RKd 4    //Set this secound
#define RKi 300  //Finally set this

double ysetpoint;
double yoriginalSetpoint = 0;
double input, yinput, youtput, output, Buffer[3];

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
PID rot(&yinput, &youtput, &ysetpoint, RKp, RKi, RKd, DIRECT);

#define forward_pin 10
#define backward_pin 9
#define left_pin 11
#define right_pin 12

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.6;
//MOTOR CONTROLLER
int ENA = 3;
int IN1 = 4;
int IN2 = 5;
int IN3 = 8;
int IN4 = 7;
int ENB = 6;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  pinMode(forward_pin, INPUT);
  pinMode(backward_pin, INPUT);
  pinMode(left_pin, INPUT);
  pinMode(right_pin, INPUT);
// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-26);
  mpu.setYGyroOffset(-39);
  mpu.setZGyroOffset(-25);
  mpu.setZAccelOffset(9060);  // 1688 factory default for my test chip

  Serial.begin(115200);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

    rot.SetMode(AUTOMATIC);
    rot.SetSampleTime(10);
    rot.SetOutputLimits(-40, 40);

    setpoint = originalSetpoint;
    ysetpoint = yoriginalSetpoint;
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}


void loop() {
  getvalues();
  printval();
  //Serial.println(digitalRead(11));
  if (millis() - last_move > 200) {
    //move();
    last_move = millis();
  }

  /*
  if (millis() - last_change > 3000) {
    setpoint += 1;
    last_change = millis();
    Serial.println(setpoint);
  }
  */

  //Serial.println(motorSpeedFactorLeft);
}

void move() {
  if (digitalRead(forward_pin)) {
    setpoint = 185;
  } else if (digitalRead(backward_pin)) {
    setpoint = 180;
  } else {
    setpoint = originalSetpoint;
  }

  if (digitalRead(left_pin)) {
    ysetpoint = ysetpoint - 10;
  } else if (digitalRead(right_pin)) {
    ysetpoint = ysetpoint + 10;
  }
}

void printval() {
  Serial.print(yinput);
  Serial.print("\t");
  Serial.print(youtput);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.print(output);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(MotorAspeed);
  Serial.print("\t");
  Serial.print(MotorBspeed);
  Serial.println("\t");
}

float kalmanFilter(float newAngle, float newRate, float dt) {
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}


void getvalues() {
  // if programming failed, don't try to do anything

  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    new_pid();
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    /*
    mpu.dmpGetQuaternion(&q, fifoBuffer);       //get value for q
    mpu.dmpGetGravity(&gravity, &q);            //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  //get value for ypr

    input = ypr[1] * 180 / M_PI + 180;
    yinput = ypr[0] * 180 / M_PI;
    */

    int16_t gy = mpu.getRotationY();
    int16_t ax = mpu.getAccelerationX();
    int16_t az = mpu.getAccelerationZ();

    // Convert gyro to deg/s (MPU6050 default: 131 LSB/(deg/s))
    float gyroRate = gy / 131.0;

    // Calculate angle from accelerometer (degrees)
    float accelAngle = atan2(ax, az) * 180 / M_PI + 180;

    // Apply Kalman filter
    input = kalmanFilter(accelAngle, gyroRate, dt);
  }
}

void new_pid() {
  //Compute error
  pid.Compute();
  rot.Compute();
  // Convert PID output to motor control
  MotorAspeed = compensate_slack(youtput, output, 0);
  MotorBspeed = compensate_slack(youtput, output, 1);
  motorController.move(MotorAspeed, MotorBspeed, MIN_ABS_SPEED);
}

double compensate_slack(double yOutput, double Output, bool A) {
  // Compensate for DC motor non-linear "dead" zone around 0 where small values don't result in movement
  //yOutput is for left,right control
  if (A) {
    if (Output >= 0)
      Output = Output + MOTORSLACK_A - yOutput;
    if (Output < 0)
      Output = Output - MOTORSLACK_A - yOutput;
  } else {
    if (Output >= 0)
      Output = Output + MOTORSLACK_B + yOutput;
    if (Output < 0)
      Output = Output - MOTORSLACK_B + yOutput;
  }
  Output = constrain(Output, BALANCE_PID_MIN, BALANCE_PID_MAX);
  return Output;
}