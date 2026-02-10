#include "I2Cdev.h"
#include <Wire.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>
//#include <digitalIOPerformance.h>                //library for faster pin R/W
//#include <Ultrasonic.h>
#include <LMotorController.h>

#define REMOTEXY_MODE__SOFTSERIAL
#define REMOTEXY_SERIAL_RX 11
#define REMOTEXY_SERIAL_TX 10
#define REMOTEXY_SERIAL_SPEED 9600
#include <RemoteXY.h>

#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 56 bytes
  { 255, 5, 0, 0, 0, 49, 0, 19, 0, 0, 0, 0, 31, 1, 106, 200, 1, 1, 4, 0,
    5, 23, 73, 60, 60, 32, 177, 26, 31, 1, 9, 19, 24, 24, 0, 2, 31, 0, 1, 41,
    18, 24, 24, 0, 2, 31, 0, 1, 74, 18, 24, 24, 0, 2, 31, 0 };

// структура определяет все переменные и события вашего интерфейса управления
struct {

  // input variables
  int8_t joystick_01_x;  // oт -100 до 100
  int8_t joystick_01_y;  // oт -100 до 100
  uint8_t button_01;     // =1 если кнопка нажата, иначе =0
  uint8_t button_02;     // =1 если кнопка нажата, иначе =0
  uint8_t button_03;     // =1 если кнопка нажата, иначе =0

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

#define d_speed 0.5
#define d_dir 3


int ENA = 3;
int IN1 = 4;
int IN2 = 5;
int IN3 = 8;
int IN4 = 7;
int ENB = 6;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 1, 1);


int MIN_ABS_SPEED = 70;

char content = 'P';
int MotorAspeed, MotorBspeed;
float MOTORSLACK_A = 40;  // Compensate for motor slack range (low PWM values which result in no motor engagement)
float MOTORSLACK_B = 40;
#define BALANCE_PID_MIN -255  // Define PID limits to match PWM max in reverse and foward
#define BALANCE_PID_MAX 255

MPU6050 mpu;

//Ultrasonic ultrasonic(A0, A1);
//int distance;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



/*********Tune these 4 values for your BOT*********/
double setpoint;  //set the value when the bot is perpendicular to ground using serial monitor.
double originalSetpoint;
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 20;
double Kd = 0.5;
double Ki = 5;

double input, yinput, youtput, output, Buffer[3];

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  Serial.begin(115200);
  RemoteXY_Init();
  init_imu();              //initialiser le MPU6050
  originalSetpoint = 182;  //consigne
  setpoint = originalSetpoint;
}



void loop() {
  getvalues();
  Bt_control();
  printval();
}


void Bt_control() {
  RemoteXY_Handler();

  if (RemoteXY.connect_flag) {
    //Serial.print(RemoteXY.joystick_01_x);
    if (RemoteXY.joystick_01_y > 10){
      setpoint = originalSetpoint + d_speed;
    } else if (RemoteXY.joystick_01_y < -10) {
      setpoint = originalSetpoint - d_speed;
    } else {
      setpoint = originalSetpoint;
    }
    
  }
  else{
    setpoint = originalSetpoint;
  }

}




void init_imu() {
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  Wire.begin();
  TWBR = 24;
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-26);
  mpu.setYGyroOffset(-39);
  mpu.setZGyroOffset(-25);
  mpu.setZAccelOffset(9060);  // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);

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

    mpu.dmpGetQuaternion(&q, fifoBuffer);       //get value for q
    mpu.dmpGetGravity(&gravity, &q);            //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  //get value for ypr

    input = ypr[1] * 180 / M_PI + 180;
    yinput = ypr[0] * 180 / M_PI;
  }
}


void new_pid() {
  //Compute error
  pid.Compute();

  motorController.move(output, MIN_ABS_SPEED);
}


void printval() {
  Serial.print(yinput);
  Serial.print("\t");
  Serial.print(youtput);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.print(originalSetpoint);
  Serial.print("\t");
  Serial.print(setpoint);
  Serial.print("\t");
  Serial.print(output);
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(MotorAspeed);
  Serial.print("\t");
  Serial.print(MotorBspeed);
  Serial.print("\t");
  Serial.print(content);
  Serial.println("\t");
}
