#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Kalman Filter Variables
float Q_angle = 0.001;   // Process noise variance for the angle
float Q_bias = 0.003;    // Process noise variance for the gyro bias
float R_measure = 0.03;  // Measurement noise variance (from accelerometer)

float angle = 0;         // Estimated angle (output)
float bias = 0;          // Gyro bias estimate
float P[2][2] = { {0, 0}, {0, 0} }; // Error covariance matrix

MPU6050 mpu;
volatile bool mpuInterrupt = false; // Indicates new data is ready
float dt = 0.01; // Sampling time (10ms)

// MPU DMP control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Interrupt Service Routine (runs when MPU6050 has new data)
void dmpDataReady() {
  mpuInterrupt = true;
}

// Kalman Filter Function
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

void setup() {
    Serial.begin(115200);
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    mpu.initialize();

    // Load and configure the DMP
    devStatus = mpu.dmpInitialize();
    
    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-26);
    mpu.setYGyroOffset(-39);
    mpu.setZGyroOffset(-25);
    mpu.setZAccelOffset(9060); // 1688 factory default for my test chip

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        Serial.println("MPU6050 DMP initialized successfully!");
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

void loop() {
    // If programming failed, don't try to do anything
    if (!dmpReady) return;

    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // You can do other processing here if needed
    }

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println("FIFO overflow!");
    } 
    // Otherwise check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02) {
        // Wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Track FIFO count here in case there is > 1 packet available
        fifoCount -= packetSize;

        // Read sensor data
        int16_t gy = mpu.getRotationY();
        int16_t ax = mpu.getAccelerationX();
        int16_t az = mpu.getAccelerationZ();

        // Convert gyro to deg/s (MPU6050 default: 131 LSB/(deg/s))
        float gyroRate = gy / 131.0;  

        // Calculate angle from accelerometer (degrees)
        float accelAngle = atan2(ax, az) * 180 / PI;

        // Apply Kalman filter
        angle = kalmanFilter(accelAngle, gyroRate, dt);

        // Print the estimated angle
        //Serial.print("Estimated Angle: ");
        Serial.println(angle);
    }
}