#include <LMotorController.h>  // Include the motor controller library

// Define motor control pins

int ENA = 3;
int IN1 = 4;
int IN2 = 5;
int IN3 = 8;
int IN4 = 7;
int ENB = 6;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, 1, 1);

int num = 0;


// Instantiate the motor controller

void setup() {

  // Initialize the motor controller

  motorController.stopMoving();  // Ensure the motor is stopped initially

  Serial.begin(9600);
}

void loop() {

  // Turn on the motor at a defined speed

  motorController.move(num,70);  // Move at full speed (255) with minimum absolute speed of 20
  Serial.println(num);

  num += 1;

  delay(200);  // Wait for 2 seconds before restarting
}
