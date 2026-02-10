/*
   -- New project --
   
   This source code of graphical user interface 
   has been generated automatically by RemoteXY editor.
   To compile this code using RemoteXY library 3.1.13 or later version 
   download by link http://remotexy.com/en/library/
   To connect using RemoteXY mobile app by link http://remotexy.com/en/download/                   
     - for ANDROID 4.15.01 or later version;
     - for iOS 1.12.1 or later version;
    
   This source code is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.    
*/

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// можете включить вывод отладочной информации в Serial на 115200
//#define REMOTEXY__DEBUGLOG

// определение режима соединения и подключение библиотеки RemoteXY
#define REMOTEXY_MODE__WIFI_POINT

#include <ESP8266WiFi.h>

// настройки соединения
#define REMOTEXY_WIFI_SSID "mo"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


#include <RemoteXY.h>

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;


// конфигурация интерфейса RemoteXY
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =  // 31 bytes
  { 255, 2, 0, 0, 0, 24, 0, 19, 0, 0, 0, 109, 111, 0, 31, 1, 106, 200, 1, 1,
    1, 0, 5, 23, 61, 60, 60, 32, 177, 26, 31 };

// структура определяет все переменные и события вашего интерфейса управления
struct {

  // input variables
  int8_t joystick_01_x;  // oт -100 до 100
  int8_t joystick_01_y;  // oт -100 до 100

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

#define forward_pin 16
#define backward_pin 14
#define left_pin 12
#define right_pin 13

void setup() {
  RemoteXY_Init();
  pinMode(forward_pin, OUTPUT);
  pinMode(backward_pin, OUTPUT);
  pinMode(left_pin, OUTPUT);
  pinMode(right_pin, OUTPUT);

  while (!Serial) {}
  Serial.begin(115200);
  Wire.begin(4, 5);
  Wire.setClock(400000);  // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1)
      ;
  }

  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);


  // TODO you setup code
}

void loop() {
  RemoteXY_Handler();

  if (RemoteXY.connect_flag) {
    if (RemoteXY.joystick_01_y > 10) {
      if (sensor.read() < 200){
        digitalWrite(forward_pin, LOW);
      } else{
        digitalWrite(forward_pin, HIGH);
      }
    } else {
      digitalWrite(forward_pin, LOW);
    }

    if (RemoteXY.joystick_01_y < -10) {
      if (sensor.read() < 200){
        digitalWrite(backward_pin, HIGH);
      } else{
        digitalWrite(backward_pin, LOW);
      }
    } else {
      digitalWrite(backward_pin, LOW);
    }

    if (RemoteXY.joystick_01_x > 80) {
      digitalWrite(right_pin, HIGH);
    } else {
      digitalWrite(right_pin, LOW);
    }

    if (RemoteXY.joystick_01_x < -80) {
      digitalWrite(left_pin, HIGH);
    } else {
      digitalWrite(left_pin, LOW);
    }

  } else {
    digitalWrite(forward_pin, LOW);
    digitalWrite(backward_pin, LOW);
    digitalWrite(left_pin, LOW);
    digitalWrite(right_pin, LOW);
  }
}