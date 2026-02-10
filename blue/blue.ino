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
#define REMOTEXY_WIFI_SSID "MO"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


#include <RemoteXY.h>

// конфигурация интерфейса RemoteXY  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 29 bytes
  { 255,2,0,0,0,22,0,19,0,0,0,0,31,1,106,200,1,1,1,0,
  5,23,73,60,60,32,177,26,31 };
  
// структура определяет все переменные и события вашего интерфейса управления 
struct {

    // input variables
  int8_t joystick_01_x; // oт -100 до 100
  int8_t joystick_01_y; // oт -100 до 100

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)
 
/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////


int forward = 183;
int turn = 180;

void setup() 
{
  RemoteXY_Init (); 

  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  
  
  // TODO you setup code
  
}

void loop() 
{ 
  RemoteXY_Handler ();

  if (RemoteXY.connect_flag){
    if (RemoteXY.joystick_01_y > 10){
      analogWrite(6, 185);
    } else {
      analogWrite(6, 183);
    }
  }
  
  
  // TODO you loop code
  // используйте структуру RemoteXY для передачи данных
  // не используйте функцию delay(), вместо нее используйте RemoteXY_delay() 


}