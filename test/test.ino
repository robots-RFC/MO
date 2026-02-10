#define REMOTEXY_MODE__SOFTSERIAL
#include <SoftwareSerial.h>
#define REMOTEXY_SERIAL_RX 11
#define REMOTEXY_SERIAL_TX 10
#define REMOTEXY_SERIAL_SPEED 9600
#include <RemoteXY.h>

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

char result[11];  // Buffer for string
int p = 0;
int i = 0;
int d = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
bool lastButtonState = false;

void setup() 
{
  RemoteXY_Init(); 
  Serial.begin(115200);
}

void loop() 
{ 
  RemoteXY_Handler();

  if (RemoteXY.button_01 == 1) {

    if ((millis() - lastDebounceTime) > debounceDelay) {

      // p += 1;

      Serial.println("pressed");

      lastDebounceTime = millis();

    }

  }

}