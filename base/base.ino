
#define JOYSTICK_ADDRESS 0x0C
#define CHECK_ADDRESS 0x36
#define CHECK_RESPONSE 0x21
#define GET_ADDRESS 0x46 // 6 bytes

#include <Wire.h>
#include "SoftwareI2C.h"

SoftwareI2C joyi2c;

int manual_x = 0; // Middle Value
int manual_y = 0; // Middle Value
int auto_x = 0; // Middle Value
int auto_y = 0; // Middle Values

byte buff[6] = {0,0,0,0,0,0};
bool manual_mode = true;

void setup()
{
  Serial.begin(9600);           // start serial for output
  Wire.begin(); // join i2c wheelchair with {{JOYSTICK_ADDRESS}}
  //Wire.onRequest(chairRequest); // register event for when wheelchair requests joystick data
  // Wire.onRecieve();
  // joyi2c.begin(3, 2); // sda, scl
}

void chairRequest()
{
  byte response[2];
  if (manual_mode){
    response[0] = manual_x;
    response[1] = manual_y;
  } else {
    response[0] = auto_x;
    response[1] = auto_y;
  }
   Wire.write(response, 2);
}
int a;
byte x;
// https://github.com/rricharz/i2c-sniffer-100kBaud-Arduino-Mega
void ReadJoystickValues(){
  // int x,y,z; //two axis data
  //start the communication with IC with the address xx
  
  // This needs to happen first
  Wire.beginTransmission(JOYSTICK_ADDRESS); 
  Wire.write(0x36); 
  Wire.endTransmission();
  Wire.requestFrom(JOYSTICK_ADDRESS, 1); // Expecting a 0x21 response always
  x = Wire.read();
  
  Wire.beginTransmission(JOYSTICK_ADDRESS); 
  //send a bit and ask for register zero
  Wire.write(0x46); 
  //end transmission
  Wire.endTransmission();
  Wire.requestFrom(JOYSTICK_ADDRESS, 6); // Expecting a 0x21 response
 
  if(Wire.available()){
  int i = 0;
    while(Wire.available()){
      buff[i++] = Wire.read();   
  }

   Serial.println(a);
   i++;
  for (int j= 0 ;j < 6; j++){
     Serial.print(buff[j],HEX);
     Serial.print(" ");
     }
//Serial.print(buff[3]);
//     Serial.print(" ");
//Serial.print(buff[1]);
//     Serial.print(" ");
   Serial.println(" ");
  }
}

void ReadSerialValues(){
  if (Serial.available()){
    auto_x = manual_x;
    auto_y = manual_y;
  }
}

void loop()
{
   ReadSerialValues();
   ReadJoystickValues();
   delay(1);}
