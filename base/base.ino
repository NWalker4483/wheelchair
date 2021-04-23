
#define JOYSTICK_ADDRESS 0
#define X_ADDRESS 0
#define Y_ADDRESS 0

#include <Wire.h>
#include "SoftwareI2C.h"


SoftwareI2C joyi2c;

int manual_x = 0; // Middle Value
int manual_y = 0; // Middle Value
int auto_x = 0; // Middle Value
int auto_y = 0; // Middle Values

bool manual_mode = true;

void setup()
{
  Serial.begin(9600);           // start serial for output
  Wire.begin(JOYSTICK_ADDRESS); // join i2c wheelchair with {{JOYSTICK_ADDRESS}}
  Wire.onRequest(chairRequest); // register event for when wheelchair requests joystick data
  
  joyi2c.begin(3, 2); // sda, scl
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

void ReadJoystickValues(){
  manual_y;
  manual_x;
  joyi2c.requestFrom(JOYSTICK_ADDRESS, 6);
  while (joyi2c.available()){
    Serial.println(joyi2c.read());
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
}
