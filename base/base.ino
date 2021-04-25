
#define JOYSTICK_ADDRESS 0x0C
#define CHECK_ADDRESS 0x36
#define CHECK_RESPONSE 0x21
#define GET_ADDRESS 0x46 // 6 bytes

#include <Wire.h>

byte buff[6] = {1, 80, 38, 147, 64, 255};

bool check_requested = false;
bool cmd_requested = false

void receiveEvent(int howMany)
{
  byte x = Wire.read(); //getting from receive FIFO Buffer
  if(x == CHECK_ADDRESS)
  {
    check_requested = true;
  } else if (x == GET_ADDRESS){
    cmd_requested = true;
  }
}

void requestEvent()
{
  if (check_requested && cmd_requested) {
    for (int i = 0; i < 6; i++){
      Wire.write(buff[i]);
    }
    check_requested = false;
    cmd_requested = false;
  } else if (check_requested) {
    Wire.write(CHECK_RESPONSE);
  }
}
void setup()
{
  Serial.begin(9600); // start serial for output

  Wire.begin(JOYSTICK_ADDRESS); // join i2c wheelchair with the JOYSTICK_ADDRESS
  Wire.onRequest(requestEvent);
  Wire.onRecieve(receiveEvent);
}

// void ReadJoystickValues(){
//   Wire.beginTransmission(JOYSTICK_ADDRESS); 
//   Wire.write(CHECK_ADDRESS); 
//   Wire.endTransmission();
//   Wire.requestFrom(JOYSTICK_ADDRESS, 1); // Expecting a 0x21 response always
//   byte c = Wire.read();
//   if(c != CHECK_RESPONSE){} // Some error has occured

//   Wire.beginTransmission(JOYSTICK_ADDRESS); 
//   Wire.write(0x46); 
//   Wire.endTransmission();
//   Wire.requestFrom(JOYSTICK_ADDRESS, 6);
 
//   if(Wire.available()){
//   int i = 0;
//     while(Wire.available()){
//       buff[i++] = Wire.read();   
//   }
//   }
// }
// }

// void ReadSerialValues(){
  // if (Serial.available()){
  //   auto_x = manual_x;
  //   auto_y = manual_y;
  // }
// }

void loop()
{
  //  ReadSerialValues();
  //  ReadJoystickValues();
  // for (int j= 0 ;j < 6; j++){
//      Serial.print(buff[j], HEX);
//      Serial.print(" ");
//      }
   delay(1);
}
