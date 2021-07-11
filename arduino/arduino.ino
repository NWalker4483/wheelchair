#include <Servo.h>

#define TOP_MAX_OFFSET 10 // Left/Right
#define BOT_MAX_OFFSET 10 // Forward/Reverse

#define BOT_MIN 110
#define BOT_ZERO 128
#define BOT_MAX 130

#define TOP_MIN 60
#define TOP_ZERO 68
#define TOP_MAX 78

#define TIMEOUT_SECONDS 2

Servo upper_servo; // create servo object to control a servo
const int upper_servo_pin = 3;
Servo lower_servo; // create servo object to control a servo
const int lower_servo_pin = 5;
// ! Temp
/*
Front of chair
 \  |  /
 <- * ->
 /  |  \
*/
int states[3][3][2] = {{{120,64},{110,68},{120,76}},
                    {{128,64},{BOT_ZERO, TOP_ZERO},{128,74}},
                    {{110,64},{134,68},{134,74}}};

byte cmd_buffer[2];
 void setAngular(int val){
  Serial.print("Angular: ");
  val = map(val, -100, 100, TOP_ZERO - TOP_MAX_OFFSET, TOP_ZERO + TOP_MAX_OFFSET);
  val = constrain(val, TOP_MIN, TOP_MAX);
   upper_servo.write(val);
   Serial.println(val);
   delay(50);
  }
 void setLinear(int val){
  Serial.print("Linear: ");
  val = map(val, -100, 100, BOT_ZERO - BOT_MAX_OFFSET, BOT_ZERO + BOT_MAX_OFFSET);
  val = constrain(val, BOT_MIN, BOT_MAX);
   lower_servo.write(val);
   Serial.println(val);
   delay(50);
  }

 void setState(int linear, int angular ){
  int x, y;
  x = 0; 
  y = 0; 
  if (abs(linear) <= 10){y = 1;}
  else if (linear > 0){y = 0;}
  else if (linear < 0){y = 2;}
  if (abs(angular) <= 10){x = 1;}
  else if (angular > 0){x = 2;}
  else if (angular < 0){x = 0;}
  Serial.print("Linear: ");
  Serial.print(states[y][x][0]);
  Serial.print(" ");
   Serial.print("Angular: ");
  Serial.println(states[y][x][1]);
//  Serial.println(linear);
//  Serial.println(angular);
  lower_servo.write(states[y][x][0]);
  upper_servo.write(states[y][x][1]);
  }
  
void stop(){
  upper_servo.write(TOP_ZERO);
  lower_servo.write(BOT_ZERO);
  delay(100);
}
  
void setup() {
  Serial.begin(9600);
  upper_servo.attach(upper_servo_pin);
  lower_servo.attach(lower_servo_pin);
  upper_servo.write(TOP_ZERO);
  lower_servo.write(BOT_ZERO);
  delay(100);
}

unsigned long last_cmd;
void loop() {
  if (Serial.available() >= 3) // Full Command is in buffer including start byte
  {
    byte strt = Serial.read();
    if (strt == '#') // Check Start Byte
    {
      last_cmd = millis();
      for (int i = 0; i < 2; i++) { // Read Command Array Bytes
        cmd_buffer[i] = Serial.read();
      }
//      setAngular(map(cmd_buffer[0],0,127,-100,100));
//      setLinear(map(cmd_buffer[1],0,127,-100,100));
      setState(map(cmd_buffer[0],0,127,-100,100), map(cmd_buffer[1],0,127,-100,100));
    }
  }
  if ((millis() - last_cmd) >= 1000 * TIMEOUT_SECONDS){
    stop();
    }
}
