#include <Servo.h>

#define TOP_MAX_OFFSET 10 // Left/Right
#define BOT_MAX_OFFSET 10 // Forward/Reverse

#define TOP_MIN 60
#define TOP_ZERO 68
#define TOP_MAX 78

#define BOT_ZERO 128

#define TIMEOUT_SECONDS 3

Servo upper_servo; // create servo object to control a servo
const int upper_servo_pin = 3;
Servo lower_servo; // create servo object to control a servo
const int lower_servo_pin = 5;

byte cmd_buffer[2];
void setLinear(int val){//-100:100
   Serial.print(" Linear: ");
  val = map(val, -100, 100, BOT_ZERO - BOT_MAX_OFFSET, BOT_ZERO + BOT_MAX_OFFSET);
  val = constrain(val, BOT_ZERO - BOT_MAX_OFFSET, BOT_ZERO + BOT_MAX_OFFSET);
   lower_servo.write(val);
   
    Serial.println(val);
  
   delay(50);
   
 }
 void setAngular(int val){
  Serial.print("Angular: ");
  val = map(val, -100, 100, TOP_MIN, TOP_MAX);
  val = constrain(val, TOP_MIN, TOP_MAX);
   upper_servo.write(val);
   Serial.println(val);
   delay(50);
  }
 void setState(int linear, int angular){
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
       setAngular(map(cmd_buffer[0],0,127,-100,100));
       setLinear(map(cmd_buffer[1],0,127,-100,100));
    }
  }
//  for (int i = -100; i <= 100; i+=5){
//    setLinear(i);
//    delay(600);
//    }
//    for (int i = 100; i >= -100; i-=5){
//    setLinear(i);
//    delay(600);
//    }

  if ((millis() - last_cmd) >= 1000 * TIMEOUT_SECONDS){
    stop();
    }
}
