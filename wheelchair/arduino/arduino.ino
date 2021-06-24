#include <Servo.h>

#define TOP_MAX_OFFSET 10 // Left/Right
#define BOT_MAX_OFFSET 15 // Forward/Reverse
#define TOP_ZERO 68
#define BOT_ZERO 128

#define TIMEOUT_SECONDS 3

Servo upper_servo; // create servo object to control a servo
const int upper_servo_pin = 3;
Servo lower_servo; // create servo object to control a servo
const int lower_servo_pin = 5;

byte cmd_buffer[2];

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
    if (Serial.read() == '#') // Check Start Byte
    {
      last_cmd = millis();
      for (int i = 0; i < 2; i++) { // Read Command Array Bytes
        cmd_buffer[i] = Serial.read();
      }
       upper_servo.write(constrain(map(cmd_buffer[0],0,127,TOP_ZERO - TOP_MAX_OFFSET, TOP_ZERO + TOP_MAX_OFFSET), TOP_ZERO - TOP_MAX_OFFSET, TOP_ZERO + TOP_MAX_OFFSET));
       delay(10);
       lower_servo.write(constrain(map(cmd_buffer[1],127,0,BOT_ZERO - BOT_MAX_OFFSET, BOT_ZERO + BOT_MAX_OFFSET), BOT_ZERO - BOT_MAX_OFFSET, BOT_ZERO + BOT_MAX_OFFSET));
      Serial.print(cmd_buffer[0]);
      Serial.print(" ");
      Serial.println(cmd_buffer[1]);
    }
  }
  if ((millis() - last_cmd) >= 1000 * TIMEOUT_SECONDS){
    stop();
    }
}
