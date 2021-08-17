#include <Servo.h>

#define x_in A0
#define y_in A5
#define deadzone 25

// TOP
#define Y_DEADZONE 10 // NotImplemented
#define Y_MAX 2200  // Forward/Reverse
#define Y_ZERO 1700
#define Y_MIN 1100  // Forward/Reverse
// BOTTOM
#define X_DEADZONE 10 // NotImplemented
#define X_MAX 2400 // Left/Right
#define X_ZERO 1700
#define X_MIN 1000 // Left/Right

#define CONFIG 0

#define TIMEOUT_SECONDS 2
#define MANUAL2AUTO_DELAY 3

Servo x_servo; // create servo object to control a servo
const int x_servo_pin = 3;
Servo y_servo; // create servo object to control a servo
const int y_servo_pin = 5;

byte cmd_buffer[2];
unsigned long last_cmd;
unsigned long last_manual_cmd;

void setState(int y, int x) {
  last_cmd = millis();
  x = constrain(-x, -100, 100);
  y = constrain(y, -100, 100);

  float magnitude = sqrt((x * x) + (y * y));
  magnitude = constrain(magnitude, 0, 100); // TBD

  float x_norm, y_norm;
  x_norm = abs(x) / 100.0;
  y_norm = abs(y) / 100.0;

  x_norm *= x > 0 ? 1.0 : -1.0;
  y_norm *= y > 0 ? 1.0 : -1.0;

  x = x_norm * magnitude;
  y = y_norm * magnitude;

  if (x > 0){
  x = map(x, 0, 100, X_ZERO, X_MAX);
  } else {
     x = map(x, 0, -100, X_ZERO, X_MIN);
  }
  if (y > 0){
  y = map(y, 0, 100, Y_ZERO, Y_MAX);
  } else {
    y = map(y, 0, -100, Y_ZERO, Y_MIN);
  }
  
  x_servo.writeMicroseconds(x);
  y_servo.writeMicroseconds(y);
}

void stop() {
  x_servo.writeMicroseconds(X_ZERO);
  y_servo.writeMicroseconds(Y_ZERO);
  delay(100);
}

void setup() {
  Serial.begin(9600);
  pinMode(x_in, INPUT);
  pinMode(y_in, INPUT);
  x_servo.attach(x_servo_pin);
  y_servo.attach(y_servo_pin);
  stop();
}

void loop() {
  while (CONFIG){
    int _delay = 1000;
    setState(100,0);
    delay(_delay);
    setState(0,0);
    delay(_delay / 2);
    setState(-100,0);
    delay(_delay);
    setState(0,0);
    delay(_delay / 2);
    setState(0,100);
    delay(_delay);
    setState(0,0);
    delay(_delay / 2);
    setState(0,-100);
    delay(_delay);
    setState(0,0);
    delay(2 * _delay);
    }
  
  int X, Y;
  X = map(analogRead(x_in), 0, 1024, -100, 100);
  Y = map(analogRead(y_in), 0, 1024, -100, 100);
  if (abs(X) >= deadzone || abs(Y) >= deadzone) {
    // last_manual_cmd = millis();
    // setState(X,Y);
  } else if (millis() - last_manual_cmd < MANUAL2AUTO_DELAY * 1000){
    stop();
  }
  
   if (Serial.available() >= 3) // Full Command is in buffer including start
   {
     byte strt = Serial.read();
     if (strt == '#') // Check Start Byte
     {
       for (int i = 0; i < 2; i++) { // Read Command Array Bytes
         cmd_buffer[i] = Serial.read();
       }
       if (millis() - last_manual_cmd >= MANUAL2AUTO_DELAY * 1000){
       setState(map(cmd_buffer[0], 0, 127, -100, 100), map(cmd_buffer[1], 0,
       127, -100, 100));}
       } 
     }
   
  if ((millis() - last_cmd) >= 1000 * TIMEOUT_SECONDS) {
    stop();
  }
}
