#include <Servo.h>

#define x_in A0
#define y_in A5
#define deadzone 25

#define Y_MAX_OFFSET 70 // Left/Right
#define Y_ZERO 90

#define X_MAX_OFFSET 90  // Forward/Reverse
#define X_ZERO 90

#define TIMEOUT_SECONDS 2
#define MANUAL2AUTO_DELAY 3

Servo x_servo; // create servo object to control a servo
const int x_servo_pin = 5;
Servo y_servo; // create servo object to control a servo
const int y_servo_pin = 3;

byte cmd_buffer[2];
unsigned long last_cmd;
unsigned long last_manual_cmd;

void setState(int x, int y) {
  last_cmd = millis();
  x = constrain(x, -100, 100);
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

  x = map(x, -100, 100, X_ZERO - X_MAX_OFFSET, X_ZERO + X_MAX_OFFSET);
  y = map(y, -100, 100, Y_ZERO - Y_MAX_OFFSET, Y_ZERO + Y_MAX_OFFSET);
  
  x_servo.write(x);
  y_servo.write(y);
}

void stop() {
  x_servo.write(X_ZERO);
  y_servo.write(Y_ZERO);
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
       if (millis() - last_manual_cmd >= MANUAL2AUTO_DELAY * 100){
       setState(map(cmd_buffer[0], 0, 127, -100, 100), map(cmd_buffer[1], 0,
       127, -100, 100));}
       } 
     }
   
  if ((millis() - last_cmd) >= 1000 * TIMEOUT_SECONDS) {
    stop();
  }
}
