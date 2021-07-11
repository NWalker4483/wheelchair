#include <Wire.h>
void requestEvent();
byte lastRecieved = 0;
int lastRecieveLength = 0;
volatile bool initialized = false;
volatile bool startup_delay = false;

void setup() {
  Wire.begin(12);               // 0C          // join i2c bus with address 0C
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
  Serial.begin(9600);           // start serial for
}
void loop() {
  if (startup_delay) {
    delay(4);
    Wire.write(128); // 80
    initialized = true;
    startup_delay = false;
  }
}

void receiveEvent(int numBytes) {
  lastRecieveLength = numBytes;
  if (numBytes == 1) {
    lastRecieved = Wire.read();
  } else {
    lastRecieved = 0;
  }
  while (Wire.available() > 0) {
    Wire.read();
  }
}
void requestEvent() {
  if (initialized) {
    Wire.write(255); // FF
    lastRecieved = 0;
    lastRecieveLength = 0;
    initialized = false;
  }
  if (lastRecieveLength == 1) {
    if (lastRecieved == 208) { // D0
      Wire.write(03);          // 03
    }
    if (lastRecieved == 128) { // 80
      Wire.write(03);          // 03
    }
    if (lastRecieved == 240) { // F0
      Wire.write(04);          // 04
      Wire.write(00);          // 00
      startup_delay = true;
    }
    if (lastRecieved == 54) { // 36
      Wire.write(01);         // 00 Blah
    }
    if (lastRecieved == 70) { // 46
      Wire.write(33);         // 21
    }
  } else if (lastRecieveLength == 2) {
    Wire.write(00); // 00
  } else if (lastRecieveLength == 4) {
    Wire.write(03); // 03
  }
}