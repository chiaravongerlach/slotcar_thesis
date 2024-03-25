#include <Servo.h>
#define COMMAND_TIMEOUT 50

Servo controller;
byte angle;
byte myArray[3];
byte* ddata = reinterpret_cast<byte*>(&myArray);
size_t pcDataLen = sizeof(myArray);
bool newData = false;
unsigned long command_timer;

void setup() {
  controller.attach(11);
  Serial.begin(9600);
  angle = 0;
  controller.write(angle);
  command_timer = millis();
}

void loop() {
  checkForNewData();
  if (newData == true) {
    newData = false;
//    response(myArray[0], myArray[1], myArray[2]); //here write the send data
    if(myArray[0] == 255 && myArray[2] == 255){
      angle = myArray[1];
      controller.attach(11);
      controller.write(angle);
      command_timer = millis();
    }
  }

  if(millis() - command_timer > COMMAND_TIMEOUT){
    // turn off servo motor --> regain human control
    controller.detach();
  }
  
}

void checkForNewData () {
  if (Serial.available() >= pcDataLen && newData == false) {
    byte inByte;
    for (byte n = 0; n < pcDataLen; n++) {
      ddata [n] = Serial.read();
    }
    while (Serial.available() > 0) { // now make sure there is no other data in the buffer
      byte dumpByte =  Serial.read();
      Serial.println(dumpByte);
    }
    newData = true;
  }
}

void response(
  byte start_byte, byte angle, byte stop_byte
){
  String data = "[" 
    + String(start_byte) + "," 
    + String(angle) + "," 
    + String(stop_byte) + "]";
  delay(50);
  Serial.println(data);
}
