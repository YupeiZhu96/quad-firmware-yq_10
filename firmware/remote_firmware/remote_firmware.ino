#include <radio.h>
char yaw = 0;
char throttle = 0;
char roll = 0;
char pitch = 0;
char p = 0;
bool done = false;
void setup() {
   pinMode(3, OUTPUT); 
   Serial.begin(9600);  // Start up serial
   Serial1.begin(230400);
   rfBegin(21);  // Initialize ATmega128RFA1 radio on channel 11 (can be 11-26)
}

void loop() {
  analogWrite(3, throttle);
  if (rfAvailable()) {
    if(!done) {
      p = rfRead();
      if(p == 'y' || p == 't' || p == 'r' || p == 'p') done = true;
    }
    else{
      char val = rfRead();
      if(p == 'y'){
        yaw = val;
        done = false;
      }
      if(p == 't'){
        throttle = val;
        done = false;
      }
      if(p == 'r'){
        roll = val;
        done = false;
      }
      if(p == 'p'){
        pitch = val;
        done = false;
      }
    }
  }
}
