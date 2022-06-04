#include <Servo.h>               // to control ESCs


void setup() {
  Serial.begin(115200);
  Serial.println(F("Booting up..."));
}

void loop() {
  // do nothing
  Serial.println(F("looping..."));
  delay(500);
}
