#include <Arduino.h>

#define soc_pin 15

void setup() {
  pinMode(soc_pin, INPUT);

  Serial.begin(9600);
  while (!Serial) {;} // wait for Arduino Serial Monitor to open
  Serial.println("Serial Initialized");
}

void loop() {
  //delay(1000);
  float socVoltage = analogRead(soc_pin);
  Serial.println(socVoltage);
}
