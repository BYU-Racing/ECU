#include "brake.h"
#include <Arduino.h>

#define BRAKE_POT_PIN 18

void setup() {
  pinMode(BRAKE_POT_PIN, INPUT);
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Setup complete");

  int foo = 5;
  float ting = 11;
  int bar = 34;
  int lol = 7;
  Serial.println()
}

void loop() {
  brake_pot_get_angle(BRAKE_POT_PIN);
  //Serial.println(brake_pot_get_angle(BRAKE_POT_PIN));
}
