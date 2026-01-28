#include <Arduino.h>
#include "temp.h"
#include "speed.h"

#define TEMP_PIN 11
#define SPEED_PIN 14

// Create a Teensy IntervalTimer (hardware timer)
IntervalTimer wheelSpeedTimer;

void setup() {
  pinMode(TEMP_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  setSpeedPin(SPEED_PIN);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  Serial.println("Setup complete");
  wheelSpeedTimer.begin(speedISR, SPEED_SENSOR_PERIOD);
}

void loop() {

  Serial.print(analogRead(SPEED_PIN));
  Serial.print(", ");
  Serial.println(getSpeed()); 

}