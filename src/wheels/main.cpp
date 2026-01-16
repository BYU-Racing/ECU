#include <Arduino.h>
#include "temp.h"
#include "speed.h"

#define TEMP_PIN 14
#define SPEED_PIN 10

void setup() {
  pinMode(TEMP_PIN, INPUT);
  pinMode(SPEED_PIN, INPUT);
  Serial.begin(9600);
  while (!Serial) {
    ;
  }
  Serial.println("Setup complete");
}

void loop() {

  Serial.print(temp_sensor_get_temp(TEMP_PIN));
  Serial.print(", ");
  Serial.println(speed_sensor_get_speed(SPEED_PIN));

}
