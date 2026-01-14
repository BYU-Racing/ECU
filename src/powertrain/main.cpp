#include <Arduino.h>
#include "flow_sensor.h"
#include "temp_sensor.h"
#include "pressure_sensor.h"

#define TEMP_SENSOR_BEGIN_PIN 14
#define TEMP_SENSOR_END_PIN 15
#define PRESSURE_SENSOR_PIN 16

// Create a Teensy IntervalTimer (hardware timer)
IntervalTimer flowRateTimer;

void setup() {
  Serial.begin(115200);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(TEMP_SENSOR_BEGIN_PIN,INPUT);
  pinMode(PRESSURE_SENSOR_PIN,INPUT);
  flowRateTimer.begin(flowISR, FLOW_SENSOR_PERIOD);
}

void loop() {
  Serial.print("Flow rate L/min: ");
  Serial.print(flowRate());
  Serial.print(" | Temp C: ");
  Serial.print(temp_sensor_get_temp(TEMP_SENSOR_BEGIN_PIN,1));//number is the mode for calculation 1 is polynomial
  Serial.print(" | Pressure ?: ");
  Serial.print(pressure_sensor_get_pressure(PRESSURE_SENSOR_PIN));
  Serial.println("");
  //delay(500);
}