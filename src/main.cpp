#include <Arduino.h>
#include "ECU.h"

constexpr int BEGIN = 9600;
constexpr int BAUDRATE = 250000;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
ECU mainECU;

void setup() {
  Serial.begin(BEGIN);
  if (mainECU.DEBUG) {
    while(!Serial); // Wait for serial to initialize
  }
  Serial.println("Start");
  if (mainECU.DEBUG) {
      Serial.println("Debug Mode Active");
  }
  
  // set up CAN
  can1.begin();
  can1.setBaudRate(BAUDRATE);

  can2.begin();
  can2.setBaudRate(BAUDRATE);

  mainECU.setCAN(can2, can1);
  mainECU.boot();
  pinMode(19, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  // DEBUG MODE
  if (mainECU.DEBUG) {
    if (CAN_message_t rmsg; can1.read(rmsg)) {
      can1.mailboxStatus();
      can2.mailboxStatus();
    }
    mainECU.run();
  }
  else {
    mainECU.run(); // Default Mode
  }
} 