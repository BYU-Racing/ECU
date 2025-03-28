#include <Arduino.h>
#include "ECU.h"

constexpr int BEGIN = 9600;
constexpr int BAUDRATE = 250000;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
ECU mainECU;

void setup() {
  Serial.begin(BEGIN);
  Serial.println("Start");

  // set up CAN
  can1.begin();
  can1.setBaudRate(BAUDRATE);

  can2.begin();
  can2.setBaudRate(BAUDRATE);

  mainECU.setCAN(can2, can1);
  mainECU.boot();

  //Set output pins for BL/HORN
  pinMode(16, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(13, OUTPUT);
  delay(100);
  digitalWrite(13, HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  mainECU.run();

  //BELOW VERIFIES MC CONNECTION
  // CAN_message_t msg;
  // msg.id = 0x0C1;

  // msg.buf[0] = 141;
  // msg.buf[1] = 0;
  // msg.buf[2] = 0;
  // msg.buf[3] = 0;
  // msg.buf[4] = 0;
  // msg.buf[5] = 0;
  // msg.buf[6] = 0;
  // msg.buf[7] = 0;

  // can1.write(msg);
  // can2.write(msg);

  // unsigned long long previousUpdate = millis();
  // while (millis() - previousUpdate <= 500) {

  //   if(can1.read(msg)) {
  //     if(msg.id == 0x0C2) {
  //       Serial.println("MOTOR RESPONSE HEARD 1");
  //     }
  //   }
  //   if(can2.read(msg)) {
  //     if(msg.id == 0x0C2) {
  //       Serial.println("MOTOR RESPONSE HEARD 2");
  //     }
  //   }
  // }
  // Serial.println("LOOP RESTART");
}
