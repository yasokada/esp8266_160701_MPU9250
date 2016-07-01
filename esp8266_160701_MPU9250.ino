/*
 * v0.2 2016 Jul. 01
 *   - based on MPU6050_DMP6 (6/21/2012 by Jeff Rowberg <jeff@rowberg.net>)
 */

#include "I2Cdev.h"
#include "MPU9150_9Axis_MotionApps41.h"
#include "Wire.h"
#include <ESP8266WiFi.h>
MPU9150 mpu;

#define INTERRUPT_PIN 14  // ESP8266

bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;
uint16_t fifoCount;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void MPU_setup() {
  Wire.begin();
  Wire.setClock(400000L); // 400kHz
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  if (mpu.testConnection()) { // TODO: NG occurred
    Serial.println("MPU connect OK");
  } else {
    Serial.println("MPU connect NG");    
  }

  uint8_t devStatus = mpu.dmpInitialize();

  // for calibration
//  mpu.setXGyroOffset(0);
//  mpu.setYGyroOffset(0);
//  mpu.setZGyroOffset(0);
//  mpu.setZAccelOffset(0);

  if (devStatus == 0) { // OK
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); 
//    uint8_t mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP initialize: OK");
  } else {
    // error
    Serial.println("DMP initialize: NG");
  }
}

void setup() {
  Serial.begin(115200);
  MPU_setup();
}

void loop() {
  uint8_t mpuIntStatus;
  uint8_t fifoBuffer[64];
// uint16_t fifoCount; // declaring fifoCount here causes FIFO overflow

  if (!dmpReady) {
    return;
  }

  while(!mpuInterrupt && fifoCount < packetSize) 
    ;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ( (mpuIntStatus & 0x10) || (fifoCount == 1024) ) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  } else if (mpuIntStatus & 0x02) {
      while(fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
      }

      mpu.getFIFOBytes(fifoBuffer, packetSize);

      fifoCount -= packetSize;

      Quaternion q;           // [w, x, y, z]
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("quat\t");
      Serial.print(q.w);      
      Serial.print("\t");
      Serial.print(q.x);      
      Serial.print("\t");
      Serial.print(q.y);      
      Serial.print("\t");
      Serial.println(q.z);      
  }
}
