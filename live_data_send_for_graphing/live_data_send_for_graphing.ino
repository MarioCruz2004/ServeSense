/*
 * IMU Serial Streamer
 *
 * Continuously reads gyroscope data from an ICM42688 IMU (via SPI) and streams
 * it to a connected laptop over USB serial at 115200 baud.
 *
 * The ICM42688 is configured at 500Hz and triggers a hardware interrupt on each
 * new sample. The main loop responds to that interrupt, reads gyrX/gyrY/gyrZ
 * and a microsecond timestamp, and prints a single comma-separated line:
 *   timestamp,gx,gy,gz
 *
 * Streams indefinitely from boot — no trigger or handshake required.
 * Libraries: ICM42688 (SPI driver), Wire, bluefruit (included but unused here)
 */


#include <bluefruit.h>
#include "ICM42688.h"
#include "Wire.h"

const int INTERRUPT_PIN = 11;
volatile bool triggerGyroCalibration = false;
volatile bool triggerRead = false;
unsigned long lastInterruptTime = 0;

ICM42688_FIFO IMU(SPI, 10);

struct __attribute__((packed)) imu_packet_t {
  float gx, gy, gz;
  unsigned long time_stamp;
};

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  SPI.begin();
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU init failed!");
    while(1);
  }

  IMU.setAccelODR(ICM42688::odr500);
  IMU.setGyroODR(ICM42688::odr500);
  // Calibrate before attaching interrupt or streaming
  Serial.println("CALIBRATING - keep sensor still...");
  IMU.calibrateGyro();  
  Serial.println("Calibration complete");


  IMU.enableDataReadyInterrupt();
  Serial.println("IMU streaming...");

  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleSPIinterrupt, RISING);

  delay(100);
}

void loop() {
  if (triggerGyroCalibration) {
    runCalibration();
    triggerGyroCalibration = false;
  }

  // attempt: adding this to run the calibration, should work if 'c' is pressed over serial....?
  // if (Serial.available() > 0) {
  //   char cmd = Serial.read();
  //   if (cmd == 'c') {
  //     runCalibration();
  //     //triggerGyroCalibration = true;
  // }

  if (triggerRead) {
    imu_packet_t myData;
    myData.time_stamp = micros();
    IMU.getAGT();
    myData.gx = IMU.gyrX();
    myData.gy = IMU.gyrY();
    myData.gz = IMU.gyrZ();
    

    Serial.print(myData.time_stamp);  Serial.print(",");
    Serial.print(myData.gx);          Serial.print(",");
    Serial.print(myData.gy);          Serial.print(",");
    Serial.println(myData.gz);

    triggerRead = false;
  }
}

void runCalibration() {
  Serial.println("!!! STARTING GYROSCOPE CALIBRATION - KEEP SENSOR STILL !!!");
  int status = IMU.calibrateGyro();
  Serial.println("GYROSCOPE Calibration Complete.");
}

void handleSPIinterrupt() {
  triggerRead = true;
}



