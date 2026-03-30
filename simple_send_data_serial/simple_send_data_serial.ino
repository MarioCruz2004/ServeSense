/*
 * IMU Serial Streamer
 *
 * Continuously reads gyroscope and accelerometer data from an ICM42688 IMU
 * (via SPI) and streams it to a connected laptop over USB serial at 115200 baud.
 *
 * The ICM42688 is configured at 500Hz and triggers a hardware interrupt on each
 * new sample. The main loop responds to that interrupt, reads gyrX/gyrY/gyrZ,
 * accX/accY/accZ, and a microsecond timestamp, then prints a single
 * comma-separated line:
 *   timestamp,gx,gy,gz,ax,ay,az
 *
 * Gyroscope bias is removed automatically by IMU.calibrateGyro() at startup.
 * Keep the sensor still for ~1 second after powering on while calibration runs.
 *
 * Streams indefinitely from boot — no trigger or handshake required.
 * Libraries: ICM42688 (SPI driver), Wire
 */

#include <bluefruit.h>
#include "ICM42688.h"
#include "Wire.h"

const int INTERRUPT_PIN = 11;
volatile bool triggerRead = false;

ICM42688_FIFO IMU(SPI, 10);

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

  // Calibrate gyro bias before streaming — keep sensor still during this
  Serial.println("Calibrating gyro — keep sensor still...");
  IMU.calibrateGyro();
  Serial.println("Calibration complete. Streaming...");

  IMU.enableDataReadyInterrupt();
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleSPIinterrupt, RISING);

  delay(100);
}

void loop() {
  if (triggerRead) {
    unsigned long ts = micros();
    IMU.getAGT();

    Serial.print(ts);           Serial.print(",");
    Serial.print(IMU.gyrX());   Serial.print(",");
    Serial.print(IMU.gyrY());   Serial.print(",");
    Serial.print(IMU.gyrZ());   Serial.print(",");
    Serial.print(IMU.accX());   Serial.print(",");
    Serial.print(IMU.accY());   Serial.print(",");
    Serial.println(IMU.accZ());

    triggerRead = false;
  }
}

void handleSPIinterrupt() {
  triggerRead = true;
}
