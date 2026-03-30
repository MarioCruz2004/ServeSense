/* 

NOTE: current version just prints out the accel values calculated 
during each position over serial. I want to change this later 
so that it writes the info to a file so that I can then feed that file 
into other programs etc 

*/

/*
 * Accelerometer Calibration Sketch
 *
 * This sketch performs a full six-position accelerometer calibration on the
 * ICM42688 IMU. It must be run once to characterize your specific hardware.
 * The calibration values are stored internally in the IMU library object and
 * applied automatically to all subsequent accX(), accY(), accZ() readings.
 *
 * How it works:
 *   The ICM42688 library's calibrateAccel() function collects a batch of
 *   samples and averages them to get a stable reading for the current
 *   orientation. It records the result as a min or max for whichever axis
 *   is currently aligned with gravity (i.e. reads close to +/- 1g). Once
 *   both a min and max have been recorded for an axis, the library computes
 *   a bias (offset) and scale factor for that axis automatically.
 *
 *   This sketch orchestrates the six required positions interactively:
 *     +X up, -X up, +Y up, -Y up, +Z up, -Z up
 *   For each position it prompts you over Serial, waits for you to send any
 *   character to confirm the IMU is held still, then calls calibrateAccel().
 *
 * After all six positions are complete, the sketch prints a verification
 * block showing the corrected readings in each orientation. Ideally each
 * axis should read close to +/-1.0g on the axis aligned with gravity and
 * close to 0.0g on the other two.
 *
 * Usage:
 *   1. Upload this sketch to the Feather.
 *   2. Open the Serial Monitor at 115200 baud.
 *   3. Follow the on-screen prompts, placing the IMU flat on each face.
 *   4. Note the printed calibration values at the end -- you will need to
 *      hardcode these into your main streaming sketch.
 *
 * Libraries: ICM42688 (SPI driver), Wire, SPI
 */

#include "ICM42688.h"
#include "Wire.h"
#include <SPI.h>

ICM42688 IMU(SPI, 10);

// The six calibration positions in human-readable order
const char* POSITION_LABELS[] = {
  "+X UP  (right side facing up)",
  "-X UP  (left side facing up)",
  "+Y UP  (top edge facing up)",
  "-Y UP  (bottom edge facing up)",
  "+Z UP  (flat, component side up)",
  "-Z UP  (flat, component side down)"
};
const int NUM_POSITIONS = 6;

// Wait for the user to send any character over Serial
void waitForUser() {
  while (Serial.available()) Serial.read(); // flush any existing input
  while (!Serial.available());              // block until input arrives
  while (Serial.available()) Serial.read(); // flush the received character
}

// Print a formatted reading of all three accel axes
void printAccel() {
  IMU.getAGT();
  Serial.print("  accX="); Serial.print(IMU.accX(), 4);
  Serial.print("  accY="); Serial.print(IMU.accY(), 4);
  Serial.print("  accZ="); Serial.println(IMU.accZ(), 4);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  SPI.begin();
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU init failed! Check wiring.");
    while (1);
  }

  Serial.println("===========================================");
  Serial.println("   ICM42688 Accelerometer Calibration");
  Serial.println("===========================================");
  Serial.println();
  Serial.println("You will be prompted to place the IMU in");
  Serial.println("six orientations one at a time.");
  Serial.println("Hold the IMU completely still on each face,");
  Serial.println("then send any character to capture that position.");
  Serial.println();
  Serial.println("Send any character to begin...");
  waitForUser();

  // --- Calibration loop ---
  for (int i = 0; i < NUM_POSITIONS; i++) {
    Serial.println();
    Serial.print("Position "); Serial.print(i + 1);
    Serial.print(" of "); Serial.print(NUM_POSITIONS);
    Serial.print(": Place IMU with ");
    Serial.println(POSITION_LABELS[i]);
    Serial.println("Hold still, then send any character to capture...");

    waitForUser();

    Serial.println("Capturing samples...");
    int result = IMU.calibrateAccel();

    if (result < 0) {
      Serial.print("  ERROR during calibration (code ");
      Serial.print(result);
      Serial.println("). Check IMU connection and retry.");
    } else {
      Serial.println("  Captured. Current corrected reading:");
      printAccel();
    }
  }

  // --- Print final results ---
  Serial.println();
  Serial.println("===========================================");
  Serial.println("   Calibration Complete");
  Serial.println("===========================================");
  Serial.println("Verification readings (gravity axis should");
  Serial.println("be close to +/-1.0, others close to 0.0):");
  Serial.println();

  for (int i = 0; i < NUM_POSITIONS; i++) {
    Serial.print(POSITION_LABELS[i]); Serial.println(":");
    printAccel();
    delay(100);
  }

  Serial.println();
  Serial.println("These calibration values can now be copied into other files for filter implementation etc.");
  Serial.println("Done.");
}

void loop() {
  // Nothing to do after calibration is complete
}
