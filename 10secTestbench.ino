#include <Wire.h>
#include <ICM42688.h>

// Initialize on I2C address 0x68
ICM42688 IMU(Wire, 0x68);

// Testbench Variables
unsigned long testStartTime = 0;
const unsigned long testDuration = 10000; // 10 seconds
unsigned long totalSamples = 0;
bool testFinished = false;

// Timing Variables
const unsigned long intervalMicros = 2000; // 500Hz = 1/500 = 0.002s = 2000us
unsigned long lastSampleMicros = 0;

void setup() {
  Serial.begin(1000000); 
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000); // 400kHz is required to complete the read in time

  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU init failed.");
    while (1);
  }

  // Set ODR to 500Hz
  IMU.setAccelODR(ICM42688::odr500);
  IMU.setGyroODR(ICM42688::odr500);

  Serial.println("--- STARTING 10s POLLING TEST (NO FIFO) ---");
  testStartTime = millis();
  lastSampleMicros = micros();
}

void loop() {
  if (!testFinished) {
    unsigned long currentMicros = micros();

    // Check if it is time to take a sample (every 2000us)
    if (currentMicros - lastSampleMicros >= intervalMicros) {
      lastSampleMicros += intervalMicros; // Keep the rhythm consistent

      // Read the data from the sensor registers
      if (IMU.getAGT() >= 0){
        totalSamples++;
      }; 
    }

    // End test after 10 seconds
    if (millis() - testStartTime >= testDuration) {
      testFinished = true;
      reportResults();
    }
  }
}

void reportResults() {
  float actualSeconds = (millis() - testStartTime) / 1000.0;
  float achievedHz = totalSamples / actualSeconds;
  
  Serial.println("\n--- POLLING PERFORMANCE REPORT ---");
  Serial.print("Total Samples: "); Serial.println(totalSamples);
  Serial.print("Actual Time: "); Serial.print(actualSeconds, 3); Serial.println("s");
  Serial.print("Achieved Rate: "); Serial.print(achievedHz, 2); Serial.println(" Hz");
  
  float integrity = (achievedHz / 500.0) * 100.0;
  Serial.print("Data Health: "); Serial.print(integrity, 2); Serial.println("%");

  if (integrity < 99.5) {
    Serial.println("\n[Note] Polling is sensitive to loop delays.");
    Serial.println("If you add Serial.print inside the 'if', this rate will drop.");
  }
}