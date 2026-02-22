#include <bluefruit.h>
#include "ICM42688.h" // Standard library for your specific chip
#include "Wire.h"

// Defining the service information that will be broadcasted

BLEService        imuService = BLEService(0x1234); 
BLECharacteristic imuCharacteristic = BLECharacteristic(0x5678);

const int BUTTON_PIN = 7; // D7 is the User Button which we'll be using for calibration
volatile bool triggerCalibration = false;
unsigned long lastInterruptTime = 0;

ICM42688 IMU(Wire, 0x68); // IMU object on I2C address 0x68

// 1. A structure to hold our data
struct __attribute__((packed)) imu_packet_t {
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long time_stamp;
};

unsigned long starting_time;
unsigned long last_sample_micros = 0;
const unsigned long intervalMicros = 200000 ; 

void setup() {
  Serial.begin(115200);

  Serial.println("Setting up the button for calibration");
  // The D7 button on the Feather connects to GND when pressed
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

  Serial.println("Starting up the I2C");
  Wire.begin();

  Serial.println("Configuring Bluetooth Settings");
  // Initialize Bluefruit
  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Max power is +4 dBm
  Bluefruit.setName("Feather-IMU");
  //Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Serial.println("Starting to broadcast service");
  // Setup Service & Characteristic
  imuService.begin();
  imuCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  imuCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  imuCharacteristic.setFixedLen(sizeof(imu_packet_t)); // Max BLE packet size is usually 20-244 bytes
  imuCharacteristic.begin();

  // Start Advertising (makes the board visible to my laptop)
  startAdv();

  // Initializing the sensor
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU init failed!");
    while(1);
  }
  last_sample_micros = micros();
  Serial.println("Streaming at 500Hz...");
}

void startAdv(void) {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(imuService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.start(0); // 0 = Don't stop advertising
}

void loop() {

  if (triggerCalibration) {
    runCalibration();
    triggerCalibration = false; // Reset the flag
  }

  if (Bluefruit.connected()) {
    unsigned long currentMicros = micros();

    // Check this later (my intervalMicros might be funky)

    // if (currentMicros - last_sample_micros >= intervalMicros) {
    //   last_sample_micros += intervalMicros;
    // }

    imu_packet_t myData;

    IMU.getAGT(); 
    myData.ax = IMU.accX();
    myData.ay = IMU.accY();
    myData.az = IMU.accZ();
    myData.gx = IMU.gyrX();
    myData.gy = IMU.gyrY();
    myData.gz = IMU.gyrZ();
    myData.time_stamp = currentMicros;
    imuCharacteristic.notify((uint8_t*)&myData, sizeof(myData));
    delay(intervalMicros/1000); // Send at 5Hz for testing
  }
}

void runCalibration() {
  Serial.println("!!! STARTING CALIBRATION - KEEP SENSOR STILL !!!");

  IMU.begin(); 
  
  Serial.println("Calibration Complete.");
}

// 3. The Interrupt Service Routine (ISR) should be short
void handleButtonPress() {
  unsigned long interruptTime = millis();
  
  // Software Debouncing: ignore interrupts if they happen too fast (within 200ms)
  if (interruptTime - lastInterruptTime > 200) {
    triggerCalibration = true;
  }
  lastInterruptTime = interruptTime;
}

