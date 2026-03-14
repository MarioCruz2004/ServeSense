#include <bluefruit.h>
#include "ICM42688.h" // Standard library for your specific chip
#include "Wire.h"

// Defining the service information that will be broadcasted

BLEService        imuService = BLEService(0x1234); 
BLECharacteristic txCharacteristic = BLECharacteristic(0x5678);
BLECharacteristic rxCharacteristic = BLECharacteristic(0x9ABC); // For Commands

const int BUTTON_PIN = 7; // D7 is the User Button which we'll be using for calibration
const int INTERRUPT_PIN = 11;
volatile bool triggerCalibration = false;
volatile bool triggerRead = false;
unsigned long lastInterruptTime = 0;

ICM42688_FIFO IMU(SPI, 10);

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
  while (!Serial) delay(10);

  SPI.begin();
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU init failed!");
    while(1);
  }

  IMU.setAccelODR(ICM42688::odr500);
	IMU.setGyroODR(ICM42688::odr500);
  IMU.enableDataReadyInterrupt();
  Serial.println("IMU has begun...");
  

  Serial.println("Setting up the button for calibration");

  // The D7 button on the Feather connects to GND when pressed
  // pinMode(BUTTON_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

  Serial.println("Configuring Bluetooth Settings");
  // Initialize Bluefruit
  // Bluefruit.begin();
  // Bluefruit.setTxPower(3); // Max power is +4 dBm
  // Bluefruit.setName("FeatherSS");
  // Bluefruit.configPrphBandwidth(3);
  
  // Bluefruit.Periph.setConnectCallback(connect_callback);
  // Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Serial.println("Starting to broadcast service");

  // // 3. FINALLY start the service
  // imuService.begin();

  // Setup Transmission Characterstic/Service
  // 1. Setup properties
  // txCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  // txCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  // txCharacteristic.setMaxLen(20);
  // txCharacteristic.setCccdWriteCallback(cccd_callback);

  // 2. Add characteristic to the service (this happens internally when you call begin)
  // txCharacteristic.begin(); 

  Serial.println("Setting up the receive characteristic...");

  // Setup Receive Characteristic/Service
  // rxCharacteristic.setProperties(CHR_PROPS_WRITE);
  // rxCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);

  // // This tells the Feather: "Run 'onWriteCallback' when data arrives"
  // rxCharacteristic.setWriteCallback(onWriteCallback);
  // rxCharacteristic.begin();

  // Start Advertising (makes the board visible to my laptop)
  // startAdv();

  Serial.println("Attaching interrupt...");
  
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleSPIinterrupt, RISING);
  
  last_sample_micros = micros();
  delay(100);
  Serial.println("Press 'g' to start data collection...");
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
  if (Serial.available() > 0) {
    char incomingByte = Serial.read(); // Read the key pressed

    if (incomingByte == 'g' || incomingByte == 'G') {
      sendData();
    }
  }
}

void sendData(){
  int num_packets_sent = 0;
  Serial.println("START");
  while (num_packets_sent < 1500){
    if (triggerRead) {
      //Serial.println("Getting the interrupt");
      imu_packet_t myData;
      unsigned long currentMicros = micros();
      IMU.getAGT(); 
      myData.ax = IMU.accX();
      myData.ay = IMU.accY();
      myData.az = IMU.accZ();
      myData.gx = IMU.gyrX();
      myData.gy = IMU.gyrY();
      myData.gz = IMU.gyrZ();
      myData.time_stamp = currentMicros;

      char buffer[50]; 

      Serial.print(myData.time_stamp);
      Serial.print(" ");
      Serial.print(myData.ax);
      Serial.print(" ");
      Serial.print(myData.ay);
      Serial.print(" ");
      Serial.print(myData.az);
      Serial.print(" ");
      Serial.print(myData.gx);
      Serial.print(" ");
      Serial.print(myData.gy);
      Serial.print(" ");
      Serial.println(myData.gz);

      triggerRead = false; // Reset the flag
      num_packets_sent += 1;
    }
  }
  Serial.println("END");
  return;
}

void connect_callback(uint16_t conn_handle) {
  Serial.print(millis());
  Serial.println(" ms: BLE connected");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  Serial.print(millis());
  Serial.print(" ms: BLE disconnected. Reason = 0x");
  Serial.println(reason, HEX);
}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value) {
  Serial.print("CCCD updated on connection ");
  Serial.println(conn_hdl);

  Serial.print("CCCD value: ");
  Serial.println(cccd_value);

  if (chr->notifyEnabled()) {
    Serial.println("Notifications enabled");
  } else {
    Serial.println("Notifications disabled");
  }
}

void runCalibration() {
  Serial.println("!!! STARTING CALIBRATION - KEEP SENSOR STILL !!!");

  IMU.begin(); 
  
  Serial.println("Calibration Complete.");
}

// The Interrupt Service Routine (ISR) should be short
void handleButtonPress() {
  unsigned long interruptTime = millis();
  
  // Software Debouncing: ignore interrupts if they happen too fast (within 200ms)
  if (interruptTime - lastInterruptTime > 200) {
    triggerCalibration = true;
  }
  lastInterruptTime = interruptTime;
}

void handleSPIinterrupt(){
  triggerRead = true;
}

// The Callback Function
void onWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  // 'data' is an array of bytes sent from your laptop
  // 'len' is how many bytes were sent
  
  if (len > 0) {
    uint8_t command = data[0]; // Look at the first byte
    
    if (command == 1) {
      Serial.println("Remote command: Start Calibration!");
      triggerCalibration = true; // Set the flag we used for the button
    } 
    else if (command == 0) {
      Serial.println("Remote command: Stop Streaming!");
      // Logic to stop streaming
    }
  }
}
