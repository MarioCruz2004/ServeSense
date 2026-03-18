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
volatile bool triggerSend = false;
unsigned long lastInterruptTime = 0;

ICM42688_FIFO IMU(SPI, 10);

// 1. A structure to hold our data
struct __attribute__((packed)) imu_packet_t {
  float ax, ay, az;
  float gx, gy, gz;
  unsigned long time_stamp;
};

// Defining some macros and globals for data buffer 
#define SAMPLE_SIZE sizeof(imu_packet_t)
#define SAMPLES_PER_PACKET 8
uint8_t dataBuffer[SAMPLE_SIZE * SAMPLES_PER_PACKET];
volatile int sampleCount = 0;

unsigned long starting_time;
unsigned long last_sample_micros = 0;
const unsigned long intervalMicros = 200000 ; 

void setupBLE(){
  Serial.println("Configuring Bluetooth");
  // Initializing Bluefruit Hardware Stack
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  // Requesting maximum bandwidth 247 bytes per transmission
  // The laptop requires a bit of time to setup before max bandwidth is available
  // So we're doing a call back function which will trigger max bandwidth after connection is setup
  Bluefruit.Periph.setConnectCallback(connect_callback);
  // Setting Name
  Bluefruit.setName("Feather");
  // Setting up service
  // increasing bluetooth transmission speed in the air
  imuService.begin();

  // Setup Characteristic: 
  // We set MaxLen to 512 so we can pack multiple 28-byte samples.
  txCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  txCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  txCharacteristic.setMaxLen(512); 
  txCharacteristic.begin();

  rxCharacteristic.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  rxCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN); // Open for writing
  rxCharacteristic.setFixedLen(1); // Since we only send 'g'
  
  // 3. Optional: Add a callback to react to the write immediately
  rxCharacteristic.setWriteCallback(receive_callback);
  
  rxCharacteristic.begin();
}

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
  
  // Serial.println("Setting up the button for calibration");
  // The D7 button on the Feather connects to GND when pressed
  // pinMode(BUTTON_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

  Serial.println("Setting up the bluetooth...");
  setupBLE();

  startAdv();

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
  Bluefruit.Advertising.start();
}

uint8_t testValue = 0;

void loop() {
  if (triggerCalibration) {
    runCalibration();
    triggerCalibration = false; // Reset the flag
  }
  if (triggerSend){
      if (Bluefruit.connected()) {
        uint16_t conn_handle = Bluefruit.connHandle();
        BLEConnection* conn = Bluefruit.Connection(conn_handle);

        while (conn->getMtu() < 100) {
          conn->requestMtuExchange(247);
          conn->requestConnectionParameter(6, 12);
          Serial.println("Still at MTU 23... Requesting again...");
          delay(1000); 
          }
        Serial.println("Hit the johnson ayeeee");
        }
      sendData();
      // Serial.println("Trigger 'g' received! Starting IMU...");
      triggerSend = false;
  }
}

void sendData(){
  int num_packets_sent = 0;
  Serial.println("START");
  if (Bluefruit.connected()) {
    const char* startMsg = "START";
    txCharacteristic.notify((uint8_t*)startMsg, strlen(startMsg));
  }
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

      Serial.print(myData.time_stamp);
      Serial.print(" ,");
      Serial.print(myData.ax);
      Serial.print(" ,");
      Serial.print(myData.ay);
      Serial.print(" ,");
      Serial.println(myData.az);
      Serial.print(" ,");
      Serial.print(myData.gx);
      Serial.print(" ,");
      Serial.print(myData.gy);
      Serial.print(" ,");
      Serial.println(myData.gz);      

      // Added lines for filling packet to be send
      // Put into buffer
      uint8_t newSample[SAMPLE_SIZE];
      memcpy(dataBuffer + (sampleCount * SAMPLE_SIZE), &myData, sizeof(myData));
      sampleCount++;

      // If buffer is full (196 bytes) send data
      if (sampleCount >= SAMPLES_PER_PACKET) {
        if (Bluefruit.connected()) {
          txCharacteristic.notify(dataBuffer, sizeof(dataBuffer));
        }
        sampleCount = 0; 
      }

      triggerRead = false; // Reset the flag
      num_packets_sent += 1;
    }
  }
  Serial.println("END");
  if (Bluefruit.connected()) {
    const char* endMsg = "END";
    txCharacteristic.notify((uint8_t*)endMsg, strlen(endMsg));
  }
  return;
}

void connect_callback(uint16_t conn_handle) {
  

  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  
  // 1. Request larger MTU (Max 247)
  conn->requestMtuExchange(247);

  Serial.print("Negotiated MTU: ");
  Serial.println(conn->getMtu());
  
  // 2. Request Data Length Extension (DLE)
  conn->requestDataLengthUpdate();
  
  // 3. Request fastest connection interval (7.5ms)
  conn->requestConnectionParameter(6); // 6 * 1.25ms = 7.5ms

  Serial.print("Connection Interval: ");
  Serial.print(conn->getConnectionInterval() * 1.25);
  Serial.println(" ms");

  conn->requestPHY();

  Serial.print("PHY Speed: ");
  Serial.println(conn->getPHY() == BLE_GAP_PHY_2MBPS ? "2 Mbps" : "1 Mbps");

  Serial.print(millis());
  Serial.println(" ms: Connection established - Requested high-speed parameters");
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
void receive_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  triggerSend = true;
}
