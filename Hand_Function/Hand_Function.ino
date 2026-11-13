#include <esp_now.h>         // Main library fro ESP-NOW
#include <WiFi.h>            // Basic wifi library which uses in ESP-NOW
#include <esp_wifi.h>        // More advanced wifi library to manually set the wifi channel
#include <Wire.h>            // ESP32 to communicate with devices talks to MPU6050 sensor
#include <MPU6050_tockn.h>   // Library for getting data from MPU6050 accelerometer and gyroscope sensor

// == ESP-NOW SETUP ==
// This is my car robot MAC Address from ESP32. Crucial for controller to know to send data
uint8_t broadcastAddress[] = {0x80, 0xB5, 0x4E, 0xDF, 0x71, 0x24};

// Structure to send data 
typedef struct struct_message {      // Custom variable type to hold multiple data (To hold char command )
    char command;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;       // To store information about the device like MAC address and WIFI channel

// == MPU6050 SENSOR SETUP ==
// Define the I2C pins that are connected from ESP32 to MPU6050
#define MY_SDA 8              
#define MY_SCL 9
MPU6050 mpu6050(Wire);             // Create object name mpu6050 to represent physical sensor chip and tells to communicate using wire I2C

// Callback function that executes when data is sent
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {     // Automatically executed everytime ESP-NOW tries to send message
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"); // Prints comfirmation massage to the Serial Monitor. Tells that if the data Succesfully or failed to connect to the robot 
}

void setup() {
  Serial.begin(115200);        // Communication with the computer

  // == MPU6050 INITIALIZATION ==
  Serial.println("== MPU6050 Controller Setup ==");   // Print messages in case for debugging
  Wire.begin(MY_SDA, MY_SCL);     //  Start I2C communication on the defined pins
  mpu6050.begin();                //  Start the MPU6050 sensor using its library
  delay(1000); // Let sensor stabilize

  Wire.beginTransmission(0x68);           // MPU6050 Address
  if (Wire.endTransmission() == 0) {      // If connection work it prints success message
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed! Check wiring and PINS.");
    while (1); 
  }

  // == ESP-NOW INITIALIZATION ==
  WiFi.mode(WIFI_STA);            // Sets the ESP32's WIFI to Station mode 
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);  // Forces ESP32 to use WIFI channel 1

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);   // This tells after ESP-NOW done sending message to robot run OnDatasent function

  // Register peer (Register the robot with controller)  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);   // Copies the robot MACAddress into peerinfo structure
  peerInfo.channel = 1;     // Sets the communication channel for this specific robot to 1
  peerInfo.encrypt = false; // Allowing the controller to send data to the robot if this steps was successful
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  mpu6050.update();
  
  // Get accelerometer values to detect tilt
  float accY = mpu6050.getAccY();     // forward / backward tilt
  float accX = mpu6050.getAccX();     // Left    / right    tilt
  float accZ = mpu6050.getAccZ();     // up      / down     tilt

  // ===========================================
  // Added code to print gyro(Track how fast each direction changes from controller) and accel(Track direct for robot to go from controller)  numbers
  // ===========================================
  Serial.print(" | Accel X: "); Serial.print(accX);
  Serial.print(" | Accel Y: "); Serial.print(accY);
  Serial.print(" | Accel Z: "); Serial.println(accZ);
  Serial.print(" | Gyro X:  "); Serial.print(mpu6050.getGyroX());
  Serial.print(" | Gyro Y:  "); Serial.print(mpu6050.getGyroY());
  Serial.print(" | Gyro Z:  "); Serial.println(mpu6050.getGyroZ());
  

  char lastCommand = myData.command;   // Saves previous command before figuring out new command
  
  // == TRANSLATE GESTURE TO COMMAND (Number from Accel Y and X that show in Serial Monitor)==
  if (accY > 0.5) {         // Tilted forward
    myData.command = 'F';
  } else if (accY < -0.5) { // Tilted backward
    myData.command = 'B';
  } else if (accX > 0.5) {  // Tilted right
    myData.command = 'R';
  } else if (accX < -0.5) { // Tilted left
    myData.command = 'L';
  } else {                 // Held flat
    myData.command = 'S';
  }

  // Only send the command if it's different from the last one
  if (lastCommand != myData.command) {  // very essential it only sends a message if the command has changed
    Serial.printf("Sending Command: %c\n", myData.command);  // if the command is new it prints the command being sent to the serial monitor
    
    // Send the command via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));  // Send the data structure to the robot's MAC address
  }
  
  delay(100); // Check for new gestures 10 times per second
}