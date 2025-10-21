#include <esp_now.h>                 // For ESP-NOW to receive wireless communication
#include <WiFi.h>                    // ""
#include <esp_wifi.h>                // ""

// == MOTOR PINS ==  (Control whether a motor spins forward or backward)
// --- Motor A Pins ---
const int motorA_IN1 = 20;
const int motorA_IN2 = 21;
const int motorA_ENA = 19;      // Control speed of the motor using PWM signal
// --- Motor B Pins ---
const int motorB_IN3 = 35;
const int motorB_IN4 = 36;
const int motorB_ENB = 37;     // ""

// Structure to receive data - MUST MATCH THE SENDER
typedef struct struct_message {
    char command;
} struct_message;

struct_message myData;

// --- Motor Control Functions ---
void robotForward(int speed) {
  // Motor A 
  digitalWrite(motorA_IN1, HIGH); 
  digitalWrite(motorA_IN2, LOW);     // set all direction pins to LOW and speed to 0 to stop the motors
  analogWrite(motorA_ENA, speed);
  
  // Motor B
  digitalWrite(motorB_IN3, HIGH); 
  digitalWrite(motorB_IN4, LOW);    //""
  analogWrite(motorB_ENB, speed);
}
void robotBackward(int speed) {
  // Motor A
  digitalWrite(motorA_IN1, LOW);  //""
  digitalWrite(motorA_IN2, HIGH);
  analogWrite(motorA_ENA, speed);
  
  // Motor B
  digitalWrite(motorB_IN3, LOW);  //""
  digitalWrite(motorB_IN4, HIGH);
  analogWrite(motorB_ENB, speed);
}
void robotTurnRight(int speed) {
  // Motor A
  digitalWrite(motorA_IN1, HIGH); 
  digitalWrite(motorA_IN2, LOW);  //""
  analogWrite(motorA_ENA, speed);

  // Motor B
  digitalWrite(motorB_IN3, LOW);   //""
  digitalWrite(motorB_IN4, HIGH);
  analogWrite(motorB_ENB, speed);
}
void robotTurnLeft(int speed) {
  // Motor A 
  digitalWrite(motorA_IN1, LOW);  //""
  digitalWrite(motorA_IN2, HIGH);
  analogWrite(motorA_ENA, speed);
  
  // Motor B
  digitalWrite(motorB_IN3, HIGH); 
  digitalWrite(motorB_IN4, LOW);  //""
  analogWrite(motorB_ENB, speed);
}
void robotStop() {
  // Motor A
  digitalWrite(motorA_IN1, LOW);   //""
  digitalWrite(motorA_IN2, LOW);   //""
  analogWrite(motorA_ENA, 0);

  // Motor B
  digitalWrite(motorB_IN3, LOW);   //""
  digitalWrite(motorB_IN4, LOW);   //""
  analogWrite(motorB_ENB, 0);
}

// Callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t * mac_info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  char command = myData.command;
  Serial.print("Command Received: ");
  Serial.println(command);
  
  int motorSpeed = 255; // Speed is a value from 0-255

  switch (command) {
    case 'F':
      robotForward(motorSpeed);
      break;
    case 'B':
      robotBackward(motorSpeed);
      break;
    case 'L':
      robotTurnLeft(motorSpeed);
      break;
    case 'R':
      robotTurnRight(motorSpeed);
      break;
    case 'S':
      robotStop();
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // --- Setup Motor Pins ---
  
  // Motor A
  pinMode(motorA_IN1, OUTPUT); 
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_ENA, OUTPUT);
  
  // Motor B
  pinMode(motorB_IN3, OUTPUT); 
  pinMode(motorB_IN4, OUTPUT);
  pinMode(motorB_ENB, OUTPUT);
  
  // --- Setup WiFi and ESP-NOW ---
  WiFi.mode(WIFI_STA);
  delay(50); // Small delay to let Wi-Fi initialize

  // Print this car's unique MAC Address
  Serial.print("MAC ADDRESS: ");
  Serial.println(WiFi.macAddress());

  // Force both devices (Controller and Robot) to use Wi-Fi channel 1
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register the callback function that will execute when data is received
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Car Ready to Receive Commands.");
}

void loop() {
  // The loop is empty because all actions are triggered by incoming messages
}