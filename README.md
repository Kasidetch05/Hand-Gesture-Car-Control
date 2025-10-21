# Hand-Gesture-Car-Control
---
Introduction.
This project details a wireless gesture-controlled car using two ESP32 microcontrollers communicating via ESP-NOW. The controller ESP32 uses an MPU6050 accelerometer/gyroscope to detect hand tilt gestures (forward, backward, left, right, stop). It translates these into movement commands (F, B, L, R, S) and transmits them wirelessly. The robot car receiver ESP32 receives the commands and controls two DC motors to drive, reverse, turn, or stop the car accordingly. It integrates wireless communication, motion sensing, and motor control on the ESP32 platform.
---
Hardware Needed.

Controller (Transmitter)
- 1× ESP32 board
- 1× MPU6050 sensor (accelerometer + gyroscope)
- 1× Breadboard 
- 1× USB cable (for power and uploading code)
- 6× Jumper wires (male–female)

Car Robot (Receiver)
- 1× ESP32 board
- 1× L298N or L293D motor driver module
- 2× DC motors with wheels
- 1× Robot chassis (to hold motors and electronics)
- 1× Battery pack (7.4V–12V)
- Several jumper wires
- 1× USB cable (for power and programming)
---
Wiring: For Controller and Car Robot
1) Controller Connections
- MPU6050 VCC → ESP32 3.3V
- MPU6050 GND → ESP32 GND
- MPU6050 SDA → ESP32 GPIO 8
- MPU6050 SCL → ESP32 GPIO 9
- ESP32   USB → Computer (for power and programming)

2) Car Robot
Connections:
- ESP32   → Motor Driver (L298N / L293D)
- GPIO 20 → IN1
- GPIO 21 → IN2
- GPIO 19 → ENA (controls speed of Motor A)
- GPIO 35 → IN3
- GPIO 36 → IN4
- GPIO 37 → ENB (controls speed of Motor B)

Motor Driver to Motors:
- OUT1 & OUT2 → Motor A terminals
- OUT3 & OUT4 → Motor B terminals

Power Connections:
- Motor Driver VCC → Battery (+) (7.4V–12V)
- Motor Driver GND → Battery (−)
- ESP32 GND        → Motor Driver GND (common ground)
---
Software and library Uses:
1) Software Required
- Arduino IDE (for coding and uploading to ESP32)
- ESP32 Board Package (installed via Arduino Board Manager)
- USB Driver for ESP32 board

2) Library Uses:
Controller (ESP32 + MPU6050)
- esp_now.h       → Enables ESP-NOW wireless communication between ESP32 boards
- WiFi.h          → Basic Wi-Fi functions required for ESP-NOW setup
- esp_wifi.h      → Used to manually set the Wi-Fi channel
- Wire.h          → For I2C communication between ESP32 and MPU6050 sensor
- MPU6050_tockn.h → Library for reading accelerometer and gyroscope data from the MPU6050

Robot Car (ESP32 Receiver)
- esp_now.h   → To receive commands via ESP-NOW
- WiFi.h      → Required for ESP-NOW initialization
- esp_wifi.h  → Sets communication channel for reliable pairing
---
Calibration and Tuning Process:

MPU6050 Sensor:
- Place the controller flat and still.
- Open Serial Monitor (115200) to check sensor values.
- AccX and AccY should be around 0 when flat.
- Adjust tilt thresholds in code if needed:

  if (accY > 0.5) → Forward
  if (accY < -0.5) → Backward
  if (accX > 0.5) → Right
  if (accX < -0.5) → Left

- Fine tune values (e.g., 0.4, 0.6) for smoother control.

Motor Check:
- Test each direction (F, B, L, R, S).
- Swap motor wires if direction is wrong.
- Adjust speed in code:

  int motorSpeed = 200; // Range: 0–255

Tuning:
- If too sensitive → increase delay in loop.
- If too slow      → raise motor speed or lower tilt threshold.
- Re-test until robot moves naturally when the hand tilt.
---
Key Control Variables:

Controller (Transmitter):
- accX, accY, accZ       → Accelerometer readings from MPU6050
- myData.command         → Character command sent to robot (F, B, L, R, S)
- lastCommand            → Stores the previous command to avoid repeating same data
- broadcastAddress[]     → MAC address of the robot ESP32
- Tilt thresholds (±0.5) → Controls how much tilt is needed to trigger movement

Robot (Receiver):
- myData.command                     → Command received from controller
- motorSpeed                         → Motor speed value (0–255)
- motorA_IN1, motorA_IN2, motorA_ENA → Pins for Motor A direction & speed
- motorB_IN3, motorB_IN4, motorB_ENB → Pins for Motor B direction & speed


Main Control Logic:
- Tilt Forward  → accY > 0.5  → Send 'F'
- Tilt Backward → accY < -0.5 → Send 'B'
- Tilt Right    → accX > 0.5  → Send 'R'
- Tilt Left     → accX < -0.5 → Send 'L'
- Flat          → Send 'S' (Stop)
---
Important Code Notes and Tips:

Controller Side:
- Only send commands when changed

if (lastCommand != myData.command) { esp_now_send(...); }

this prevents flooding the robot with repeated messages.

- Tilt thresholds matter (±0.5) adjust these values to match the hand sensitivity.
- Always check serial monitor for accX, accY, accZ and gyro readings to debug gestures.
- Delay in loop

  delay(100);

- Controls gesture update frequency (~10 times/sec). Increase if movement is too jittery.


Robot Side:
- Motor direction pins must match wiring check IN1/IN2/IN3/IN4 and swap wires if motors spin the wrong way.
- Motor speed (0–255) can be tuned for smooth or fast motion:

  int motorSpeed = 200;

- All actions triggered by ESP-NOW callback OnDataRecv() handles movement automatically; no need to poll in loop().
- Make sure ESP32 GND and motor driver GND are connected to avoid erratic motor behavior.

More Tips:
- Keep ESP32 and robot on same Wi-Fi channel (esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE)) for reliable ESP-NOW communication.
- Test each component separately before full integration:
- Sensor readings  → controller
- Motor directions → receiver
- Wireless sending/receiving
- Optional smoothing: Average multiple sensor readings to reduce jitter.
- Debugging: Print myData.command on controller and receiver to ensure commands match.
---
Challenges Faced:
- ESP32 board damaged due to power overload from an unsuitable power supply.
- Using the wrong ESP32 board type caused compilation and communication issues.
- Incorrect or incompatible libraries prevented the code from functioning properly.

Improvement:
- Improve the robot car’s appearance—currently built from electronics waste to save cost.
- Use more powerful motors to increase speed and performance.
- Upgrade to a better controller system for faster and more accurate response.
- Optimize controller-car tuning for smoother and more reliable movement.
---












