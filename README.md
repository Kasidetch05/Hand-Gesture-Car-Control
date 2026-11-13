<div align="center">

# Hand Gesture Controlled Car

**Tilt your hand, drive the car.**
A wireless robot car driven by hand tilt, built on two ESP32 boards that talk to each other over **ESP-NOW**.

![Platform](https://img.shields.io/badge/platform-ESP32-E7352C?logo=espressif&logoColor=white)
![Framework](https://img.shields.io/badge/framework-Arduino-00979D?logo=arduino&logoColor=white)
![Protocol](https://img.shields.io/badge/wireless-ESP--NOW-blue)
![Sensor](https://img.shields.io/badge/sensor-MPU6050-orange)
![Status](https://img.shields.io/badge/status-working-brightgreen)

</div>

---


## Overview

The project has two parts:

| Unit | Sketch | Role |
|------|--------|------|
| **Hand controller** (transmitter) | [`Hand_Function/Hand_Function.ino`](Hand_Function/Hand_Function.ino) | Reads hand tilt from an MPU6050, turns it into a drive command, and sends it to the car over ESP-NOW |
| **Car body** (receiver) | [`Car_Bodypart_Function/Car_Bodypart_Function.ino`](Car_Bodypart_Function/Car_Bodypart_Function.ino) | Receives each command and drives two DC motors through an H-bridge motor driver |

ESP-NOW is a peer-to-peer protocol, so you don't need a Wi-Fi router, a pairing step or an internet connection. Commands reach the car with very low delay.

---

## How It Works

```
 ┌──────────────────────────┐        ESP-NOW (Wi-Fi ch. 1)       ┌──────────────────────────┐
 │      HAND CONTROLLER     │  ───────────────────────────────▶  │         CAR BODY         │
 │                          │        1-byte command: F B L R S    │                          │
 │  MPU6050 ──I2C──▶ ESP32  │                                    │  ESP32 ──▶ Motor Driver  │
 │  (accelerometer X / Y)   │                                    │           ──▶ Motor A/B  │
 └──────────────────────────┘                                    └──────────────────────────┘
```

1. The controller reads the MPU6050 accelerometer **10 times per second**.
2. The tilt on the X and Y axes is turned into one command character.
3. A command is sent **only when it changes**, which keeps radio traffic low.
4. The car's receive callback runs the matching motor function right away. The car's `loop()` is empty because everything is event-driven.



### Gesture Map

| Hand gesture | Sensor condition | Command | Car action |
|--------------|------------------|:-------:|------------|
| Tilt forward  | `accY >  0.5` | `F` | Drive forward |
| Tilt backward | `accY < -0.5` | `B` | Drive backward |
| Tilt right    | `accX >  0.5` | `R` | Spin right |
| Tilt left     | `accX < -0.5` | `L` | Spin left |
| Hold flat     | otherwise     | `S` | Stop |

> Forward and backward are checked first, so they win over left and right when the hand is tilted on both axes.

---

## Hardware

| Qty | Component | Notes |
|:---:|-----------|-------|
| 2 | ESP32 dev board | The pin numbers used (GPIO 8/9, 19–21, 35–37) suggest an **ESP32-S3**. |
| 1 | MPU6050 (GY-521) module | Accelerometer and gyroscope on the controller |
| 1 | Dual H-bridge motor driver | L298N or a similar driver with IN1–IN4 and ENA/ENB |
| 2 | DC gear motors + wheels | Differential drive |
| 1 | Robot car chassis | 2WD or 4WD |
| — | Batteries | One supply for the motors and one for the controller ESP32 |
| — | Jumper wires | |

---

## Wiring

### Hand Controller: ESP32 ↔ MPU6050

| MPU6050 | ESP32 |
|---------|-------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO **8** |
| SCL | GPIO **9** |

### Car Body: ESP32 ↔ Motor Driver

| Motor driver | ESP32 | Function |
|--------------|-------|----------|
| IN1 | GPIO **20** | Motor A direction |
| IN2 | GPIO **21** | Motor A direction |
| ENA | GPIO **19** | Motor A speed (PWM) |
| IN3 | GPIO **35** | Motor B direction |
| IN4 | GPIO **36** | Motor B direction |
| ENB | GPIO **37** | Motor B speed (PWM) |
| GND | GND | **Shared ground is required** |

> Power the motors from the driver's own supply, not from the ESP32's 3.3V pin. Connect the grounds of the driver and the ESP32 together.

---

## Software Setup

### Requirements

- [Arduino IDE](https://www.arduino.cc/en/software) 2.x (or the VS Code Arduino extension)
- **esp32 by Espressif Systems** board package, **v3.x**. The sketches use the newer ESP-NOW callback signatures (`esp_now_recv_info_t`, `wifi_tx_info_t`), which older 2.x cores do not have.
- **MPU6050_tockn** library (install it from the Library Manager)

Built-in libraries used: `esp_now.h`, `WiFi.h`, `esp_wifi.h`, `Wire.h`

### Installation

1. **Flash the car first.** Upload `Car_Bodypart_Function.ino` to the car's ESP32.
2. Open the Serial Monitor at **115200 baud** and copy the printed address:
   ```
   MAC ADDRESS: 80:B5:4E:DF:71:24
   ```
3. **Set the car's MAC address in the controller.** In `Hand_Function.ino`, replace the address with your car's:
   ```cpp
   uint8_t broadcastAddress[] = {0x80, 0xB5, 0x4E, 0xDF, 0x71, 0x24};
   ```
4. Upload `Hand_Function.ino` to the controller's ESP32.
5. Power both boards. The Serial Monitor on each side should show:
   - Car: `Car Ready to Receive Commands.`
   - Controller: `MPU6050 connection successful!`, then `Delivery Success` after each command

---

## Configuration

| Setting | Location | Default | Description |
|---------|----------|---------|-------------|
| Tilt threshold | `Hand_Function.ino`, `loop()` | `0.5` g | Lower it for more sensitivity, raise it for a wider stop zone |
| Update rate | `Hand_Function.ino`, `delay(100)` | 10 Hz | How often the gesture is sampled |
| Motor speed | `Car_Bodypart_Function.ino`, `motorSpeed` | `255` | PWM duty from 0 to 255 |
| Wi-Fi channel | Both sketches | `1` | **Must be the same on both boards** |
| Peer MAC | `Hand_Function.ino`, `broadcastAddress` | — | Must match the car's ESP32 |

---

## Troubleshooting

| Symptom | Likely cause and fix |
|---------|----------------------|
| `MPU6050 connection failed!` | Check the SDA/SCL wiring and the 3.3V supply. The I2C address should be `0x68` (AD0 tied low). |
| `Delivery Fail` on the controller | The MAC address is wrong, the car is off, or the boards use different Wi-Fi channels |
| Car moves the wrong way | Swap that motor's two wires, or swap its IN pins in the code |
| Left and right are reversed | Swap the `robotTurnLeft()` and `robotTurnRight()` logic, or mount the MPU6050 the other way round |
| Car twitches near flat | Raise the tilt threshold above `0.5` |
| ESP32 resets when the motors start | Brown-out: give the motors a separate battery and share only the ground |

---

## Project Structure

```
Hand_gesture_control_car/
├── Car_Bodypart_Function/
│   └── Car_Bodypart_Function.ino   # Receiver: ESP-NOW → motor driver
├── Hand_Function/
│   └── Hand_Function.ino           # Transmitter: MPU6050 → ESP-NOW
└── README.md
```

---

## Possible Improvements

- [ ] Proportional speed: map the tilt angle to PWM speed instead of always using 255
- [ ] Diagonal moves, such as forward-left and forward-right
- [ ] Failsafe that stops the car if no packet arrives within a set time
- [ ] Gyroscope calibration and filtering to smooth the input
- [ ] ESP-NOW encryption so only your controller can drive the car

---

## Related Projects

This project is part of my **Arduino Coding Project** robotics collection:

| Project | Description |
|---------|-------------|
| **Hand Gesture Control Car** | *This project.* Hand-tilt control of an ESP32 car over ESP-NOW |
| **Robotic Arm 2** | Servo-driven robotic arm |
| **Servo Arm Robot (Fix Circuit)** | Servo arm robot with a revised circuit |
| **Self-Balancing Robot** | Two-wheeled robot that keeps its balance with an IMU and a control loop |

---

<div align="center">

Built with ESP32, Arduino, and an MPU6050.

</div>
