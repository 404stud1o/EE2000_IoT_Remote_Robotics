# Remote Control Object Grabbing Robotic Arm for EE2000 Embedded System Team Project

## Microcontroller
This project consist the use of the Joy-It NodeMCU-ESP32 development kit, which uses the ESP32-WROOM-32 microcontroller chip module. 

## Components
| Component                          | Quantity |
| ---------------------------------- | -------- |
| Joy-It NodeMCU-ESP32               |    1     |
| 9V PP3 Alkaline Battery            |    1     |
| TS7805 5V DC Voltage Regulator     |    1     |
| PP3 Battery Snap                   |    1     |
| Miniature SPDT Slide Switch (Power)|    1     |
| Red LED (Power Indicator)          |    1     |
| White LED (Claw State Indicator)   |    1     |
| 330Ω Resistor                      |    2     |
| 2-Pin DIP Tactile Switch           |    2     |
| InvenSense MPU6050 6-Axis <br> Motion Tracking Development Module |    1     |  

## Pinout
| Pin on Component         | Pin on ESP |
| ------------------------ | ---------- |
| Power Indicator LED      | 3V3        |
| Claw Grab DIP Switch     | GPIO4      |
| Claw Release DIP Switch  | GPIO13     |
| Claw State Indicator LED | GPIO18     |
| MPU6050 VDD              | 3V3        |
| MPU6050 SDA              | GPIO21     |
| MPU6050 SCL              | GPIO22     |

## Power Supply
- 9V PP3 Alkaline Battery, with TS7805 5V DC Voltage Regulator

## Project Developers
| Role                                   | Developer                  |
| -------------------------------------- | -------------------------- |
| Project Manager                        | Damian Leahu               |
| Remote Controller Developer & Designer | Henry Li                   |
| Robotic Arm Device Developer           | Matthew Godwin             |
| Robotic Arm Device Designers           | Paramwier Singh Katar <br> Mohammed Berghout          |
## PlatformIO IDE Notice
This directory is intended for PlatformIO Test Runner and project tests.

Unit Testing is a software testing method by which individual units of
source code, sets of one or more MCU program modules together with associated
control data, usage procedures, and operating procedures, are tested to
determine whether they are fit for use. Unit testing finds problems early
in the development cycle.

More information about PlatformIO Unit Testing:
- https://docs.platformio.org/en/latest/advanced/unit-testing/index.html
