# Jump-Gliding Robot Controller  

## Overview  
This project provides an embedded system for a jump-gliding robot. The system consists of two hardware components:  
1. **Robot Controller** – Responsible for controlling the robot’s movements.  
2. **User Command Controller** – A wireless remote controller for user commands.  

## Hardware  

### 1. Robot Controller  
The robot controller manages movement control and sensor data processing.  

- **MCU**: STM32F4  
- **DC Motor Drivers**: HR8833  
- **IMU**: MPU9250 (9-axis motion sensor)  
- **RF Communication**: HC-12 (Long-range wireless communication module)  
- **Angle Sensor & ADC**: For precise position sensing  
- **File Handling & Storage**: SDIO (Secure Digital Input Output) for data logging  

### 2. User Command Controller  
The user command controller sends control signals to the robot via RF communication.  

- **MCU**: STM32F4 Nucleo  
- **Input Interfaces**:  
  - DIP switches (for mode selection)  
  - Tact switches (for discrete commands)  
  - Joystick (for movement control)  
- **RF Communication**: HC-12  

## Software  

### Robot Controller  
The robot controller software is responsible for:  
- **DC Motor Position Control**: Precise motor control for jump-gliding maneuvers.  
- **IMU Data Processing**: Reading and filtering IMU sensor data for stability.  
- **Data Logging**: Storing sensor data and control states for analysis.  
