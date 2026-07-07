# UAV Camera Stabilization System
## Summary
This project involved the design and development of a single-axis UAV camera stabilization system for integration into a Class II unmanned aerial vehicle (UAV). The system combines embedded motor control, inertial sensing, encoder feedback, and flight controller communication to maintain camera orientation during flight.

The objective was to develop a compact electromechanical payload capable of real-time stabilization using field-oriented control (FOC) and closed-loop feedback.

## Mission Planner Parameters
- MNT1_TYPE = 4 (Mavlink Storm32)
- Serial2_BAUD = 115200
- Serial2_Protocol = 2 (Mavlink2)
- SR2_Extra1 = 50 (Hz)
- RC#_Func = 212 (MNT1_Roll) # up to user for MNT Control

### System Architecture
![System Architecture](images/UAVCameraGimbalPinout.png)
- SAMD21 Seeeduino Microcontroller
- SimpleFOC Mini motor driver board
- AS5600 Magnetic Encoder
- GM3506 Gimbal Motor
- MPU6050 Accelerometer

### Logic Pinout 
SAMD21
    -> SimpleFOC
        8, 9, 10, 3 -> IN1, IN2, IN3, ENABLE

    -> MPU6050 
        A5/D5, A4/D4 -> SCL, SDA 

    -> AS5600
        A5/D5, A4/D4 -> SCL, SDA
    
    -> Flight Controller
        RX, TX -> TX, RX
    


---- Common Parameters that need adjusting ------
