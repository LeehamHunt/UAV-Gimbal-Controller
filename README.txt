----- Intro ------
This code is for a 1-axis Roll gimbal, works with following Mission Planner Parameters
MNT1_TYPE = 4 (Mavlink Storm32)
Serial2_BAUD = 115200
Serial2_Protocol = 2 (Mavlink2)
SR2_Extra1 = 50 (Hz)
RC#_Func = 212 (MNT1_Roll) # up to user for MNT Control

----- Hardware Used ------
SAMD21 Seeeduino Microcontroller
SimpleFOC Mini motor driver board
AS5600 Magnetic Encoder
GM3506 Gimbal Motor
MPU6050 Accelerometer

---- Logic Pinout ----- 
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