#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <mavlink/ardupilotmega/mavlink.h>

// ===== HARDWARE =====
BLDCMotor motor = BLDCMotor(11);  // 7 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(8, 9, 10, 3);
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

MPU6050 imu(Wire);
HardwareSerial &fcSerial = Serial1;

// ===== CONTROL STATE =====
float imu_roll = 0.0f;           // IMU measured roll (degrees)
float encoder_angle = 0.0f;      // Motor position (radians)

// TEST MODE - set to true to test basic motor movement
bool test_mode = true;           // Change to false after testing
float test_target = 0.0f;        // Test position target

// Command from Mission Planner (DO_MOUNT_CONTROL)
float commanded_offset_roll = 0.0f;   // Degrees
float commanded_offset_pitch = 0.0f;
float commanded_offset_yaw = 0.0f;

// Vehicle attitude from flight controller
float vehicle_roll = 0.0f;
float vehicle_pitch = 0.0f;
float vehicle_yaw = 0.0f;

// Timing
unsigned long last_mavlink_time = 0;
const unsigned long MAVLINK_TIMEOUT = 500;  // ms

// ===== MAVLINK PARSER =====
void readMAVLinkNonBlocking() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (fcSerial.available()) {
        uint8_t c = fcSerial.read();

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            
            // Vehicle attitude from Pixhawk
            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&msg, &att);

                vehicle_roll  = att.roll * 180.0f / M_PI;
                vehicle_pitch = att.pitch * 180.0f / M_PI;
                vehicle_yaw   = att.yaw * 180.0f / M_PI;
                
                last_mavlink_time = millis();
            }

            // Mount control command from Mission Planner
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);
                
                if (cmd.command == MAV_CMD_DO_MOUNT_CONTROL) {
                    // param1 = pitch, param2 = roll, param3 = yaw (degrees)
                    commanded_offset_pitch = cmd.param1;
                    commanded_offset_roll = cmd.param2;
                    commanded_offset_yaw = cmd.param3;
                    
                    Serial.print(F("*** MOUNT COMMAND: Roll="));
                    Serial.print(commanded_offset_roll);
                    Serial.print(F(" Pitch="));
                    Serial.println(commanded_offset_pitch);
                    
                    // Send ACK
                    mavlink_message_t ack_msg;
                    mavlink_command_ack_t ack;
                    ack.command = cmd.command;
                    ack.result = MAV_RESULT_ACCEPTED;
                    ack.target_system = cmd.target_system;
                    ack.target_component = cmd.target_component;
                    
                    mavlink_msg_command_ack_encode(1, 1, &ack_msg, &ack);
                    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                    uint16_t len = mavlink_msg_to_send_buffer(buf, &ack_msg);
                    fcSerial.write(buf, len);
                }
            }
        }
    }
}

// ===== SETUP =====
void setup() {
    Serial.begin(115200);
    while (!Serial);
    
    fcSerial.begin(57600);
    Wire.begin();
    
    // Initialize IMU
    Serial.println(F("Initializing IMU..."));
    if (imu.begin() != 0) {
        Serial.println(F("IMU init failed!"));
        while(1);
    }
    Serial.println(F("Calibrating IMU offsets (keep still)..."));
    imu.calcOffsets();
    Serial.println(F("IMU ready"));
    
    // Initialize encoder
    encoder.init();
    Serial.println(F("Encoder ready"));
    
    // Initialize motor driver
    driver.voltage_power_supply = 12;
    driver.voltage_limit = 12;
    driver.pwm_frequency = 25000;  // Start lower, increase if quiet
    driver.init();
    
    // Configure motor
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::angle;
    
    // Check if encoder direction needs flipping
    motor.sensor_direction = Direction::CW;  // Try CW first, change to CCW if backwards
    
    // Zero the encoder at current position for testing
    //motor.zero_electric_angle = 0;
    
    // ===== CRITICAL: START WITH VERY LOW GAINS =====
    // Angle Loop (Position Control)
    motor.P_angle.P = 3.0;        // START VERY LOW
    motor.P_angle.I = 0.0;        // No integral
    motor.P_angle.D = 0.2;        // Light damping
    motor.P_angle.output_ramp = 0;
    motor.P_angle.limit = 10;     // Very conservative velocity limit
    
    // Velocity Loop
    motor.PID_velocity.P = 0.05;  // Very low for stability
    motor.PID_velocity.I = 0.5;   // Reduced integral
    motor.PID_velocity.D = 0.0;
    motor.PID_velocity.output_ramp = 500;  // Slower ramp
    motor.PID_velocity.limit = 3;  // Match voltage_limit
    
    // Motor Limits
    motor.voltage_limit = 4;      // START VERY LOW - increase slowly
    motor.current_limit = 0.5;    // NOTE: Needs current sensor hardware!
    motor.velocity_limit = 20;    // rad/s
    
    // IMPORTANT: Current limiting only works if you have current sensors!
    // Without hardware current sensing, use voltage_limit to control current
    
    // Low-pass filter
    motor.LPF_velocity.Tf = 0.02;
    
    motor.init();
    motor.initFOC();
    
    Serial.println(F("FOC initialized. Testing encoder..."));
    
    // Test encoder readings
    for (int i = 0; i < 5; i++) {
        float angle = encoder.getAngle();
        Serial.print(F("Encoder angle: "));
        Serial.print(angle * 180.0f / PI);
        Serial.println(F(" degrees"));
        delay(200);
    }
    
    Serial.println(F("Manually rotate motor - watch for changes"));
    delay(3000);
    
    for (int i = 0; i < 5; i++) {
        float angle = encoder.getAngle();
        Serial.print(F("Encoder angle: "));
        Serial.print(angle * 180.0f / PI);
        Serial.println(F(" degrees"));
        delay(200);
    }
    
    delay(1000);
    
    Serial.println(F("\n=== 1-Axis Gimbal Ready ==="));
    if (test_mode) {
        Serial.println(F("*** TEST MODE ACTIVE ***"));
        Serial.println(F("Motor will slowly sweep +/- 30 degrees"));
        Serial.println(F("Watch encoder values change"));
        Serial.println(F("Set test_mode = false for normal operation"));
    } else {
        Serial.println(F("Stabilization: -IMU_roll + offset"));
        Serial.println(F("Command via: DO_MOUNT_CONTROL"));
    }
    Serial.println(F("===========================\n"));
}

// ===== MAIN LOOP =====
void loop() {
    // High-speed FOC loop
    motor.loopFOC();
    
    // Read MAVLink messages
    readMAVLinkNonBlocking();
    
    // Update IMU
    imu.update();
    imu_roll = imu.getAngleZ();  // Z-axis is typically roll for MPU6050
    
    // Read current motor position
    encoder_angle = encoder.getAngle();
    
    // ===== CONTROL LAW =====
    if (test_mode) {
        // TEST MODE: Slow sine wave for debugging
        static unsigned long test_start = millis();
        float t = (millis() - test_start) / 1000.0f;  // seconds
        test_target = 30.0f * sin(t * 0.5f);  // +/- 30 degrees, 0.5 Hz
        motor.target = test_target * PI / 180.0f;
    } else {
        // NORMAL MODE: Stabilization + offset
        // Target = -IMU_roll (stabilization) + commanded_offset (user command)
        float target_angle_deg = -imu_roll + commanded_offset_roll;
        motor.target = target_angle_deg * PI / 180.0f;
    }
    
    // Execute motion control
    motor.move();
    
    // ===== TIMEOUT PROTECTION =====
    // Return to zero offset if no MAVLink data
    if (millis() - last_mavlink_time > MAVLINK_TIMEOUT) {
        commanded_offset_roll = 0.0f;
    }
    
    // ===== DEBUG OUTPUT (2Hz) =====
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 500) {
        last_debug = millis();
        
        float target_deg = motor.target * 180.0f / PI;
        float encoder_deg = encoder_angle * 180.0f / PI;
        float error_deg = (motor.target - encoder_angle) * 180.0f / PI;
        
        if (test_mode) {
            Serial.print(F("TEST | Target: ")); Serial.print(target_deg, 1);
            Serial.print(F(" | Encoder: ")); Serial.print(encoder_deg, 1);
            Serial.print(F(" | Error: ")); Serial.print(error_deg, 1);
            Serial.print(F(" | Vel: ")); Serial.print(motor.shaft_velocity, 2);
            Serial.print(F(" | Vq: ")); Serial.println(motor.voltage.q, 2);
        } else {
            Serial.print(F("IMU: ")); Serial.print(imu_roll, 1);
            Serial.print(F(" | Cmd: ")); Serial.print(commanded_offset_roll, 1);
            Serial.print(F(" | Target: ")); Serial.print(target_deg, 1);
            Serial.print(F(" | Encoder: ")); Serial.print(encoder_deg, 1);
            Serial.print(F(" | Error: ")); Serial.print(error_deg, 1);
            Serial.print(F(" | Vq: ")); Serial.println(motor.voltage.q, 2);
        }
    }
}