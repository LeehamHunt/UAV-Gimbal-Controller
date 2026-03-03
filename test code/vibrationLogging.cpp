#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_sensor.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <mavlink/ardupilotmega/mavlink.h>

// ================= CONFIGURATION =================
// Starting position in degrees (motor shaft angle, accounting for gear ratio)
// This is where the gimbal will move to on startup
const float STARTUP_POSITION_DEG = 01.50f;  // Change this to your desired startup angle

// Set to true to move to startup position on boot
// Set to false to stay at current position
const bool MOVE_TO_STARTUP = true;

// ================= HARDWARE =================
BLDCMotor motor = BLDCMotor(11, 5.5);
BLDCDriver3PWM driver = BLDCDriver3PWM(8, 9, 10, 3);
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

Adafruit_MPU6050 imu;
Adafruit_MPU6050 imu_vibration;
HardwareSerial &fcSerial = Serial1;

// ================= GIMBAL STATE =================
float vehicle_roll_deg = 0.0f;
float vehicle_pitch_deg = 0.0f;
float vehicle_yaw_deg = 0.0f;
bool fc_attitude_valid = false;  // Track if we're getting FC data

float imu_roll_deg = 0.0f;  // Backup/high-rate attitude

float gear_ratio = 0.0665f; //gear ratio compensation

float mount_offset_roll_deg = 0.0f;
float mount_offset_pitch_deg = 0.0f;
float mount_offset_yaw_deg = 0.0f;


float vib_accel_x = 0.0f; //vibration logging
float vib_accel_y = 0.0f;
float vib_accel_z = 0.0f;

// Gimbal mode
enum GimbalMode {
    MODE_FOLLOW,      // Follow vehicle attitude + offset
    MODE_LOCK,        // Lock to absolute angle
    MODE_RC           // RC control
};
GimbalMode gimbal_mode = MODE_FOLLOW;

// Mission/Waypoint data
struct Waypoint {
    float lat;
    float lon;
    float alt;
    uint16_t command;
};
Waypoint current_waypoint = {0, 0, 0, 0};
uint16_t current_wp_index = 0;
uint16_t total_waypoints = 0;

// Startup state
bool startup_complete = false;

// ================= MAVLINK PARSER =================
void sendHeartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t hb;
    
    hb.type = MAV_TYPE_GIMBAL;
    hb.autopilot = MAV_AUTOPILOT_INVALID;
    hb.base_mode = 0;
    hb.custom_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;
    
    mavlink_msg_heartbeat_encode(1, MAV_COMP_ID_GIMBAL, &msg, &hb);
    
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    fcSerial.write(buf, len);
}

void sendGimbalAttitude() {
    mavlink_message_t msg;
    mavlink_gimbal_device_attitude_status_t att;
    
    // Current gimbal angle (from encoder)
    float gimbal_roll_rad = encoder.getAngle() / gear_ratio;  // Undo gear ratio
    
    // Convert to quaternion (simplified - roll only)
    float half_roll = gimbal_roll_rad / 2.0f;
    att.q[0] = cos(half_roll);  // w
    att.q[1] = sin(half_roll);  // x
    att.q[2] = 0;               // y
    att.q[3] = 0;               // z
    
    att.target_system = 0;
    att.target_component = 0;
    att.time_boot_ms = millis();
    att.flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK;
    
    mavlink_msg_gimbal_device_attitude_status_encode(
        1, MAV_COMP_ID_GIMBAL, &msg, &att);
    
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    fcSerial.write(buf, len);
}

void readMAVLinkNonBlocking() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (fcSerial.available()) {
        uint8_t c = fcSerial.read();

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            
            // ===== VEHICLE ATTITUDE =====
            if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
                mavlink_attitude_t att;
                mavlink_msg_attitude_decode(&msg, &att);
                
                vehicle_roll_deg = att.roll * 180.0f / PI;
                vehicle_pitch_deg = att.pitch * 180.0f / PI;
                vehicle_yaw_deg = att.yaw * 180.0f / PI;
                
                fc_attitude_valid = true;  // Mark that we're receiving FC data
            }
            
            // ===== GIMBAL DEVICE SET ATTITUDE (Modern protocol - msgid 284) =====
            if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE) {
                mavlink_gimbal_device_set_attitude_t gim;
                mavlink_msg_gimbal_device_set_attitude_decode(&msg, &gim);

                // Quaternion to Euler (roll only for 1-axis gimbal)
                float qw = gim.q[0];
                float qx = gim.q[1];
                float qy = gim.q[2];
                float qz = gim.q[3];

                float roll_rad = atan2f(
                    2.0f * (qw*qx + qy*qz),
                    1.0f - 2.0f * (qx*qx + qy*qy)
                );

                mount_offset_roll_deg = roll_rad * 180.0f / PI;
                
                // Check flags for mode
                if (gim.flags & GIMBAL_DEVICE_FLAGS_ROLL_LOCK) {
                    gimbal_mode = MODE_LOCK;
                } else {
                    gimbal_mode = MODE_FOLLOW;
                }
                
                Serial.print(F("Gimbal cmd: "));
                Serial.print(mount_offset_roll_deg, 1);
                Serial.print(F("° Mode: "));
                Serial.println(gimbal_mode == MODE_LOCK ? "LOCK" : "FOLLOW");
            }
            
            // ===== DO_MOUNT_CONTROL (Legacy - msgid 205) =====
            if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                mavlink_command_long_t cmd;
                mavlink_msg_command_long_decode(&msg, &cmd);

                if (cmd.command == MAV_CMD_DO_MOUNT_CONTROL) {
                    // param1=pitch, param2=roll, param3=yaw (degrees)
                    mount_offset_pitch_deg = cmd.param1;
                    mount_offset_roll_deg = cmd.param2;
                    mount_offset_yaw_deg = cmd.param3;
                    
                    //Serial.print(F("Mount cmd: R="));
                    //Serial.print(mount_offset_roll_deg, 1);
                    //Serial.print(F("° P="));
                    //Serial.println(mount_offset_pitch_deg, 1);
                    
                    // Send ACK
                    mavlink_message_t ack_msg;
                    mavlink_command_ack_t ack;
                    ack.command = cmd.command;
                    ack.result = MAV_RESULT_ACCEPTED;
                    ack.target_system = cmd.target_system;
                    ack.target_component = cmd.target_component;
                    
                    mavlink_msg_command_ack_encode(1, MAV_COMP_ID_GIMBAL, &ack_msg, &ack);
                    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
                    uint16_t len = mavlink_msg_to_send_buffer(buf, &ack_msg);
                    fcSerial.write(buf, len);
                }
            }
            
            // ===== MISSION CURRENT (which waypoint is active) =====
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
                mavlink_mission_current_t mc;
                mavlink_msg_mission_current_decode(&msg, &mc);
                
                current_wp_index = mc.seq;
                
                //Serial.print(F("Active WP: "));
                //Serial.println(current_wp_index);
            }
            
            // ===== MISSION COUNT (total waypoints) =====
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
                mavlink_mission_count_t mc;
                mavlink_msg_mission_count_decode(&msg, &mc);
                
                total_waypoints = mc.count;
                
                Serial.print(F("Total WP: "));
                Serial.println(total_waypoints);
            }
            
            // ===== MISSION ITEM (waypoint data) =====
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
                mavlink_mission_item_t mi;
                mavlink_msg_mission_item_decode(&msg, &mi);
                
                // Store current waypoint data
                if (mi.seq == current_wp_index) {
                    current_waypoint.lat = mi.x;
                    current_waypoint.lon = mi.y;
                    current_waypoint.alt = mi.z;
                    current_waypoint.command = mi.command;
                    
                    Serial.print(F("WP data: "));
                    Serial.print(current_waypoint.lat, 6);
                    Serial.print(F(", "));
                    Serial.println(current_waypoint.lon, 6);
                }
            }
        }
    }
}

void requestAttitudeRate(uint16_t rate_hz) {
    mavlink_message_t msg;
    mavlink_command_long_t cmd = {};

    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
    cmd.confirmation = 0;

    cmd.param1 = MAVLINK_MSG_ID_ATTITUDE;
    cmd.param2 = 1e6f / rate_hz;  // microseconds

    mavlink_msg_command_long_encode(
        1, MAV_COMP_ID_GIMBAL, &msg, &cmd);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    fcSerial.write(buf, len);
}


// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    
    delay(1000);

    fcSerial.begin(115200);
    requestAttitudeRate(57.6);  // 57.6 Hz

    Wire.begin();

    Serial.println(F("\n=== Roll Gimbal Controller ==="));

    // -------- Base IMU --------
    Serial.print(F("IMU Base init... "));
    if (imu.begin(0x68) != 0) {
        Serial.println(F("FAILED!"));
        while(1);
    }
    Serial.println(F("OK"));
    
    Serial.println(F("Calibrating IMU..."));
    imu.setAccelerometerRange(MPU6050_RANGE_8_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println(F("Base IMU ready"));

        // -------- Vibration IMU --------
    Serial.print(F("IMU Vibration init... "));
    if (imu.begin(0x69) != 0) {
        Serial.println(F("FAILED!"));
        while(1);
    }
    Serial.println(F("OK"));
    
    Serial.println(F("Calibrating IMU..."));
    imu.setAccelerometerRange(MPU6050_RANGE_2_G);
    imu.setGyroRange(MPU6050_RANGE_250_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_260_HZ);
    Serial.println(F("Base IMU ready"));

    // -------- Encoder --------
    encoder.init();
    
    // Read current absolute position
    float current_pos = encoder.getAngle();
    Serial.print(F("Encoder ready - Current position: "));
    Serial.print(current_pos / gear_ratio, 2);  // Show in degrees (undo gear ratio)
    Serial.println(F("°"));

    // -------- Driver --------
    driver.voltage_power_supply = 12;
    driver.voltage_limit = 8;
    driver.init();
    Serial.println(F("Driver ready"));

    // -------- Motor --------
    motor.linkDriver(&driver);
    motor.linkSensor(&encoder);
    motor.controller = MotionControlType::angle;

    // PID tuning (adjust as needed)
    motor.P_angle.P = 9.0;
    motor.P_angle.I = 0.0;
    motor.P_angle.D = 0.90;
    motor.P_angle.limit = 60;

    motor.PID_velocity.P = 0.05;
    motor.PID_velocity.I = 1.0;
    motor.PID_velocity.output_ramp = 1500;
    motor.PID_velocity.limit = 100;

    motor.voltage_limit = 8;
    motor.current_limit = 0.6;
    motor.LPF_velocity.Tf = 0.02;

    motor.init();
    motor.initFOC();
    
    Serial.println(F("\n✓ Gimbal ready"));
    
    if (MOVE_TO_STARTUP) {
        Serial.print(F("Moving to startup position: "));
        Serial.print(STARTUP_POSITION_DEG, 1);
        Serial.println(F("°"));
    } else {
        Serial.println(F("Staying at current position"));
    }
    
    Serial.println(F("Listening for MAVLink commands...\n"));
    
    delay(1000);
}

// ================= MAIN LOOP =================
void loop() {
    // --- MAVLink communication ---
    readMAVLinkNonBlocking();
    
    // Send heartbeat every second
    static uint32_t last_hb = 0;
    if (millis() - last_hb > 1000) {
        last_hb = millis();
        sendHeartbeat();
    }
    
    // Send gimbal attitude at 10Hz
    static uint32_t last_att = 0;
    if (millis() - last_att > 100) {
        last_att = millis();
        sendGimbalAttitude();
    }

    // --- IMU settling time ---
    static bool imu_ready = false;
    static uint32_t imu_start = millis();


    if (!imu_ready) {
        if (millis() - imu_start > 1500) {
            imu_ready = true;
            Serial.println(F("IMU settled"));
        }
        return;
    }
    
    // --- Startup positioning ---
    if (!startup_complete) {
        static uint32_t startup_timer = millis();
        
        if (MOVE_TO_STARTUP) {
            // Move to startup position
            motor.target = STARTUP_POSITION_DEG * gear_ratio;  // Apply gear ratio
            motor.loopFOC();
            motor.move();
            
            // Check if we've reached the target (within 1 degree)
            float current_angle = encoder.getAngle() / gear_ratio;
            if (abs(current_angle - STARTUP_POSITION_DEG) < 1.0f || 
                millis() - startup_timer > 3000) {  // Timeout after 3 seconds
                startup_complete = true;
                
                // Initialize mount offset to maintain startup position
                // This prevents the gimbal from immediately moving when control starts
                mount_offset_roll_deg = STARTUP_POSITION_DEG;
                
                Serial.println(F("Startup positioning complete, starting control"));
                Serial.print(F("Mount offset initialized to: "));
                Serial.print(mount_offset_roll_deg, 1);
                Serial.println(F("°"));
            }
        } else {
            // Just wait a bit then start
            if (millis() - startup_timer > 500) {
                startup_complete = true;
                
                // Read current position and set offset to maintain it
                float current_angle = encoder.getAngle() / gear_ratio;
                mount_offset_roll_deg = current_angle;
                
                Serial.println(F("Starting control from current position"));
                Serial.print(F("Mount offset initialized to: "));
                Serial.print(mount_offset_roll_deg, 1);
                Serial.println(F("°"));
            }
        }
        return;
    }
    
    // --- IMU update (always update for rate damping) ---
    sensors_event_t a, g, temp;
    imu.getEvent(&a,&g,&temp);
    

    imu_roll_deg = g.gyro.z;
    float roll_rate = g.gyro.z;  // deg/s
    

    // --- IMU Vibration monitoring ---
    sensors_event_t vib_a, vib_g, vib_temp;
    imu_vibration.getEvent(&vib_a,&vib_g,&vib_temp);

    vib_accel_x = vib_a.acceleration.x;
    vib_accel_y = vib_a.acceleration.y;
    vib_accel_z = vib_a.acceleration.z;



    // --- Choose attitude source ---
    float current_roll_deg;
    if (fc_attitude_valid) {
        // Use FC attitude (drift-free, fused with GPS/compass)
        current_roll_deg = vehicle_roll_deg;
    } else {
        // Fallback to local IMU if no FC data
        current_roll_deg = imu_roll_deg;
    }

    // --- Low-pass filter attitude ---
    static float roll_filt = 0.0f;
    const float alpha = 0.9f;  // Higher alpha = faster response, less smooth
    roll_filt += alpha * (current_roll_deg - roll_filt);

    // --- Control law with gear ratio ---
    float target_angle_deg;
    float target_angle_rad;  // For plotting (motor shaft angle)
    
    if (gimbal_mode == MODE_LOCK) {
        // LOCK mode: absolute angle (ignore vehicle roll)
        target_angle_deg = mount_offset_roll_deg;
        target_angle_rad = mount_offset_roll_deg * gear_ratio;
    } else {
        // FOLLOW mode: stabilize + offset
        target_angle_deg = -roll_filt + mount_offset_roll_deg;
        target_angle_rad = (
            -roll_filt +                    // Stabilization (FC attitude)
            //0.06f * roll_rate +             // Feedforward damping (local IMU rate)
            mount_offset_roll_deg           // User offset
        ) * gear_ratio;                          // Gear ratio
    }

    motor.target = target_angle_rad;//+5; //offset to face down

    // --- Fast loops - RUN AS FAST AS POSSIBLE ---
    motor.loopFOC();
    motor.move();
    

    //--- Debug output (20Hz for plotting) ---
    static uint32_t dbg = 0;
    if (millis() - dbg > 50) {  // 20Hz
        dbg = millis();
        
        // Format: R:roll T:target A:actual E:error V:velocity Vq:voltage
        // Target and Actual are in degrees (gimbal reference frame, not motor shaft)
        float actual_angle_deg = encoder.getAngle() / gear_ratio;  // Convert from motor shaft to gimbal angle
        
        Serial.print("R:"); Serial.print(current_roll_deg, 2);
        Serial.print(" T:"); Serial.print(target_angle_deg, 4);
        Serial.print(" A:"); Serial.print(-actual_angle_deg, 4);
        Serial.print(" E:"); Serial.print(target_angle_deg - actual_angle_deg, 4);
        Serial.print(" V:"); Serial.print(motor.shaft_velocity, 2);
        Serial.print(" Vq:"); Serial.print(motor.voltage.q, 2);
        Serial.print(" VX:"); Serial.print(vib_accel_x, 3);
        Serial.print(" VY:"); Serial.print(vib_accel_y, 3);
        Serial.print(" VZ:"); Serial.println(vib_accel_z, 3);
        
    }
    // if (Serial.available()) {
    //     char cmd = Serial.read();

    //     if (cmd == 'd' || cmd =='D') {
    //         //dump vib data
    //         Serial.println(F("\n====== Vibration Data Dump ========"));
    //         Serial.print(F("Samples stored:  "));
    //         Serial.println(vib_samples_stored)
    //     }
    // }

}