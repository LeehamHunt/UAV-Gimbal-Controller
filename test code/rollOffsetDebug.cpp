#include <Arduino.h>
#include <mavlink/ardupilotmega/mavlink.h>

HardwareSerial &fcSerial = Serial1;

// Statistics
uint32_t total_messages = 0;
uint32_t attitude_count = 0;
uint32_t heartbeat_count = 0;
uint32_t gimbal_cmd_count = 0;
uint32_t other_count = 0;

// Latest data
float vehicle_roll_deg = 0.0f;
float vehicle_pitch_deg = 0.0f;
float vehicle_yaw_deg = 0.0f;
uint32_t last_attitude_time = 0;

float mount_roll_cmd = 0.0f;
float mount_pitch_cmd = 0.0f;
uint32_t last_cmd_time = 0;
String last_cmd_type = "None";

uint8_t fc_system_id = 0;
uint8_t fc_component_id = 0;
uint8_t fc_autopilot = 0;

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

void sendMountStatus() {
    mavlink_message_t msg;
    mavlink_mount_status_t status;
    
    // Send current gimbal angles (in centidegrees)
    status.pointing_a = (int32_t)(mount_roll_cmd * 100);   // Roll
    status.pointing_b = (int32_t)(mount_pitch_cmd * 100);  // Pitch  
    status.pointing_c = 0;  // Yaw
    
    status.target_system = 0;  // Broadcast
    status.target_component = 0;
    
    mavlink_msg_mount_status_encode(1, MAV_COMP_ID_GIMBAL, &msg, &status);
    
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    fcSerial.write(buf, len);
}

void requestAttitudeStream() {
    // Try multiple methods to request ATTITUDE
    
    // Method 1: SET_MESSAGE_INTERVAL (modern)
    {
        mavlink_message_t msg;
        mavlink_command_long_t cmd = {};
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.command = MAV_CMD_SET_MESSAGE_INTERVAL;
        cmd.confirmation = 0;
        cmd.param1 = MAVLINK_MSG_ID_ATTITUDE;
        cmd.param2 = 20000;  // 20ms = 50Hz
        
        mavlink_msg_command_long_encode(1, MAV_COMP_ID_GIMBAL, &msg, &cmd);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        fcSerial.write(buf, len);
    }
    
    // Method 2: REQUEST_DATA_STREAM (legacy)
    {
        mavlink_message_t msg;
        mavlink_request_data_stream_t req;
        req.target_system = 1;
        req.target_component = 1;
        req.req_stream_id = MAV_DATA_STREAM_EXTRA1;  // Attitude stream
        req.req_message_rate = 50;  // 50Hz
        req.start_stop = 1;  // Start
        
        mavlink_msg_request_data_stream_encode(1, MAV_COMP_ID_GIMBAL, &msg, &req);
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        fcSerial.write(buf, len);
    }
    
    Serial.println(F(">>> Sent REQUEST for ATTITUDE (both methods)"));
}

void processMAVLink() {
    mavlink_message_t msg;
    mavlink_status_t status;

    while (fcSerial.available()) {
        uint8_t c = fcSerial.read();

        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            total_messages++;
            
            // Track FC info from first heartbeat
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && fc_system_id == 0) {
                fc_system_id = msg.sysid;
                fc_component_id = msg.compid;
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                fc_autopilot = hb.autopilot;
            }
            
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    heartbeat_count++;
                    break;
                }
                
                case MAVLINK_MSG_ID_ATTITUDE: {
                    attitude_count++;
                    mavlink_attitude_t att;
                    mavlink_msg_attitude_decode(&msg, &att);
                    
                    vehicle_roll_deg = att.roll * 180.0f / PI;
                    vehicle_pitch_deg = att.pitch * 180.0f / PI;
                    vehicle_yaw_deg = att.yaw * 180.0f / PI;
                    last_attitude_time = millis();
                    
                    // Serial.print(F("✓ ATTITUDE: R="));
                    // Serial.print(vehicle_roll_deg, 2);
                    // Serial.print(F("° P="));
                    // Serial.print(vehicle_pitch_deg, 2);
                    // Serial.print(F("° Y="));
                    // Serial.println(vehicle_yaw_deg, 2);
                    // break;
                }
                
                case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE: {
                    gimbal_cmd_count++;
                    mavlink_gimbal_manager_set_attitude_t gim;
                    mavlink_msg_gimbal_manager_set_attitude_decode(&msg, &gim);
                    
                    float qw = gim.q[0];
                    float qx = gim.q[1];
                    float qy = gim.q[2];
                    float qz = gim.q[3];
                    
                    float roll = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy)) * 180.0f / PI;
                    float pitch = asinf(2.0f*(qw*qy - qz*qx)) * 180.0f / PI;
                    
                    mount_roll_cmd = roll;
                    mount_pitch_cmd = pitch;
                    last_cmd_time = millis();
                    last_cmd_type = "GIMBAL_MANAGER_SET_ATTITUDE";
                    
                    Serial.print(F("✓ GIMBAL_MGR_SET_ATT: R="));
                    Serial.print(roll, 2);
                    Serial.print(F("° P="));
                    Serial.print(pitch, 2);
                    Serial.print(F("° flags=0x"));
                    Serial.println(gim.flags, HEX);
                    break;
                }
                
                case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW: {
                    gimbal_cmd_count++;
                    mavlink_gimbal_manager_set_pitchyaw_t gim;
                    mavlink_msg_gimbal_manager_set_pitchyaw_decode(&msg, &gim);
                    
                    mount_pitch_cmd = gim.pitch * 180.0f / PI;
                    mount_roll_cmd = gim.yaw * 180.0f / PI;
                    last_cmd_time = millis();
                    last_cmd_type = "GIMBAL_MANAGER_SET_PITCHYAW";
                    
                    Serial.print(F("✓ GIMBAL_MGR_SET_PITCHYAW: P="));
                    Serial.print(mount_pitch_cmd, 2);
                    Serial.print(F("° Y="));
                    Serial.print(mount_roll_cmd, 2);
                    Serial.print(F("° flags=0x"));
                    Serial.println(gim.flags, HEX);
                    break;
                }
                
                case MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE: {
                    gimbal_cmd_count++;
                    mavlink_gimbal_device_set_attitude_t gim;
                    mavlink_msg_gimbal_device_set_attitude_decode(&msg, &gim);
                    
                    float qw = gim.q[0];
                    float qx = gim.q[1];
                    float qy = gim.q[2];
                    float qz = gim.q[3];
                    
                    float roll = atan2f(2.0f*(qw*qx + qy*qz), 1.0f - 2.0f*(qx*qx + qy*qy)) * 180.0f / PI;
                    float pitch = asinf(2.0f*(qw*qy - qz*qx)) * 180.0f / PI;
                    
                    mount_roll_cmd = roll;
                    mount_pitch_cmd = pitch;
                    last_cmd_time = millis();
                    last_cmd_type = "GIMBAL_DEVICE_SET_ATTITUDE";
                    
                    Serial.print(F("✓ GIMBAL_DEVICE_SET_ATT: R="));
                    Serial.print(roll, 2);
                    Serial.print(F("° P="));
                    Serial.print(pitch, 2);
                    Serial.print(F("° flags=0x"));
                    Serial.println(gim.flags, HEX);
                    break;
                }
                
                case MAVLINK_MSG_ID_COMMAND_LONG: {
                    mavlink_command_long_t cmd;
                    mavlink_msg_command_long_decode(&msg, &cmd);
                    
                    Serial.print(F("→ COMMAND_LONG received: CMD="));
                    Serial.print(cmd.command);
                    Serial.print(F(" target_sys="));
                    Serial.print(cmd.target_system);
                    Serial.print(F(" target_comp="));
                    Serial.println(cmd.target_component);
                    
                    if (cmd.command == MAV_CMD_DO_MOUNT_CONTROL) {
                        gimbal_cmd_count++;
                        mount_pitch_cmd = cmd.param1;
                        mount_roll_cmd = cmd.param2;
                        last_cmd_time = millis();
                        last_cmd_type = "DO_MOUNT_CONTROL";
                        
                        Serial.print(F("✓ DO_MOUNT_CONTROL: P="));
                        Serial.print(mount_pitch_cmd, 2);
                        Serial.print(F("° R="));
                        Serial.print(mount_roll_cmd, 2);
                        Serial.println(F("°"));
                        
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
                        Serial.println(F("  → Sent ACK"));
                    }
                    else if (cmd.command == MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW) {
                        gimbal_cmd_count++;
                        mount_pitch_cmd = cmd.param1;
                        mount_roll_cmd = cmd.param2;
                        last_cmd_time = millis();
                        last_cmd_type = "DO_GIMBAL_MANAGER_PITCHYAW";
                        
                        Serial.print(F("✓ DO_GIMBAL_MGR_PITCHYAW: P="));
                        Serial.print(mount_pitch_cmd, 2);
                        Serial.print(F("° Y="));
                        Serial.print(mount_roll_cmd, 2);
                        Serial.println(F("°"));
                        
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
                        Serial.println(F("  → Sent ACK"));
                    }
                    else if (cmd.command == MAV_CMD_DO_MOUNT_CONFIGURE) {
                        gimbal_cmd_count++;
                        last_cmd_type = "DO_MOUNT_CONFIGURE";
                        last_cmd_time = millis();
                        
                        int mode = (int)cmd.param1;
                        Serial.print(F("✓ DO_MOUNT_CONFIGURE: mode="));
                        Serial.print(mode);
                        Serial.print(F(" ("));
                        switch(mode) {
                            case 0: Serial.print(F("RETRACT")); break;
                            case 1: Serial.print(F("NEUTRAL")); break;
                            case 2: Serial.print(F("MAVLINK_TARGETING")); break;
                            case 3: Serial.print(F("RC_TARGETING")); break;
                            case 4: Serial.print(F("GPS_POINT")); break;
                            default: Serial.print(F("UNKNOWN")); break;
                        }
                        Serial.println(F(")"));
                        
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
                        Serial.println(F("  → Sent ACK"));
                    }
                    else {
                        Serial.print(F("  → Unhandled command: "));
                        Serial.println(cmd.command);
                    }
                    break;
                }
                
                default:
                    other_count++;
                    break;
            }
        }
    }
}

void printStatus() {
    Serial.println(F("\n========================================"));
    Serial.println(F("MAVLink Communication Status"));
    Serial.println(F("========================================"));
    
    // Flight Controller Info
    Serial.print(F("FC System ID: "));
    Serial.print(fc_system_id);
    Serial.print(F(" | Component: "));
    Serial.print(fc_component_id);
    Serial.print(F(" | Autopilot: "));
    Serial.print(fc_autopilot);
    Serial.println(F(" (8=ArduPilot)"));
    
    // Message Statistics
    Serial.println(F("\n--- Message Counts ---"));
    Serial.print(F("Total messages: "));
    Serial.println(total_messages);
    Serial.print(F("  Heartbeats: "));
    Serial.print(heartbeat_count);
    Serial.println(F(" ✓"));
    Serial.print(F("  Attitude: "));
    Serial.print(attitude_count);
    if (attitude_count == 0) {
        Serial.println(F(" ⚠ ZERO"));
    } else {
        Serial.println(F(" ✓"));
    }
    Serial.print(F("  Gimbal cmds: "));
    Serial.print(gimbal_cmd_count);
    if (gimbal_cmd_count == 0) {
        Serial.println(F(" (waiting for commands)"));
    } else {
        Serial.println(F(" ✓"));
    }
    Serial.print(F("  Other: "));
    Serial.println(other_count);
    
    // Latest Attitude
    Serial.println(F("\n--- Latest Vehicle Attitude ---"));
    if (attitude_count > 0) {
        Serial.print(F("Roll: "));
        Serial.print(vehicle_roll_deg, 2);
        Serial.print(F("° | Pitch: "));
        Serial.print(vehicle_pitch_deg, 2);
        Serial.print(F("° | Yaw: "));
        Serial.print(vehicle_yaw_deg, 2);
        Serial.println(F("°"));
        
        uint32_t age = millis() - last_attitude_time;
        Serial.print(F("Age: "));
        Serial.print(age);
        Serial.print(F(" ms"));
        
        if (age > 2000) {
            Serial.println(F(" ⚠ STALE - No recent attitude!"));
        } else if (age > 500) {
            Serial.println(F(" ⚠ OLD"));
        } else {
            Serial.println(F(" ✓ FRESH"));
        }
    } else {
        Serial.println(F("⚠ NO ATTITUDE DATA RECEIVED"));
        Serial.println(F(""));
        Serial.println(F("This is NORMAL with SERIAL_PROTOCOL=8 (Gimbal)."));
        Serial.println(F("Options:"));
        Serial.println(F("  1. Change to SERIAL2_PROTOCOL=2 (MAVLink2)"));
        Serial.println(F("  2. Set SR2_EXTRA1=50 parameter on FC"));
        Serial.println(F("  3. Use only local IMU in main code"));
    }
    
    // Latest Command
    Serial.println(F("\n--- Latest Gimbal Command ---"));
    if (gimbal_cmd_count > 0) {
        Serial.print(F("Type: "));
        Serial.println(last_cmd_type);
        Serial.print(F("Pitch: "));
        Serial.print(mount_pitch_cmd, 2);
        Serial.print(F("° | Roll: "));
        Serial.print(mount_roll_cmd, 2);
        Serial.println(F("°"));
        
        uint32_t age = millis() - last_cmd_time;
        Serial.print(F("Age: "));
        Serial.print(age);
        Serial.println(F(" ms"));
    } else {
        Serial.println(F("⚠ NO GIMBAL COMMANDS RECEIVED"));
        Serial.println(F("Try sending commands from Mission Planner:"));
        Serial.println(F("  Setup > Optional Hardware > Camera Gimbal"));
        Serial.println(F("  Or use: DO_MOUNT_CONTROL command"));
    }
    
    Serial.println(F("========================================\n"));
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);
    
    Serial.println(F("\n╔════════════════════════════════════╗"));
    Serial.println(F("║  MAVLink Gimbal Debug Tool         ║"));
    Serial.println(F("╚════════════════════════════════════╝\n"));
    
    // Start serial to FC
    Serial.println(F("Starting FC serial at 115200 baud..."));
    fcSerial.begin(115200);
    delay(100);
    
    Serial.println(F("Ready - Listening for MAVLink messages\n"));
    Serial.println(F("This gimbal identifies as:"));
    Serial.println(F("  System ID: 1"));
    Serial.println(F("  Component ID: 154 (MAV_COMP_ID_GIMBAL)"));
    Serial.println(F("  Type: MAV_TYPE_GIMBAL\n"));
    Serial.println(F("Sending:"));
    Serial.println(F("  - Heartbeat every 1s"));
    Serial.println(F("  - MOUNT_STATUS every 200ms (enables MP gimbal tab)"));
    Serial.println(F("  - ATTITUDE requests every 5s\n"));
    Serial.println(F("Commands to test:"));
    Serial.println(F("  - DO_MOUNT_CONTROL (legacy)"));
    Serial.println(F("  - DO_MOUNT_CONFIGURE (set mode)"));
    Serial.println(F("  - DO_GIMBAL_MANAGER_PITCHYAW"));
    Serial.println(F("\nStatus report every 5 seconds...\n"));
}

void loop() {
    // Process incoming messages
    processMAVLink();
    
    // Send heartbeat every second
    static uint32_t last_hb = 0;
    if (millis() - last_hb > 1000) {
        last_hb = millis();
        sendHeartbeat();
    }
    
    // Send mount status at 5Hz (required for Mission Planner to enable gimbal tab!)
    static uint32_t last_mount_status = 0;
    if (millis() - last_mount_status > 200) {
        last_mount_status = millis();
        sendMountStatus();
    }
    
    // Request attitude stream every 5 seconds
    static uint32_t last_request = 0;
    if (millis() - last_request > 5000) {
        last_request = millis();
        requestAttitudeStream();
    }
    
    // Print status every 5 seconds
    static uint32_t last_status = 0;
    if (millis() - last_status > 5000) {
        last_status = millis();
        printStatus();
    }
}