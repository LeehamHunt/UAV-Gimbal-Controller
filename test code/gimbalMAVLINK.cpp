#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <mavlink/ardupilotmega/mavlink.h>

// ----- HARDWARE -----
BLDCMotor motor = BLDCMotor(7,5.5);
BLDCDriver3PWM driver = BLDCDriver3PWM(8, 9, 10, 3);
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

MPU6050 imu(Wire);
HardwareSerial &fcSerial = Serial1;

// ----- CONTROL VARIABLES -----
float rollAngle = 0.0f;

// ----- MAVLINK OFFSET VARIABLES -----
float gimbalRoll = 0.0f;
float gimbalPitch = 0.0f;
float gimbalYaw = 0.0f;

float vehicleRoll = 0.0f;
float vehiclePitch = 0.0f;
float vehicleYaw = 0.0f;

float mount_offset_roll = 0.0f;   // Roll offset for motor
float mount_offset_pitch = 0.0f;
float mount_offset_yaw = 0.0f;

// ----- TIMING -----
uint32_t lastControlMicros = 0;
const uint32_t controlInterval = 1000; // 1 kHz = 1000 µs

// -------------------- MAVLINK PARSER --------------------
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

            vehicleRoll  = att.roll * 180.0f / M_PI;
            vehiclePitch = att.pitch * 180.0f / M_PI;
            vehicleYaw   = att.yaw * 180.0f / M_PI;
        }

        // Gimbal actual attitude
        if (msg.msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS) {
            mavlink_gimbal_device_attitude_status_t gimbal;
            mavlink_msg_gimbal_device_attitude_status_decode(&msg, &gimbal);

            // Convert quaternion to Euler angles (degrees)
            float q0 = gimbal.q[0];
            float q1 = gimbal.q[1];
            float q2 = gimbal.q[2];
            float q3 = gimbal.q[3];

            gimbalRoll  = atan2f(2.0f*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180.0f / M_PI;
            gimbalPitch = asinf(2.0f*(q0*q2 - q3*q1)) * 180.0f / M_PI;
            gimbalYaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180.0f / M_PI;

            // Compute offset relative to vehicle
            mount_offset_roll  = gimbalRoll  - vehicleRoll;
            mount_offset_pitch = gimbalPitch - vehiclePitch;
            mount_offset_yaw   = gimbalYaw   - vehicleYaw;
        }
    }
}


}

// -------------------- SETUP --------------------
void setup() {
Serial.begin(115200);
while (!Serial);

fcSerial.begin(57600);
Wire.begin();
imu.begin();
imu.calcOffsets();

encoder.init();


driver.voltage_power_supply = 12;
driver.voltage_limit = 8;
driver.pwm_frequency = 50000;
driver.init();
motor.linkDriver(&driver);
motor.linkSensor(&encoder);

motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
motor.controller = MotionControlType::angle;

motor.P_angle.P = 0.0;
//motor.P_angle.D = 0.6;
//motor.P_angle.I = 0.0;
//motor.P_angle.limit = 40;

//motor.PID_velocity.P = 0.05;
//motor.PID_velocity.I = 0.8;
motor.PID_velocity.output_ramp = 1500;
motor.PID_velocity.limit = 30;

motor.voltage_limit =6;
motor.current_limit = 0.60;
motor.LPF_velocity.Tf = 0.02;

motor.init();
motor.initFOC();
delay(1000);


Serial.println(F("Gimbal stabilization ready."));


}

// -------------------- MAIN LOOP --------------------
void loop() {
  motor.loopFOC();
  motor.move();
  readMAVLinkNonBlocking();

  imu.update();
  rollAngle = imu.getAngleZ();

  // Apply gimbal-to-vehicle offset for stabilization
  float targetAngle = -rollAngle + mount_offset_roll;
  motor.target = targetAngle;



// Debug at ~2Hz
  static uint32_t debugT = 0;
  if (millis() - debugT > 500) {
    debugT = millis();
    Serial.print("Encoder: "); Serial.print(encoder.getAngle());
    Serial.print(F(" | GimbalRoll: ")); Serial.print(gimbalRoll);
    Serial.print(F(" | VehicleRoll: ")); Serial.print(vehicleRoll);
    Serial.print(F(" | Offset: ")); Serial.print(mount_offset_roll);
    Serial.print(F(" | IMU: ")); Serial.println(imu.getAngleZ());
  }

}