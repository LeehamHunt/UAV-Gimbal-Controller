// (snip includes same as yours)
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "mavlink/ardupilotmega/mavlink.h"

// motor/driver/enc same as your wiring
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(6, 7, 10, 3);
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

// IMU/MAVLink as before...
MPU6050 gyro(Wire);
HardwareSerial &fcSerial = Serial1;
float mount_offset_deg = 0;

void setup(){
  Serial.begin(115200);
  while(!Serial);
  fcSerial.begin(57600);
  Wire.begin();
  encoder.init();
  motor.linkSensor(&encoder);

  byte st = gyro.begin();
  while(st) { Serial.println("MPU init..."); delay(500); }
  gyro.calcOffsets();

  // driver config
  driver.voltage_power_supply = 12;
  driver.pwm_frequency = 20000; // try 16k/20k/25k/32k variations
  driver.init();

  motor.linkDriver(&driver);
  motor.controller = MotionControlType::angle;

  // Try Sine PWM first
  motor.foc_modulation = FOCModulationType::SinePWM;

  // safe conservative tuning
  motor.P_angle.P = 2.0;
  motor.P_angle.D = 0.1;
  motor.PID_velocity.P = 0.02;
  motor.PID_velocity.I = 0.2;

  motor.voltage_limit = 4.0;
  motor.current_limit = 0.4;
  motor.velocity_limit = 20;

  motor.init();
  motor.initFOC();

  Serial.println("FOC ready (SinePWM, safe limits).");
}

void loop(){
  gyro.update();
  float roll = gyro.getAngleZ();
  // read MAVLink omitted for brevity
  float target = -radians(roll) + radians(mount_offset_deg);

  motor.loopFOC();
  motor.move(target);

  static uint32_t t=0;
  if(millis()-t>500){
    t=millis();
    Serial.print("enc:"); Serial.print(encoder.getAngle());
    Serial.print(" roll:"); Serial.print(roll);
    Serial.print(" target(deg):"); Serial.println(mount_offset_deg);
  }
}
