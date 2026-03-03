// (snip includes same as yours)
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "mavlink/ardupilotmega/mavlink.h"

// motor/driver/enc same as your wiring
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(2, 3, 9, 1);
MagneticSensorI2C encoder = MagneticSensorI2C(AS5600_I2C);

// IMU/MAVLink as before...
MPU6050 gyro(Wire);
HardwareSerial &fcSerial = Serial1;
float mount_offset_deg = 0;

void setup(){
//   // 3 synchronized PWM outputs (250 kHz) on D2, D3, D9 using TCC0 + TCC1
//   // 1. Setup GCLK4 as 48 MHz source
//   REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(4);
//   while (GCLK->STATUS.bit.SYNCBUSY);
//   REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN |
//                      GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
//   while (GCLK->STATUS.bit.SYNCBUSY);
//   // 2. Enable GCLK4 for TCC0 and TCC1
//   REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK4 | GCLK_CLKCTRL_ID_TCC0_TCC1;
//   while (GCLK->STATUS.bit.SYNCBUSY);
//   // 3. Configure pin multiplexing
//   // D2 (PA10) → TCC1/WO[0] (Function E)
//   PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
//   PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg |= PORT_PMUX_PMUXE_E;
//   // D3 (PA11) → TCC1/WO[1] (Function E)
//   PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;
//   PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
//   // D9 (PA07) → TCC0/WO[1] (Function E)
//   PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
//   PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E;
//   // 4. Configure TCC0
//   // Set PWM frequency
//   // pwm = 48MHz / 2 * (REG_TCC0_PER + 1)
//   // Yogurt Examples:
//   // Change REG_TCC0_PER to:
//   // 24 = 1 MHz
//   // 96 = 250 kHz
//   // 240 = 100 kHz
//   // 480 = 50 kHz
//   REG_TCC0_WAVE = TCC_WAVE_POL(0xF) | TCC_WAVE_WAVEGEN_DSBOTH;
//   while (TCC0->SYNCBUSY.bit.WAVE);
//   REG_TCC0_PER = 24488.79592;     // 48MHz / (2*96) = 250 kHz (dual-slope mode)
//   while (TCC0->SYNCBUSY.bit.PER);
//   REG_TCC0_CC1 = 48;
//   while (TCC0->SYNCBUSY.bit.CC1);
//   REG_TCC0_CC2 = 48;
//   while (TCC0->SYNCBUSY.bit.CC2);
//   // 5. Configure TCC1
//   REG_TCC1_WAVE = TCC_WAVE_POL(0x3) | TCC_WAVE_WAVEGEN_DSBOTH;
//   while (TCC1->SYNCBUSY.bit.WAVE);
//   REG_TCC1_PER = 24488.79592;     // Same frequency
//   while (TCC1->SYNCBUSY.bit.PER);
//   REG_TCC1_CC0 = 48;
//   while (TCC1->SYNCBUSY.bit.CC0);
//   REG_TCC1_CC1 = 48;
//   while (TCC1->SYNCBUSY.bit.CC1);
//   // 6. Enable both timers simultaneously
//   REG_TCC0_CTRLA = TCC_CTRLA_PRESCALER_DIV1;
//   REG_TCC1_CTRLA = TCC_CTRLA_PRESCALER_DIV1;
//   // Enable both
//   REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE;
//   REG_TCC1_CTRLA |= TCC_CTRLA_ENABLE;
//   while (TCC0->SYNCBUSY.bit.ENABLE || TCC1->SYNCBUSY.bit.ENABLE);
//   // Force synchronization restart for perfect phase alignment
//   REG_TCC0_CTRLBSET = TCC_CTRLBSET_CMD_RETRIGGER;
//   REG_TCC1_CTRLBSET = TCC_CTRLBSET_CMD_RETRIGGER;
//   while (TCC0->SYNCBUSY.bit.CTRLB || TCC1->SYNCBUSY.bit.CTRLB);


  
  
  
  
  
  
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
