#include <Arduino.h>
#include <SimpleFOC.h>
#include <CAN.h>


const uint32_t PIXHAWK_RPY_ID = 0x2;
const float INT16_MAX_DEG = 32767.0f;

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 9);
BLDCMotor motor = BLDCMotor(11, 5.7); // GM3506: 22 poles = 11 pole pairs
float last_target = 0;
float smoothed_roll = 0;
float current_target = 0;
unsigned long last_can_time = 0;

bool readRPY(float &rollT) {
  int packetSize = CAN.parsePacket();
  if (packetSize == 0 || CAN.packetId() != PIXHAWK_RPY_ID) return false;
  if (packetSize < 2) { while (CAN.available()) CAN.read(); return false; }
  int16_t r_i = (int16_t)((CAN.read() << 8) | CAN.read());
  while (CAN.available()) CAN.read();
  rollT = (float)r_i * 180.0f / INT16_MAX_DEG;
  return true;
}

void setup() {
  //Serial.begin(115200);
  CAN.setClockFrequency(8E6);
  if (!CAN.begin(500E3)) while (1);
  CAN.filter(PIXHAWK_RPY_ID);

  sensor.init();
  pinMode(8, OUTPUT); digitalWrite(8, HIGH);
  
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;
  driver.pwm_frequency = 25000; // Increase PWM freq to reduce buzzing
  driver.init();
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  
  // CONSERVATIVE BASELINE - Start here and tune up gradually
  // Angle (Position) Loop
  motor.P_angle.P = 11.0;       // START LOW - we'll increase slowly
  motor.P_angle.I = 0.000;       // No integral term yet
  motor.P_angle.D = .7;       // Moderate damping
  motor.P_angle.output_ramp = 0; 
  motor.P_angle.limit = 50;    // Conservative velocity limit
  
  // Velocity Loop - Keep this stable first
  motor.PID_velocity.P = 0.05;   // Low for stability
  motor.PID_velocity.I = 1.0;   // Moderate I term velocity tracking
  //motor.PID_velocity.D = 0.0;   // No D term in velocity loop
  motor.PID_velocity.output_ramp = 1500; 
  motor.PID_velocity.limit = 40;
  
  // Motor limits
  motor.voltage_limit = 12;
  motor.current_limit = 0.8;
  motor.velocity_limit = 40;    // Conservative limit
  
  // Low-pass filter - More filtering for stability
  motor.LPF_velocity.Tf = 0.02; // More filtering = smoother, less noise
  
  motor.init();
  motor.initFOC();
  delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();
  
  // output hz debug
  // static unsigned long msg_count = 0;
  // static unsigned long rate_timer = 0;

  float rollT = 0;
  if (readRPY(rollT)) {
    rollT = constrain(rollT, -90, 90)-35;
    float new_target = (rollT) * 0.06; // Gear ratio
    
    // Smooth exponential filtering instead of ramping
    // Alpha: 0.1 = very smooth, 0.5 = responsive, 1.0 = no filtering
    float alpha = 1; // Adjust between 0.1 (smooth) and 0.5 (fast)
    last_target = alpha * new_target + (1 - alpha) * last_target;
    
    motor.target = last_target;
    
    // output hz debug
    // msg_count++;
    // if (millis() - rate_timer > 1000) {
    // Serial.print("CAN Rate: ");
    // Serial.print(msg_count);
    // Serial.println(" Hz");
    // msg_count = 0;
    // rate_timer = millis();
    // }

    // Debugging output - optimized for plotter
    // static unsigned long lastPrint = 0;
    // if (millis() - lastPrint > 50) { // 20Hz update rate for smooth plotting
    //   lastPrint = millis();
    //   Serial.print("R:"); Serial.print(rollT, 2);
    //   Serial.print(" T:"); Serial.print(-last_target, 4);
    //   Serial.print(" A:"); Serial.print(sensor.getAngle(), 4);
    //   Serial.print(" E:"); Serial.print(last_target - sensor.getAngle(), 4);
    //   Serial.print(" V:"); Serial.print(motor.shaft_velocity, 2);
    //   Serial.print(" Vq:"); Serial.println(motor.voltage.q, 2);
    // }
  }
}