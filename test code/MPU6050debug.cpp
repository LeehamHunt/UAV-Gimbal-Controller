#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 gyro(Wire);
float rollAngle = 0.0;
void setup () {
    Serial.begin(115200);
    while (!Serial) { ; }  // wait for Serial monitor to open
    //Serial.print('1');
    Wire.begin();
    gyro.begin();
    delay(200);


}

void loop() {
    gyro.update();
    rollAngle = gyro.getAngleX();
    Serial.println(rollAngle);
    
}