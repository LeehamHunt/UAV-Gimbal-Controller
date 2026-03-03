#include <Arduino.h>
#include <mavlink.h>   // from okalachev/MAVLink library

#define SerialFC Serial1  // Xiao UART to FC

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // wait for Serial monitor to open
  Serial.print('1');
  SerialFC.begin(57600);     // FC TELEM baud
  Serial.println("MAVLink roll monitor ready");
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while (SerialFC.available()) {
    uint8_t c = SerialFC.read();
   
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(&msg, &att);

        Serial.print("Roll (deg): ");
        Serial.println(att.roll * 180.0 / M_PI);
      }
    }
  }
}
// void loop() {
//   if (SerialFC.available()) {
//     int c = SerialFC.read();
//     Serial.print("0x");
//     Serial.print(c, HEX);
//     Serial.print(' ');
//   }
// }