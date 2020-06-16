//=====================================================================
//  Accelerometer
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/05/05  First release
//=====================================================================
#include <Adafruit_LIS3DH.h>

#define LIS3DH_ADDRESS 0x19

Adafruit_LIS3DH accel = Adafruit_LIS3DH();

void setup() {
  // initialize serial communication at 115200 second per second:
  Serial.begin(115200);
  // initialize i2c communication with LIS3DH:
  accel.begin(LIS3DH_ADDRESS);

  accel.setClick(0, 0);                      // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);          // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_10_HZ);  // Data rate = 10Hz

  delay(100);
}

void loop() {
    accel.read();

    Serial.print("X [g] = " + String(accel.x_g));
    Serial.print(", ");
    Serial.print("Y [g] = " + String(accel.y_g));
    Serial.print(", ");
    Serial.print("Z [g] = " + String(accel.z_g));
    Serial.println("");

    delay(100);
}