//=====================================================================
//  Accelerometer Doubletap Interrupt
//
//    (c) 2021 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/05/12  First release
//=====================================================================

#include <Adafruit_LIS3DH.h>

#define LIS3DH_ADDRESS 0x19

// Adjust this number for the sensitivity of the 'click' force
// this strongly depend on the range! for 16G, try 5-10
// for 8G, try 10-20. for 4G try 20-40. for 2G try 40-80
#define CLICKTHRESHHOLD 80

#define SINGLETAP 1
#define DOUBLETAP 2

Adafruit_LIS3DH accel = Adafruit_LIS3DH();

void click() {
  Serial.println("Clicked!");
}

void setup() {
  // initialize serial communication at 115200 second per second:
  Serial.begin(115200);
  
  // initialize i2c communication with LIS3DH:
  accel.begin(LIS3DH_ADDRESS);
  
  accel.setRange(LIS3DH_RANGE_2_G);
  
  // 0 = turn off click detection & interrupt
  // 1 = single click only interrupt output
  // 2 = double click only interrupt output, detect single click
  // Adjust threshhold, higher numbers are less sensitive
  accel.setClick(DOUBLETAP, CLICKTHRESHHOLD);

  attachInterrupt(1, click, RISING);

  delay(100);

  accel.getClick();
}

void loop() {

}