//=====================================================================
//  Battery Voltage
//
//    (c) 2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01  First release
//=====================================================================

#include <Wire.h>

const int BATT_ADC_ADDR = 0x50;

void setup() {
  // initialize serial communication at 115200 second per second:
  Serial.begin(115200);
  // initialize I2C communication at 100kHz:
  Wire.begin();
  delay(10);
}

void loop(){
  // read ADC registers:
  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR,2);
  uint8_t adcVal1 = Wire.read();
  uint8_t adcVal2 = Wire.read();

  // when ADC is not connected, read values are 0xFF:
  if (adcVal1 == 0xff && adcVal2 == 0xff) {
    adcVal1 = adcVal2 = 0;
  }

  // voltage mV = adcVal * Vref(3.3V) / resolution(8bit) * Vdiv(2)
  double tempMillivolt = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  float dataBatt = (float)(tempMillivolt / 1000);

  Serial.println("Batt[V]  = " + String(dataBatt));
  delay(1000);
}
