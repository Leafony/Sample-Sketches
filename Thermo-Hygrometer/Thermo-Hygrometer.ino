//=====================================================================
//  Thermo-Hygrometer
//
//    (c) 2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01  First release
//=====================================================================
#include <Wire.h>
#include <HTS221.h>

//---------------------------
// Data for two-point correction
//---------------------------
// Temperature correction data 0
float TL0 = 25.0;     // 4-Sensors Temperature measurement value
float TM0 = 25.0;     // Thermometer and other measurements value
// Temperature correction data 1
float TL1 = 40.0;     // 4-Sensors Temperature measurement value
float TM1 = 40.0;     // Thermometer and other measurements value

// Humidity correction data 0
float HL0 = 60.0;     // 4-Sensors Humidity measurement value
float HM0 = 60.0;     // Hygrometer and other measurements value
// Humidity correction data 1
float HL1 = 80.0;     // 4-Sensors Humidity measurement value
float HM1 = 80.0;     // Hygrometer and other measurements value

void setup() {
  // initialize serial communication at 115200 second per second:
  Serial.begin(115200);
  Wire.begin();             // I2C 100kHz
  // initialize i2c communication with HTS221:
  smeHumidity.begin();
  delay(10);
}

void loop() {
  // read temperature and humidity:
  float dataTemp = (float)smeHumidity.readTemperature();
  float dataHumid = (float)smeHumidity.readHumidity();

  // calibration:
  dataTemp = TM0 + (TM1 - TM0) * (dataTemp - TL0) / (TL1 - TL0);      // Temperature correction
  dataHumid = HM0 + (HM1 - HM0) * (dataHumid - HL0) / (HL1 - HL0);    // Humidity correction

  Serial.println(String(dataTemp) + "[℃], " + String(dataHumid) + "[%]");
  delay(1000);
}
