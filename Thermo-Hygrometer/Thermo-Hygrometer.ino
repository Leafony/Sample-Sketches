//=====================================================================
//  Thermo-Hygrometer
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/05/05  First release
//=====================================================================
#include <Wire.h>
#include <HTS221.h>

//---------------------------
// 2点補正用データ
//---------------------------
// 温度補正用データ0
float TL0 = 25.0;     // 4-Sensors温度測定値
float TM0 = 25.0;     // 温度計等測定値
// 温度補正用データ1
float TL1 = 40.0;     // 4-Sensors温度測定値
float TM1 = 40.0;     // 温度計等測定値

// 湿度補正用データ0
float HL0 = 60.0;     // 4-Sensors湿度測定値
float HM0 = 60.0;     // 湿度計等測定値
// 湿度補正用データ1
float HL1 = 80.0;     // 4-Sensors湿度測定値
float HM1 = 80.0;     // 湿度計等測定値

void setup() {
  // initialize serial communication at 115200 second per second:
  Serial.begin(115200);
  // initialize i2c communication with HTS221:
  smeHumidity.begin();
  delay(10);
}

void loop() {
  // read temperature and humidity:
  float dataTemp = (float)smeHumidity.readTemperature();
  float dataHumid = (float)smeHumidity.readHumidity();

  // calibration:
  dataTemp = TM0 + (TM1 - TM0) * (dataTemp - TL0) / (TL1 - TL0);      // 温度補正
  dataHumid = HM0 + (HM1 - HM0) * (dataHumid - HL0) / (HL1 - HL0);    // 湿度補正

  Serial.println(String(dataTemp) + "[℃], " + String(dataHumid) + "[%]");
  delay(1000);
}