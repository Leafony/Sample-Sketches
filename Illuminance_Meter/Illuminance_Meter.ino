//=====================================================================
//  Illuminance meter
//
//    (c) 2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01  First release
//=====================================================================
#include <Wire.h>
#include <ClosedCube_OPT3001.h>

#define OPT3001_ADDRESS 0x45      // ADDR pin = VCC

ClosedCube_OPT3001 illum;

float dataIllum;

void setup() {
  // initialize serial communication at 115200 second per second:
  Serial.begin(115200);
  // initialize i2c communication with OPT3001:
  Wire.begin();

  delay(10);

  OPT3001_Config illumConfig;
  OPT3001_ErrorCode illumErrorConfig;

  illum.begin(OPT3001_ADDRESS);

  illumConfig.RangeNumber = B1100;               // automatic full scale
  illumConfig.ConvertionTime = B1;               // convertion time = 800ms
  illumConfig.ModeOfConversionOperation = B11;   // continous conversion
  illumConfig.Latch = B0;                        // hysteresis-style

  illumErrorConfig = illum.writeConfig(illumConfig);

  if(illumErrorConfig != NO_ERROR){
    illumErrorConfig = illum.writeConfig(illumConfig);   //retry
  }
}

void loop() {
    OPT3001 result = illum.readResult();

    if(result.error == NO_ERROR){
      dataIllum = result.lux;
    }

    Serial.println("Lum[lx] = " + String(dataIllum));

    delay(1000);

}
