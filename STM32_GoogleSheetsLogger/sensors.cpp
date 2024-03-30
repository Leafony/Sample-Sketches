#include "sensors.h"

ClosedCube_OPT3001 illum;

void initSensors()
{
  OPT3001_Config illumConfig;
  OPT3001_ErrorCode illumErrorConfig;

  Wire.begin();
  illum.begin(OPT3001_ADDRESS);
  smeHumidity.begin();

  illumConfig.RangeNumber = B1100;             // automatic full scale
  illumConfig.ConvertionTime = B1;             // convertion time = 800ms
  illumConfig.ModeOfConversionOperation = B11; // continous conversion
  illumConfig.Latch = B0;                      // hysteresis-style
  illumErrorConfig = illum.writeConfig(illumConfig);
  if (illumErrorConfig != NO_ERROR)
  {
    illumErrorConfig = illum.writeConfig(illumConfig); // retry
  }
}

float getTemperature()
{
  // sensor calibration data
  float TL0 = 25.0;
  float TM0 = 25.0;
  float TL1 = 40.0;
  float TM1 = 40.0;

  // read temperature
  float temperature = (float)smeHumidity.readTemperature();
  // calibrate temperature
  temperature = TM0 + (TM1 - TM0) * (temperature - TL0) / (TL1 - TL0);
  return temperature;
}

float getHumidity()
{
  // sensor calibration data
  float HL0 = 60.0;
  float HM0 = 60.0;
  float HL1 = 80.0;
  float HM1 = 80.0;

  // read temperature
  float humidity = (float)smeHumidity.readHumidity();
  // calibrate temperature
  humidity = HM0 + (HM1 - HM0) * (humidity - HL0) / (HL1 - HL0);
  return humidity;
}

float getIlluminance()
{
  OPT3001 result = illum.readResult();
  float illumination;

  if (result.error == NO_ERROR)
  {
    illumination = result.lux;
  }
  else
  {
    illumination = -99.9;
  }
  return illumination;
}