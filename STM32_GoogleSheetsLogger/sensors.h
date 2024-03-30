#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>

#define OPT3001_ADDRESS 0x45

void initSensors();
float getTemperature();
float getHumidity();
float getIlluminance();

#endif // SENSORS_H