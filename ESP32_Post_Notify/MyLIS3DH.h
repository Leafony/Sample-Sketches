#ifndef MyLIS3DH_H
#define MyLIS3DH_H

#include "Arduino.h"
#include "Adafruit_LIS3DH.h"

class MyLIS3DH : public Adafruit_LIS3DH {
    public:
    void intrWhenDetect6dOrientation();
    uint8_t getRegINT1SRC();
};

#endif