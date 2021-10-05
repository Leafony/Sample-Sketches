#include "MyLIS3DH.h"

/*!
 *  @brief  configure LIS3DH to generate interrupt signal on INT1 when
 *          detecting 6D movement.
 *  @param None
 */
void MyLIS3DH::intrWhenDetect6dOrientation() {
    // Set LIS3DH to low power mode with ODR = 10Hz.
    writeRegister8(LIS3DH_REG_CTRL1, 0x2C);
    delay(10);

    // No high pass filters
    writeRegister8(LIS3DH_REG_CTRL2, 0x00);
    delay(10);

    // AOI1 interrupt generation is routed to INT1 pin.
    writeRegister8(LIS3DH_REG_CTRL3, 0x40); 
    delay(10);

    // FS = ±2g
    writeRegister8(LIS3DH_REG_CTRL4, 0x00); 
    delay(10);

    // Interrupt signal on INT1 pin is not latched.
    // If LIR_INT1 is enabled, an interrupt from AOI1 makes 
    // INT1 pin will go high from low and stay high.
    // Reading the INT1_SRC(31h) register will clear
    // the interrupt signal on INT1 pin.
    writeRegister8(LIS3DH_REG_CTRL5, 0x00); 
    delay(10);

    // Threshold = 11LSBs * 15.625mg/LSB = 171.875mg.
    writeRegister8(LIS3DH_REG_INT1THS, 0x0B);
    delay(10);

    // Duration = 3LSBs * (1/10Hz) = 0.3s. 1LSB = 1/ODR = 100msec. If the
    // X, Y, or Z axis exceeds thresholds for longer than 0.3s
    // duration, then the interrupt will be generated. Duration = 0 means
    // that the interrupt will be generated immediately.
    writeRegister8(LIS3DH_REG_INT1DUR, 0x03); 
    delay(10);

    // 6D movement detection with ZUPE, ZDOWNE, YUPE, YDOWNE, XUPE
    // and XDOWNE bits enabled.
    // [AOI 6D ZHIE ZLIE YHIE YLIE XHIE XLIE]
    writeRegister8(LIS3DH_REG_INT1CFG, 0x70); 
    delay(10);
}

/*!
 *  @brief  get data in the register INT1SRC
 *  @param None
 */
uint8_t MyLIS3DH::getRegINT1SRC() {
    return readRegister8(LIS3DH_REG_INT1SRC);
}

