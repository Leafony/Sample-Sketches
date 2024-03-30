#include <TCA9536.h>

TCA9536::TCA9536()
{
  _i2cPort = NULL;
  _debugPort = NULL;
  _deviceAddress = TCA9536_ADDRESS_INVALID;
}

bool TCA9536::begin(TwoWire &wirePort)
{
  _deviceAddress = TCA9536_ADDRESS;
  _i2cPort = &wirePort;

  return (isConnected());
}

// Returns true if a device ack's at given address
boolean TCA9536::isConnected(void)
{
  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  return (_i2cPort->endTransmission() == 0);
}

void TCA9536::setDebugStream(Stream &debugPort)
{
  _debugPort = &debugPort;
}

TCA9536_error_t TCA9536::pinMode(uint8_t pin, uint8_t mode)
{
  TCA9536_error_t err;
  uint8_t cfgRegister = 0;

  if (pin > TCA9536_MAX_GPIO)
    return TCA9536_ERROR_UNDEFINED;

  err = readI2CRegister(&cfgRegister, TCA9536_REGISTER_CONFIGURATION);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  cfgRegister &= ~(1 << pin); // Clear pin bit
  if (mode == INPUT)          // Set the bit if it's being set to INPUT (opposite of Arduino)
  {
    cfgRegister |= (1 << pin);
  }
  return writeI2CRegister(cfgRegister, TCA9536_REGISTER_CONFIGURATION);
}

TCA9536_error_t TCA9536::write(uint8_t pin, uint8_t value)
{
  TCA9536_error_t err;
  uint8_t outputRegister = 0;

  if (pin > TCA9536_MAX_GPIO)
    return TCA9536_ERROR_UNDEFINED;

  err = readI2CRegister(&outputRegister, TCA9536_REGISTER_OUTPUT_PORT);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  // TODO: Break out of here if it's already set correctly
  outputRegister &= ~(1 << pin); // Clear pin bit
  if (value == HIGH)             // Set the bit if it's being set to HIGH (opposite of Arduino)
  {
    outputRegister |= (1 << pin);
  }
  return writeI2CRegister(outputRegister, TCA9536_REGISTER_OUTPUT_PORT);
}

TCA9536_error_t TCA9536::digitalWrite(uint8_t pin, uint8_t value)
{
  return write(pin, value);
}

uint8_t TCA9536::readReg()
{
  TCA9536_error_t err;
  uint8_t inputRegister = 0;

  err = readI2CRegister(&inputRegister, TCA9536_REGISTER_INPUT_PORT);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  return (inputRegister & (0x0f));
}

uint8_t TCA9536::read(uint8_t pin)
{
  TCA9536_error_t err;
  uint8_t inputRegister = 0;

  if (pin > TCA9536_MAX_GPIO)
    return TCA9536_ERROR_UNDEFINED;

  err = readI2CRegister(&inputRegister, TCA9536_REGISTER_INPUT_PORT);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  return (inputRegister & (1 << pin)) >> pin;
}

uint8_t TCA9536::digitalRead(uint8_t pin)
{
  return read(pin);
}

TCA9536_error_t TCA9536::invert(uint8_t pin, TCA9536_invert_t inversion)
{
  TCA9536_error_t err;
  uint8_t invertRegister = 0;

  if (pin > TCA9536_MAX_GPIO)
    return TCA9536_ERROR_UNDEFINED;

  err = readI2CRegister(&invertRegister, TCA9536_REGISTER_POLARITY_INVERSION);
  if (err != TCA9536_ERROR_SUCCESS)
  {
    return err;
  }
  // TODO: Break out of here if it's already set correctly
  invertRegister &= ~(1 << pin);   // Clear pin bit
  if (inversion == TCA9536_INVERT) // Set the bit if it's being set to inverted
  {
    invertRegister |= (1 << pin);
  }
  return writeI2CRegister(invertRegister, TCA9536_REGISTER_POLARITY_INVERSION);
}

TCA9536_error_t TCA9536::revert(uint8_t pin)
{
  return invert(pin, TCA9536_RETAIN);
}

TCA9536_error_t TCA9536::readI2CBuffer(uint8_t *dest, TCA9536_REGISTER_t startRegister, uint16_t len)
{
  if (_deviceAddress == TCA9536_ADDRESS_INVALID)
  {
    return TCA9536_ERROR_INVALID_ADDRESS;
  }
  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  _i2cPort->write(startRegister);
  if (_i2cPort->endTransmission(false) != 0)
  {
    return TCA9536_ERROR_READ;
  }

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len);
  for (int i = 0; i < len; i++)
  {
    dest[i] = _i2cPort->read();
  }

  return TCA9536_ERROR_SUCCESS;
}

TCA9536_error_t TCA9536::writeI2CBuffer(uint8_t *src, TCA9536_REGISTER_t startRegister, uint16_t len)
{
  if (_deviceAddress == TCA9536_ADDRESS_INVALID)
  {
    return TCA9536_ERROR_INVALID_ADDRESS;
  }
  _i2cPort->beginTransmission((uint8_t)_deviceAddress);
  _i2cPort->write(startRegister);
  for (int i = 0; i < len; i++)
  {
    _i2cPort->write(src[i]);
  }
  if (_i2cPort->endTransmission(true) != 0)
  {
    return TCA9536_ERROR_WRITE;
  }
  return TCA9536_ERROR_SUCCESS;
}

TCA9536_error_t TCA9536::readI2CRegister(uint8_t *dest, TCA9536_REGISTER_t registerAddress)
{
  return readI2CBuffer(dest, registerAddress, 1);
}

TCA9536_error_t TCA9536::writeI2CRegister(uint8_t data, TCA9536_REGISTER_t registerAddress)
{
  return writeI2CBuffer(&data, registerAddress, 1);
}
