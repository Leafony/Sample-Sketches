#include <Wire.h>

typedef enum
{
  TCA9536_ADDRESS = 0x40,
  TCA9536_ADDRESS_INVALID = 0xFF
} TCA9536_Address_t;

typedef enum
{
  TCA9536_REGISTER_INPUT_PORT = 0x00,
  TCA9536_REGISTER_OUTPUT_PORT = 0x01,
  TCA9536_REGISTER_POLARITY_INVERSION = 0x02,
  TCA9536_REGISTER_CONFIGURATION = 0x03,
  TCA9536_REGISTER_INVALID
} TCA9536_REGISTER_t;

typedef enum
{
  TCA9536_ERROR_READ = -4,
  TCA9536_ERROR_WRITE = -3,
  TCA9536_ERROR_INVALID_ADDRESS = -2,
  TCA9536_ERROR_UNDEFINED = -1,
  TCA9536_ERROR_SUCCESS = 1
} TCA9536_error_t;
const TCA9536_error_t TCA9536_SUCCESS = TCA9536_ERROR_SUCCESS;

typedef enum
{
  TCA9536_RETAIN,
  TCA9536_INVERT,
  TCA9536_INVERT_END
} TCA9536_invert_t;

#define TCA9536_MAX_GPIO 3

class TCA9536
{
public:
  TCA9536();
  bool begin(TwoWire &wirePort = Wire);
  bool isConnected();

  // setDebugStream to enable library debug statements
  void setDebugStream(Stream &debugPort = Serial);

  // pinMode can set a pin (0-3) to INPUT or OUTPUT
  TCA9536_error_t pinMode(uint8_t pin, uint8_t mode);

  // digitalWrite and write can be used to set a pin HIGH or LOW
  TCA9536_error_t digitalWrite(uint8_t pin, uint8_t value);
  TCA9536_error_t write(uint8_t pin, uint8_t value);

  // readReg can be used to read the whole input register (4 bits)
  uint8_t readReg();

  // digitalRead and read can be used to read a pin (0-3)
  uint8_t digitalRead(uint8_t pin);
  uint8_t read(uint8_t pin);

  // invert and revert can be used to invert (or not) the I/O logic during a read
  TCA9536_error_t invert(uint8_t pin, TCA9536_invert_t inversion = TCA9536_INVERT);
  TCA9536_error_t revert(uint8_t pin);

private:
  TwoWire *_i2cPort; // The generic connection to user's chosen I2C hardware
  Stream *_debugPort;
  TCA9536_Address_t _deviceAddress;

  // I2C Read/Write
  TCA9536_error_t readI2CBuffer(uint8_t *dest, TCA9536_REGISTER_t startRegister, uint16_t len);
  TCA9536_error_t writeI2CBuffer(uint8_t *src, TCA9536_REGISTER_t startRegister, uint16_t len);
  TCA9536_error_t readI2CRegister(uint8_t *dest, TCA9536_REGISTER_t registerAddress);
  TCA9536_error_t writeI2CRegister(uint8_t data, TCA9536_REGISTER_t registerAddress);
};
