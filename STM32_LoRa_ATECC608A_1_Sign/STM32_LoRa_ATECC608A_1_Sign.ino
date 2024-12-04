//=====================================================================
//    (c) 2024 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2024/10/29  First release
//=====================================================================
//
// @seealso https://github.com/sparkfun/SparkFun_ATECCX08a_Arduino_Library/blob/master/examples/Example2_Sign/Example2_Sign.ino
//
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>

ATECCX08A atecc = ATECCX08A();

// array to hold our 32 bytes of message. Note, it must be 32 bytes, no more or less.
uint8_t message[32] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};

void printMessage();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial); // Wait for serial monitor to open

  // Check to see if the ATECCX08A is connected
  if (atecc.begin() == true)
  {
    Serial.println("Successful wakeUp(). I2C connections are good.");
  }
  else
  {
    Serial.println("Device not found. Check wiring.");
    while (1); // stall out forever
  }

  printMessage();

  // you should configure the device before getting public key
  // see also: atecc.writeConfigSparkFun()
  atecc.readConfigZone(false); // Debug argument false (OFF)
  if (!(atecc.configLockStatus && atecc.dataOTPLockStatus && atecc.slot0LockStatus))
  {
    Serial.print("Device not configured. Please use the configuration sketch.");
    while (1); // stall out forever.
  }

  if(atecc.generatePublicKey() == false)
  {
    Serial.println("Failure to generate This device's Public Key");
    while (1); // stall out forever.
  }
  // you can now use the public key from atecc.publicKey64Bytes[] array

  // create a signature from the message array
  atecc.createSignature(message);
  Serial.println("Signature created. Check the serial monitor for the signature.");
  // you can now use the signature from atecc.signature[] array
}

void loop() {
  // do nothing
}

// Function to print the message array in a format
void printMessage()
{
  Serial.println("uint8_t message[32] = {");
  for (int i = 0; i < sizeof(message) ; i++)
  {
    Serial.print("0x");
    if ((message[i] >> 4) == 0) Serial.print("0"); // print preceeding high nibble if it's zero
    Serial.print(message[i], HEX);
    if (i != 31) Serial.print(", ");
    if ((31 - i) % 16 == 0) Serial.println();
  }
  Serial.println("};");
}
