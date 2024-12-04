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

// copy result from the serial monitor and paste it here
uint8_t message[32] = {
0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 
0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};
uint8_t publicKey[64] = {
0x13, 0xA0, 0x26, 0xBB, 0x32, 0xC4, 0x8D, 0x5C, 0x7F, 0xF6, 0xBA, 0xC4, 0x4D, 0x6E, 0xDD, 0x8C, 
0x47, 0xD9, 0x02, 0xD3, 0x4A, 0xE4, 0x53, 0xCC, 0x99, 0x45, 0x85, 0x33, 0x85, 0x61, 0x6C, 0xDC, 
0x51, 0xAB, 0xDC, 0xA7, 0xE5, 0xCF, 0xC4, 0x18, 0xF5, 0xAD, 0x49, 0x6A, 0x2F, 0x07, 0x5C, 0x59, 
0xA8, 0xA0, 0x2C, 0x59, 0xC8, 0x71, 0x11, 0xFF, 0xEB, 0x20, 0x68, 0x42, 0x35, 0xA9, 0x9E, 0xBD
};
uint8_t signature[64] = {
0x75, 0x38, 0x7B, 0x58, 0xFD, 0x74, 0x76, 0x41, 0xA2, 0x1A, 0xF9, 0x8E, 0x81, 0xCA, 0xD9, 0xAB, 
0x0E, 0xE9, 0x3E, 0x65, 0xE8, 0xC7, 0x52, 0xE5, 0x60, 0xFA, 0x59, 0xB3, 0x59, 0x0D, 0x60, 0xCA, 
0xD6, 0x2D, 0xD3, 0x23, 0xF3, 0xB2, 0xF8, 0x20, 0xB7, 0xBC, 0x52, 0x5B, 0x1D, 0x96, 0x7C, 0x3B, 
0x28, 0x00, 0xA4, 0x4A, 0xAE, 0x44, 0x11, 0x71, 0xBF, 0x0D, 0x6F, 0xCC, 0xF3, 0xC8, 0xDD, 0x47
};

void printMessage();
void printPublicKey();
void printSignature();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while (!Serial)
    ;

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

  // you should configure the device before getting public key
  // see also: atecc.writeConfigSparkFun()
  atecc.readConfigZone(false); // Debug argument false (OFF)
  if (!(atecc.configLockStatus && atecc.dataOTPLockStatus && atecc.slot0LockStatus))
  {
    Serial.print("Device not configured. Please use the configuration sketch.");
    while (1); // stall out forever.
  }

  printMessage();
  printPublicKey();
  printSignature();

  // verify the signature
  if (atecc.verifySignature(message, signature, publicKey)) {
    Serial.println("Success! Signature Verified.");
  } else {
    Serial.println("Verification failure.");
  }
}

void loop() {
  // do nothing
}


void printMessage()
{
  Serial.println("uint8_t message[32] = {");
  for (int i = 0; i < sizeof(message) ; i++)
  {
    Serial.print("0x");
    if ((message[i] >> 4) == 0) Serial.print("0");
    Serial.print(message[i], HEX);
    if (i != 31) Serial.print(", ");
    if ((31 - i) % 16 == 0) Serial.println();
  }
  Serial.println("};");
}

void printPublicKey()
{
  Serial.println("uint8_t publicKeyExternal[64] = {");
  for (int i = 0; i < sizeof(publicKey) ; i++)
  {
    Serial.print("0x");
    if ((publicKey[i] >> 4) == 0) Serial.print("0");
    Serial.print(publicKey[i], HEX);
    if (i != 63) Serial.print(", ");
    if ((63 - i) % 16 == 0) Serial.println();
  }
  Serial.println("};");
  Serial.println();
}

void printSignature()
{
  Serial.println("uint8_t signature[64] = {");
  for (int i = 0; i < sizeof(signature) ; i++)
  {
    Serial.print("0x");
    if ((signature[i] >> 4) == 0) Serial.print("0");
    Serial.print(signature[i], HEX);
    if (i != 63) Serial.print(", ");
    if ((63 - i) % 16 == 0) Serial.println();
  }
  Serial.println("};");
  Serial.println();
}
