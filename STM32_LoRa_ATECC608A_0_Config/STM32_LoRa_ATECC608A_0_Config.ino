//=====================================================================
//    (c) 2023 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2024/10/29  First release
//=====================================================================
//
// @seealso https://github.com/sparkfun/SparkFun_ATECCX08a_Arduino_Library/blob/master/examples/Example1_Configuration/Example1_Configuration.ino
//
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>

ATECCX08A atecc = ATECCX08A();

void printInfo();

void setup() {
  Wire.begin();
  Serial.begin(115200);

  if (atecc.begin() == true) {
    Serial.println("Successful wakeUp(). I2C connections are good.");
  } else {
    Serial.println("Device not found. Check wiring.");
    while (1);
  }

  printInfo(); // see function below for library calls and data handling

  Serial.println("Would you like to configure your Cryptographic Co-processor with SparkFun Standard settings? (y/n)");
  Serial.println("***Note, this is PERMANENT and cannot be changed later***");
  Serial.println("***If you do not want to do this, type an 'n' or unplug now.***");

  while (Serial.available() == 0); // wait for user input

  if (Serial.read() == 'y')
  {
    Serial.println();
    Serial.println("Configuration beginning.");

    Serial.print("Write Config: \t");
    if (atecc.writeConfigSparkFun() == true) Serial.println("Success!");
    else Serial.println("Failure.");

    Serial.print("Lock Config: \t");
    if (atecc.lockConfig() == true) Serial.println("Success!");
    else Serial.println("Failure.");

    Serial.print("Key Creation: \t");
    if (atecc.createNewKeyPair() == true) Serial.println("Success!");
    else Serial.println("Failure.");

    Serial.print("Lock Data-OTP: \t");
    if (atecc.lockDataAndOTP() == true) Serial.println("Success!");
    else Serial.println("Failure.");

    Serial.print("Lock Slot 0: \t");
    if (atecc.lockDataSlot0() == true) Serial.println("Success!");
    else Serial.println("Failure.");

    Serial.println("Configuration done.");
    Serial.println();
  }
  else
  {
    Serial.println("Unfortunately, you cannot use any features of the ATECCX08A without configuration and locking.");
  }

  printInfo(); // see function below for library calls and data handling

  delay(10);
}

void loop(){
  if (atecc.wakeUp()) {
    Serial.println("Wake Up!");
  } else {
    Serial.println("Failed to wake up");
  }
  delay(100000);
}

void printInfo()
{
  // Read all 128 bytes of Configuration Zone
  // These will be stored in an array within the instance named: atecc.configZone[128]
  atecc.readConfigZone(false); // Debug argument false (OFF)

  // Print useful information from configuration zone data
  Serial.println();

  Serial.print("Serial Number: \t");
  for (int i = 0 ; i < 9 ; i++)
  {
    if ((atecc.serialNumber[i] >> 4) == 0) Serial.print("0"); // print preceeding high nibble if it's zero
    Serial.print(atecc.serialNumber[i], HEX);
  }
  Serial.println();

  Serial.print("Rev Number: \t");
  for (int i = 0 ; i < 4 ; i++)
  {
    if ((atecc.revisionNumber[i] >> 4) == 0) Serial.print("0"); // print preceeding high nibble if it's zero
    Serial.print(atecc.revisionNumber[i], HEX);
  }
  Serial.println();

  Serial.print("Config Zone: \t");
  if (atecc.configLockStatus) Serial.println("Locked");
  else Serial.println("NOT Locked");

  Serial.print("Data/OTP Zone: \t");
  if (atecc.dataOTPLockStatus) Serial.println("Locked");
  else Serial.println("NOT Locked");

  Serial.print("Data Slot 0: \t");
  if (atecc.slot0LockStatus) Serial.println("Locked");
  else Serial.println("NOT Locked");

  Serial.println();

  // if everything is locked up, then configuration is complete, so let's print the public key
  if (atecc.configLockStatus && atecc.dataOTPLockStatus && atecc.slot0LockStatus)
  {
    if(atecc.generatePublicKey() == false)
    {
      Serial.println("Failure to generate This device's Public Key");
      Serial.println();
    }
  }
}
