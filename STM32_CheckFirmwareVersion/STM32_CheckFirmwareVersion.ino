/*
 * This example check if the firmware loaded on the WiFi101
 * shield is updated.
 *
 * Circuit:
 * - WiFi101 Shield attached
 *
 * Created 29 July 2015 by Cristian Maglie
 * This code is in the public domain.
 */
#include <Arduino.h>
#include <SPI.h>
#include <WiFi101Leafony.h>
#include <driver/source/nmasic.h>
#include <TCA9536.h>

// WiFi101LeafonyをSTM32で動作させる場合に必要な記述
extern "C" void attachInterruptMultiArch(uint32_t pin, void *chip_isr, uint32_t mode)
{
  void (*_c)(void) = (void (*)(void))(chip_isr);
  attachInterrupt(pin, _c, mode);
}
extern "C" void detachInterruptMultiArch(uint32_t pin)
{
  detachInterrupt(pin);
}

/********************
 * IOエキスパンダー
 ********************/
#define IOEX_CHIP_EN_PIN 0
#define IOEX_WAKE_PIN 1

TCA9536 io;

void setup()
{
  // Initialize serial
  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();

  // Initialize the PCA9536 with a begin function
  if (io.begin() == false)
  {
    Serial.println("TCA9536 not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }

  // WiFi用のピンアサインを設定
  WiFi.setPins(D10, D5, A4, -1); // CS = D10, IRQ = D5, RESET_N = A4, CHIP_EN = N.A.
  // WiFi用SPIの通信速度を設定
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  // WAKEを初期化
  io.pinMode(IOEX_WAKE_PIN, OUTPUT);
  io.write(IOEX_WAKE_PIN, HIGH);

  // CHIP_ENを初期化
  io.pinMode(IOEX_CHIP_EN_PIN, OUTPUT);
  io.write(IOEX_CHIP_EN_PIN, HIGH);

  // Print a welcome message
  Serial.println("WiFi101 firmware check.");
  Serial.println();

  // Check for the presence of the shield
  Serial.print("WiFi101 shield: ");
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("NOT PRESENT");
    return; // don't continue
  }
  Serial.println("DETECTED");

  // Print firmware version on the shield
  String fv = WiFi.firmwareVersion();
  String latestFv;
  Serial.print("Firmware version installed: ");
  Serial.println(fv);

  if (REV(GET_CHIPID()) >= REV_3A0)
  {
    // model B
    latestFv = WIFI_FIRMWARE_LATEST_MODEL_B;
  }
  else
  {
    // model A
    latestFv = WIFI_FIRMWARE_LATEST_MODEL_A;
  }

  // Print required firmware version
  Serial.print("Latest firmware version available : ");
  Serial.println(latestFv);

  // Check if the latest version is installed
  Serial.println();
  if (fv >= latestFv)
  {
    Serial.println("Check result: PASSED");
  }
  else
  {
    Serial.println("Check result: NOT PASSED");
    Serial.println(" - The firmware version on the shield do not match the");
    Serial.println("   version required by the library, you may experience");
    Serial.println("   issues or failures.");
  }
}

void loop()
{
  // do nothing
}
