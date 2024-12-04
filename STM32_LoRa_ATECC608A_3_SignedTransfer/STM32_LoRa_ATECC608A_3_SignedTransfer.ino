//=====================================================================
//    (c) 2024 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2024/11/07  First release
//=====================================================================
//
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TCA9536.h>
#include <LoRa.h>
#include <SoftwareSerial.h>
#include <SparkFun_ATECCX08a_Arduino_Library.h>

// IOエキスパンダーのピン番号
#define IOEX_PIN_0 0
#define IOEX_PIN_1 1
#define IOEX_PIN_2 2
#define IOEX_PIN_3 3

// LoRaのピン番号
#define LORA_BOOT0 IOEX_PIN_0
#define LORA_RESET IOEX_PIN_2
#define LORA_IRQ_DUMB D10

// LoRa周波数
// 日本国内で使用する場合は必ず923MHzを使用してください。
#define LORA_FREQUENCY 923E6        // AS923 https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#frequency
#define LORA_SPREADING_FACTOR 7     // 6-12の間で設定可能 https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#spreading-factor
#define LORA_SIGNAL_BANDWIDTH 125E3 // https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#signal-bandwidth
#define LORA_GAIN 0                 // 0-6の間で設定可能 https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#lna-gain

void SystemClock_Config(void);
void resetLoRa(bool path_through);

int counter = 0;

TCA9536 io(TCA9536_Address_t::TCA9536A_ADDRESS);

ATECCX08A atecc = ATECCX08A();

// 送信する32バイトのメッセージ
uint8_t message[32] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F
};

/**
 * @brief LoRa初期化
 *
 * @param path_through パススルーモード有効化
 */
void resetLoRa(bool path_through = false)
{
  // SPIパススルーモード
  pinMode(LORA_IRQ_DUMB, OUTPUT);
  if (path_through)
  {
    digitalWrite(LORA_IRQ_DUMB, LOW);
  }
  else
  {
    digitalWrite(LORA_IRQ_DUMB, HIGH);
  }
  delay(100);

  // ハードウェアリセット
  io.write(LORA_BOOT0, LOW);

  io.write(LORA_RESET, HIGH);
  delay(200);
  io.write(LORA_RESET, LOW);
  delay(200);
  io.write(LORA_RESET, HIGH);
  delay(50);
}

/**
 * @brief 公開鍵を表示
 *
 */
void printPublicKey()
{
  Serial.println("uint8_t publicKey[64] = {");
  for (int i = 0; i < sizeof(atecc.publicKey64Bytes) ; i++)
  {
    Serial.print("0x");
    if ((atecc.publicKey64Bytes[i] >> 4) == 0) Serial.print("0");
    Serial.print(atecc.publicKey64Bytes[i], HEX);
    if (i != 63) Serial.print(", ");
    if ((63 - i) % 16 == 0) Serial.println();
  }
  Serial.println("};");
}

/**
 * @brief 送信メッセージを表示
 *
 */
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

/**
 * @brief 署名を表示
 *
 */
void printSignature()
{
  Serial.println("uint8_t signature[64] = {");
  for (int i = 0; i < sizeof(atecc.signature) ; i++)
  {
    Serial.print("0x");
    if ((atecc.signature[i] >> 4) == 0) Serial.print("0");
    Serial.print(atecc.signature[i], HEX);
    if (i != 63) Serial.print(", ");
    if ((63 - i) % 16 == 0) Serial.println();
  }
  Serial.println("};");
  Serial.println();
}

void setup()
{
  // CPUスピードを80MHz→16MHzに変更
  // 低速にすることでLoRaのSPI通信速度を下げることができ、
  // 安定して通信することができる
  SystemClock_Config();

  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("LoRa Transmitter Example");

  Wire.begin();

  // IOエキスパンダーを初期化
  if (io.begin() == false)
  {
    Serial.println("TCA9536 not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }
  io.disablePullUp(true);
  io.pinMode(LORA_RESET, OUTPUT);
  io.pinMode(LORA_BOOT0, OUTPUT);
  io.write(LORA_BOOT0, HIGH);

  // LoRaをリセット
  resetLoRa(false);
  resetLoRa(true);

  // LoRa通信設定
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setGain(LORA_GAIN);
  LoRa.enableCrc();

  // LoRaを初期化
  if (LoRa.begin(LORA_FREQUENCY) == false)
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  // ATECCX08Aを初期化
  if (atecc.begin() == false)
  {
    Serial.println("ATECCX08A not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }

  // ATECCX08Aの設定チェック
  atecc.readConfigZone(false);
  if (!(atecc.configLockStatus && atecc.dataOTPLockStatus && atecc.slot0LockStatus))
  {
    Serial.println("ATECCX08A not configured. Please use the configuration sketch. Freezing...");
    while (1)
      ;
  }

  // 公開鍵を生成して署名
  if(atecc.generatePublicKey() == false)
  {
    Serial.println("Failure to generate This device's Public Key. Freezing...");
    while (1)
      ;
  }
  atecc.createSignature(message);
}

void loop()
{
  Serial.print("Sending packet: ");
  Serial.println(counter);

  LoRa.beginPacket();
  LoRa.write(0xFF);
  LoRa.write(atecc.publicKey64Bytes, 64);
  printPublicKey();
  LoRa.write(message, 32);
  printMessage();
  LoRa.write(atecc.signature, 64);
  printSignature();
  LoRa.endPacket();

  Serial.println("end");

  counter++;

  delay(30000);
}
