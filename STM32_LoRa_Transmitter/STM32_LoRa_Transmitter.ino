#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TCA9536.h>
#include <LoRa.h>
#include <SoftwareSerial.h>

// IOエキスパンダーのピン番号
#define IOEX_PIN_0 0
#define IOEX_PIN_1 1
#define IOEX_PIN_2 2
#define IOEX_PIN_3 3

// LoRaのピン番号
#define LORA_BOOT0 IOEX_PIN_0
#define LORA_RESET IOEX_PIN_2
#define LORA_IRQ_DUMB 10  // D10 に変更（Arduinoのピン定義に合わせる）

// LoRa周波数
#define LORA_FREQUENCY 923E6        // 日本向け AS923
#define LORA_SPREADING_FACTOR 7     // 拡散率（6-12の範囲）
#define LORA_SIGNAL_BANDWIDTH 125E3 // 信号帯域幅
#define LORA_GAIN 0                 // ゲイン（0-6の範囲）

void resetLoRa(bool path_through = false);

int counter = 0;

TCA9536 io(TCA9536_Address_t::TCA9536_ADDRESS);

/**
 * @brief LoRa初期化
 *
 * @param path_through パススルーモード有効化
 */
void resetLoRa(bool path_through)
{
  // SPIパススルーモード
  pinMode(LORA_IRQ_DUMB, OUTPUT);
  digitalWrite(LORA_IRQ_DUMB, path_through ? LOW : HIGH);
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

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Transmitter Example");

  Wire.begin();

  // IOエキスパンダーを初期化
  if (!io.begin())
  {
    Serial.println("TCA9536 not detected. Please check wiring. Freezing...");
    while (1);
  }

  io.disablePullUp(true);
  io.pinMode(LORA_RESET, OUTPUT);
  io.pinMode(LORA_BOOT0, OUTPUT);
  io.write(LORA_BOOT0, HIGH);

  // LoRaをリセット
  resetLoRa(false);
  resetLoRa(true);

  // SPI通信開始
  SPI.begin();

  // LoRaを初期化
  if (!LoRa.begin(LORA_FREQUENCY))
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // LoRa通信設定（`LoRa.begin()` の後に適用）
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setGain(LORA_GAIN);
  LoRa.enableCrc();
}

void loop()
{
  Serial.print("Sending packet: ");
  Serial.println(counter);

  LoRa.beginPacket();
  LoRa.write(0xFF);
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  Serial.println("end");

  counter++;

  delay(1000);
}
