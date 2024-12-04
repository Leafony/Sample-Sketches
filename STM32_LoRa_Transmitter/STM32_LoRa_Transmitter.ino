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
