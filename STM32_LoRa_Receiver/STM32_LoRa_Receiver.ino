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
#define LORA_RF_IRQ D5

// LoRa周波数
// 日本国内で使用する場合は必ず923MHzを使用してください。
#define LORA_FREQUENCY 923E6        // AS923 https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#frequency
#define LORA_SPREADING_FACTOR 7     // 6-12の間で設定可能 https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#spreading-factor
#define LORA_SIGNAL_BANDWIDTH 125E3 // https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#signal-bandwidth
#define LORA_GAIN 0                 // 0-6の間で設定可能 https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#lna-gain

void SystemClock_Config(void);
void resetLoRa(bool path_through);

const byte localAddress = 0xFF;

TCA9536 io(TCA9536_Address_t::TCA9536_ADDRESS);

/**
 * @brief 受信時に呼ばれるハンドラ
 *
 * @param packetSize
 */
void onReceive(int packetSize)
{
  if (packetSize == 0)
  {
    Serial.println("no packet\r");
    return; // if there's no packet, return
  }
  int recipient = LoRa.read();
  String incoming = "";
  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }
  if (recipient != localAddress && recipient != 0xFF)
  {
    Serial.println("This message is not for me.");
    return;
  }
  Serial.print(incoming);
  Serial.print(",");
  Serial.println(LoRa.packetRssi());
}

/**
 * @brief LoRa初期化
 *
 * @param path_through パススルーモード有効化
 */
void resetLoRa(bool paththrough = false)
{
  // SPIパススルーモード
  pinMode(LORA_IRQ_DUMB, OUTPUT);
  if (paththrough)
  {
    digitalWrite(LORA_IRQ_DUMB, LOW);
  }
  else
  {
    digitalWrite(LORA_IRQ_DUMB, HIGH);
  }
  delay(100);

  // Hardware reset
  io.write(LORA_BOOT0, LOW);

  delay(1000);
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

  Serial.println("LoRa Receiver Basic");

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

  while (LoRa.begin((long)LORA_FREQUENCY) == false)
  {
    Serial.println("Starting LoRa failed!\r");
    delay(1000);
  }

  // LoRa通信設定
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setGain(LORA_GAIN);
  LoRa.enableCrc();

  // register the receive callback
  LoRa.onReceive(onReceive);

  // put the radio into receive mode
  LoRa.receive();

  Serial.println("Receiver start\r");
}

void loop()
{
  // do noting
  delay(1000);
}
