#ifndef __CONFIG_H__
#define __CONFIG_H__

// Set to true to enable debugging output
#define DEBUG true
#define DEBUG_SERIAL \
  if (DEBUG)         \
  Serial

// システム起動間隔 (ms)
#define WAKEUP_INTERVAL_MS 1 * 10 * 1000 // 10秒

// ランダム送信間隔 (ms)
// LoRaの輻輳を避けるために付与するランダムな送信オフセット時間の最大値
#define RANDOM_INTERVAL_MS 1 * 1000 // 1秒

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
#define LORA_FREQUENCY 923E6 // AS923
// LoRa SF
#define LORA_SPREADING_FACTOR 7 // 6-12の間で設定可能 https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#spreading-factor
// LoRa Bandwidth
#define LORA_SIGNAL_BANDWIDTH 125E3 // https://github.com/sandeepmistry/arduino-LoRa/blob/master/API.md#signal-bandwidth
// LoRa送信電力
#define LORA_TX_POWER 17 // dBm

// I2Cアドレス
#define LIS2DH_ADDRESS 0x19  // Accelerometer (SD0/SA0 pin = VCC)
#define OPT3001_ADDRESS 0x45 // Ambient Light Sensor (ADDR pin = VCC)
#define BATT_ADC_ADDR 0x50   // Battery ADC

//---------------------------
// Data for two-point correction
//---------------------------
// Temperature correction data 0
const float TL0 = 25.0; // 4-Sensors Temperature measurement value
const float TM0 = 25.0; // Thermometer and other measurements value
// Temperature correction data 1
const float TL1 = 40.0; // 4-Sensors Temperature measurement value
const float TM1 = 40.0; // Thermometer and other measurements value

// Humidity correction data 0
const float HL0 = 60.0; // 4-Sensors Humidity measurement value
const float HM0 = 60.0; // Hygrometer and other measurements value
// Humidity correction data 1
const float HL1 = 80.0; // 4-Sensors Humidity measurement value
const float HM1 = 80.0; // Hygrometer and other measurements value

#endif // __CONFIG_H__
