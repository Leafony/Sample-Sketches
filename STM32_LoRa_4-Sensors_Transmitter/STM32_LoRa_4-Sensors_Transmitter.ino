#include <Arduino.h>
#include <IWatchdog.h>
#include <STM32LowPower.h>
#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TCA9536.h>
#include <LoRa.h>
#include <Adafruit_LIS3DH.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>

#include "config.h"

void SystemClock_Config(void);
void resetLoRa(bool path_through);
void sleepLoRa();
float readBatteryVoltage();
void initSensors();
void sleepSensors();
void wakeupSensors();
void sendPacket();

// IO Expander on LoRa Mary leaf
TCA9536 io(TCA9536_Address_t::TCA9536A_ADDRESS);

// Sensors
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;

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
 * @brief LoRaをスリープ
 *
 */
void sleepLoRa()
{
  // LoRaをスリープ
  LoRa.end();
  digitalWrite(LORA_IRQ_DUMB, LOW);
  io.write(LORA_BOOT0, LOW);
  io.write(LORA_RESET, LOW);

  // ピンを入力にして電流リークを防ぐ
  pinMode(LORA_IRQ_DUMB, INPUT);
  io.pinMode(LORA_RESET, INPUT);
  io.pinMode(LORA_BOOT0, INPUT);
}

/**
 * @brief バッテリ電圧読み出し
 * @details mV = adcVal * Vref(3.3V) / resolution(8bit) * Vdiv(2)
 * @return 2桁のHEX String
 */
float readBatteryVoltage()
{
  String strAdcVal;
  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR, 2);
  uint8_t adcVal1 = Wire.read();
  uint8_t adcVal2 = Wire.read();
  uint8_t adcVal = (adcVal1 << 4) | (adcVal2 >> 4);
  return (float)adcVal * 3.3 / 256.0 * 2.0;
}

/**
 * @brief センサの初期化
 *
 */
void initSensors()
{
  DEBUG_SERIAL.println("Initializing sensors...");

  // HTS221 (temperature /humidity)
  smeHumidity.begin();

  // OPT3001 (light)
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;
  light.begin(OPT3001_ADDRESS);
  newConfig.RangeNumber = B1100;             // automatic full scale
  newConfig.ConvertionTime = B0;             // conversion time = 100ms
  newConfig.ModeOfConversionOperation = B01; // single-shot conversion
  newConfig.Latch = B0;                      // hysteresis-style
  errorConfig = light.writeConfig(newConfig);

  // LIS2DH (accel)
  accel.begin(LIS2DH_ADDRESS);
  accel.setClick(0, 0);                     // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);         // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_10_HZ); // Data rate = 10Hz

  delay(100); // wait until all sensors are ready

  DEBUG_SERIAL.println("Sensors initialized.");
}

/**
 * @brief センサーリーフをスリープさせる
 *
 */
void sleepSensors()
{
  // HTS221 sleep
  smeHumidity.deactivate();

  // OPT3001 sleep
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;
  newConfig.ModeOfConversionOperation = B00;
  errorConfig = light.writeConfig(newConfig);

  // LIS2DH sleep
  accel.setDataRate(LIS3DH_DATARATE_POWERDOWN);
}

/**
 * @brief センサーリーフをスリープから復帰させる
 *
 */
void wakeupSensors()
{
  DEBUG_SERIAL.println(F("Wakeup Sensors."));

  // HTS221 wakeup
  smeHumidity.activate();

  // OPT3001 wakeup
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;
  newConfig.RangeNumber = B1100;             // automatic full scale
  newConfig.ModeOfConversionOperation = B01; // single-shot conversion
  errorConfig = light.writeConfig(newConfig);

  // LIS2DH wakeup
  accel.setDataRate(LIS3DH_DATARATE_10_HZ); // Data rate = 10Hz

  delay(300);
}

/**
 * @brief
 */
void sendPacket()
{
  // LIS2DH : accelerometer
  float x_g, y_g, z_g;

  // HTS221 : Temperature/Humidity
  float temp = 0;
  float humid = 0;

  // OPT3001 : illuminance
  float illum = 0;

  // battery
  float batt = 0;

  DEBUG_SERIAL.println("Sending packet... ");

  // センサを立ち上げ
  wakeupSensors();

  // 温湿度を読み出し
  smeHumidity.begin();
  temp = (float)smeHumidity.readTemperature();
  humid = (float)smeHumidity.readHumidity();
  temp = TM0 + (TM1 - TM0) * (temp - TL0) / (TL1 - TL0);   // 温度補正
  humid = HM0 + (HM1 - HM0) * (humid - HL0) / (HL1 - HL0); // 湿度補正
  if (humid > 100)
  {
    humid = 100;
  }

  // 照度読み出し
  OPT3001 result = light.readResult();
  illum = result.lux;

  // 加速度読み出し
  accel.read();
  x_g = accel.x_g; // X-axis
  y_g = accel.y_g; // Y-axis
  z_g = accel.z_g; // Z-axis

  // 電池電圧読み出し
  batt = readBatteryVoltage();

  // センサをスリープ
  sleepSensors();

  // LoRaでデータを送信
  LoRa.beginPacket();
  LoRa.write(0xFF);
  LoRa.print(batt);
  LoRa.print("V,");
  LoRa.print(temp);
  LoRa.print("℃,");
  LoRa.print(humid);
  LoRa.print("%,");
  LoRa.print(illum);
  LoRa.print("lx,");
  LoRa.print(x_g);
  LoRa.print("g,");
  LoRa.print(y_g);
  LoRa.print("g,");
  LoRa.print(z_g);
  LoRa.print("g");
  LoRa.endPacket();

  DEBUG_SERIAL.print(batt);
  DEBUG_SERIAL.print("V,");
  DEBUG_SERIAL.print(temp);
  DEBUG_SERIAL.print("℃,");
  DEBUG_SERIAL.print(humid);
  DEBUG_SERIAL.print("%,");
  DEBUG_SERIAL.print(illum);
  DEBUG_SERIAL.print("lx,");
  DEBUG_SERIAL.print(x_g);
  DEBUG_SERIAL.print("g,");
  DEBUG_SERIAL.print(y_g);
  DEBUG_SERIAL.print("g,");
  DEBUG_SERIAL.print(z_g);
  DEBUG_SERIAL.println("g");

  DEBUG_SERIAL.println("end!");
}

/**
 * @brief
 *
 */
void setup()
{
  // Initialize the IWDG with 10 seconds timeout.
  // This would cause a CPU reset if the IWDG timer
  // is not reloaded in approximately 10 seconds.
  IWatchdog.begin(10 * 1000 * 1000);

  // CPUスピードを80MHz→16MHzに変更
  // 低速にすることでLoRaのSPI通信速度を下げることができ、
  // 安定して通信することができる
  SystemClock_Config();

  // STM32LowPowerを初期化
  LowPower.begin();

  // 乱数設定
  randomSeed(analogRead(A0));

#if DEBUG == true
  // シリアルを初期化
  Serial.begin(115200);
  while (!Serial)
    ;
#endif // DEBUG

  DEBUG_SERIAL.println("Starting LoRa 4-Sensors Transmitter...");

  // I2Cを初期化
  Wire.begin();

  // IOエキスパンダーを初期化
  if (io.begin() == false)
  {
    DEBUG_SERIAL.println("TCA9536 not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }
  io.disablePullUp(true);
  io.pinMode(LORA_RESET, OUTPUT);
  io.pinMode(LORA_BOOT0, OUTPUT);
  io.write(LORA_BOOT0, HIGH);

  // LoRaを初期化
  resetLoRa(false);
  resetLoRa(true);
  if (LoRa.begin(LORA_FREQUENCY) == false)
  {
    DEBUG_SERIAL.println("Starting LoRa failed!");
    while (1)
      ;
  }

  // LoRa通信設定
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.enableCrc();               // CRCを有効化
  LoRa.setTxPower(LORA_TX_POWER); // 送信電力を設定

  // センサーを初期化
  initSensors();

  DEBUG_SERIAL.println("Successfully started!");
}

/**
 * @brief
 *
 */
void loop()
{
  // センサデータを送信
  sendPacket();

  // LoRaをスリープ
  sleepLoRa();

  // ランダムな時間待機を生成し、輻輳を避ける
  int randomInterval = random(RANDOM_INTERVAL_MS);

  // IWDGをリロード
  IWatchdog.reload();

  // システムを一定期間停止
  DEBUG_SERIAL.println("Restart after " + String(WAKEUP_INTERVAL_MS + randomInterval) + " ms...");
  DEBUG_SERIAL.flush();
  LowPower.shutdown(WAKEUP_INTERVAL_MS + randomInterval);
}
