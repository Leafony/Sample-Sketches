#include <Arduino.h>
#include <SPI.h>
#include <STM32LowPower.h>
#include <ArduinoHttpClient.h>
#include <WiFi101Leafony.h>
#include <driver/source/nmasic.h>
#include <TCA9536.h>
#include "arduino_secrets.h"
#include "sensors.h"

#include <Wire.h> // add ito

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
 * デバイス設定
 ********************/
const String deviceName = DEVICE_NAME;
const int interval = INTERVAL;

const int BATT_ADC_ADDR = 0x50; // add ito

/********************
 * Wi-Fi
 * SSIDとパスワードを src/arduino_secrets.h に記述してください
 ********************/
#define WIFI_CS_PIN D10
#define WIFI_IRQ_PIN D5
#define WIFI_RST_PIN A4

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
int status = WL_IDLE_STATUS; // the WiFi radio's status

const char server[] = "script.google.com";
WiFiClient client;

/********************
 * IOエキスパンダー
 ********************/
#define IOEX_CHIP_EN_PIN 0
#define IOEX_WAKE_PIN 1

TCA9536 io;

/********************
 * プロトタイプ宣言
 ********************/
void attachInterruptMultiArch(uint32_t pin, void *chip_isr, uint32_t mode);
void detachInterruptMultiArch(uint32_t pin);
void printWiFiStatus();
void wifiDozeMode();
void wifiPowerDown();
void logToGoogleSheets();

/**
 * @brief メイン初期化処理
 *
 */
void setup()
{
  // Initialize serial
  Serial.begin(115200);
#ifdef DEBUG
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif // DEBUG

  LowPower.begin(); // STM32 Low Power Libraryを初期化
  Wire.begin();     // I2Cを初期化

  initSensors(); // センサリーフを初期化

  // IOエキスパンダーを初期化
  if (io.begin() == false)
  {
    Serial.println("TCA9536 not detected. Please check wiring. Freezing...");
    while (1)
      ;
  }
  io.pinMode(IOEX_WAKE_PIN, OUTPUT);
  io.pinMode(IOEX_CHIP_EN_PIN, OUTPUT);

  // WiFiピンを初期化
  WiFi.setPins(WIFI_CS_PIN, WIFI_IRQ_PIN, WIFI_RST_PIN, -1); //  WiFi用のピンアサインを設定: CS = D10, IRQ = D5, RESET_N = A4, CHIP_EN = N.A.
  SPI.setClockDivider(SPI_CLOCK_DIV8);                       // WiFi用SPIの通信速度を設定
  io.write(IOEX_WAKE_PIN, HIGH);                             // WAKEを初期化
  io.write(IOEX_CHIP_EN_PIN, HIGH);                          // CHIP_ENを初期化

  // Check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("WiFi101 shield: NOT PRESENT");
    while (true)
      ;
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED)
  {
#ifdef DEBUG
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
#endif // DEBUG
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // wait 5 seconds for connection:
    delay(5000);
  }

#ifdef DEBUG
  Serial.println("You're connected to the network");
  printWiFiStatus();
#endif // DEBUG
}

/**
 * @brief メインループ
 *
 */
void loop()
{


  // Google Sheetsにログを記録
  logToGoogleSheets();

#ifdef DOZE_MODE
  // WiFiをDozeモードに移行
  wifiDozeMode();
  // STM32をスリープ
  Serial.flush();
  LowPower.deepSleep(interval * 1000);
#else
  delay(100);
  // WiFiとSTM32の電源を切り消費電力を最低にする
  wifiPowerDown();
  Serial.flush();
  LowPower.shutdown(interval * 1000);
#endif // DOZE_MODE
}

/**
 * @brief
 *
 */
void printWiFiStatus()
{
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi 101 Shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/**
 * @brief Dozeモードに移行
 * @details WiFiモジュールのコンフィギュレーションを維持したまま消費電流400uA未満の低電力モードに移行する
 */
void wifiDozeMode()
{
#ifdef DEBUG
  Serial.println("WiFi is going to Doze Mode");
#endif // DEBUG
  WiFi.maxLowPowerMode();
}

/**
 * @brief WiFiモジュールの電源を切る
 * @details WiFiモジュールの電源を切り、消費電流を4uA未満の最小限に抑える
 */
void wifiPowerDown()
{
#ifdef DEBUG
  Serial.println("WiFi is going to Power Down Mode");
#endif // DEBUG
  io.write(IOEX_CHIP_EN_PIN, LOW);
  digitalWrite(WIFI_RST_PIN, LOW);
}

/**
 * @brief Google Sheetsにログを記録
 */
void logToGoogleSheets()
{
  // センサーを読取り
  float temperature = getTemperature();
  float humidity = getHumidity();
  float illuminance = getIlluminance();

#if 1 // ito
  // read ADC registers:
  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR,2);

  uint8_t adcVal1 = Wire.read();
  uint8_t adcVal2 = Wire.read();

  // when ADC is not connected, read values are 0xFF:
  if (adcVal1 == 0xff && adcVal2 == 0xff) {
    adcVal1 = adcVal2 = 0;
  }

  // voltage mV = adcVal * Vref(3.3V) / resolution(8bit) * Vdiv(2)
  double tempMillivolt = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  float dataBatt = (float)(tempMillivolt / 1000);
#endif


#ifdef DEBUG
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, ");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, ");
  Serial.print("Illuminance: ");
  Serial.print(illuminance);
  Serial.println(" lx");
#if 1 // ito
  Serial.print("Battery: ");
  Serial.print(dataBatt);
  Serial.println(" V");

#endif

#endif // DEBUG

  String url = "/macros/s/";
  url += GAS_DEPLOY_ID;
  url += "/exec?";
  url += "UniqueID=";
  url += deviceName;
  url += "&temperature=";
  url += temperature;
  url += "&humidity=";
  url += humidity;
  url += "&illumination=";
  url += illuminance;
#if 1 // ito
  url += "&Battery=";
  url += dataBatt;
#endif

  if (client.connectSSL(server, 443))
  {
#ifdef DEBUG
    Serial.println("connected to server");
#endif // DEBUG

    client.print("GET ");
    client.print(url);
    client.println(" HTTP/1.1");
    client.println("Host: script.google.com");
    client.println("Connection: close");
    client.println();
    client.stop();

#ifdef DEBUG
    Serial.println("Request sent");
#endif // DEBUG
  }
}
