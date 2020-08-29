#include <string>
#include "esp_pm.h"

#include "FS.h"
#include "SPIFFS.h"

#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLEBeacon.h"
#include "BLE2902.h"
// #include "WiFi.h"

#include <Wire.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>
#include <Adafruit_LIS3DH.h>

#include "esp_log.h"
static const char *TAG = "main";

// Pin Assign
static const uint8_t PUSH_BUTTON = 0;
static const uint8_t SDA_2 = 21;

// Sleep and Awake time
const int TIME_TO_SLEEP = 5; // Time ESP32 will go to sleep (in seconds)
const int TIME_TO_AWAKE = 2; // Time ESP32 will awake (in seconds)

// BLE variables
const char *SERVICE_UUID = "ec6ba320-e16b-11ea-87d0-0242ac130003";
const char *CHARACTERISTIC_UUID = "ec6ba321-e16b-11ea-87d0-0242ac130003";
const std::string deviceName = "Leaf_Z";
BLECharacteristic *pCharacteristic;
static bool deviceConnected = false;
static bool inLogDataTransporting = false;

// OPT3001 Illuminance Sensor
static const uint8_t OPT3001_ADDRESS = 0x45;
static ClosedCube_OPT3001 illum;

// Accel
Adafruit_LIS3DH accel = Adafruit_LIS3DH();

// Battery ADC
const int BATT_ADC_ADDR = 0x50;

// Sensors variables
static float dataTemp = 0;
static float dataHumid = 0;
static float dataIllum = 0;
static float dataBatt = 0;

// SPIFFS logging
const char* logpath = "/log.txt";

//*****************************************************************************
// Prototypes
//*****************************************************************************
void initPowerManager();
void enableSleep();
void startSleep();
void print_wakeup_reason();

void setupSensors();
void getSensors();
void sleepSensors();

void saveLogData();
void sendLogData();

void setupBLE();
void decodeBLECommand(std::string);
void sendLogData();

//*****************************************************************************
// ESP32 Power Management Functions
//*****************************************************************************
void initPowerManager() {
  // ESP32 power management settings
	esp_pm_config_esp32_t esp_pm_config_esp32;
	esp_pm_config_esp32.max_cpu_freq = RTC_CPU_FREQ_240M;
	esp_pm_config_esp32.min_cpu_freq = RTC_CPU_FREQ_XTAL;
	esp_pm_config_esp32.light_sleep_enable = true;
	esp_pm_configure(&esp_pm_config_esp32);

  // Turn off the Wi-Fi to save power
  // WiFi.mode(WIFI_OFF);
}

//*****************************************************************************
// ESP32 Deepsleep Functions
//*****************************************************************************
void enableSleep()
{
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * 1000000ULL); // microseconds
  ESP_LOGI(TAG, "Setup ESP32 to sleep for every %d Seconds", TIME_TO_SLEEP);
}

void startSleep()
{
  ESP_LOGI(TAG, "Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
  // below this sentence will never be called.
}

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    ESP_LOGI(TAG, "Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    ESP_LOGI(TAG, "Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    ESP_LOGI(TAG, "Wakeup caused by ULP program");
    break;
  default:
    ESP_LOGI(TAG, "Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    break;
  }
}

//*****************************************************************************
// AI01A Sensor Leaf Functions
//*****************************************************************************
void setupSensors()
{
  // initialize i2c communication with HTS221:
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  Wire.begin();
  // smeHumidity.begin();

  // initialize i2c communication with OPT3001
  OPT3001_Config illumConfig;
  OPT3001_ErrorCode illumErrorConfig;
  illum.begin(OPT3001_ADDRESS);
  illumConfig.RangeNumber = B1100;             // automatic full scale
  illumConfig.ConvertionTime = B1;             // convertion time = 800ms
  illumConfig.ModeOfConversionOperation = B11; // continous conversion
  illumConfig.Latch = B0;                      // hysteresis-style
  illumErrorConfig = illum.writeConfig(illumConfig);
  if (illumErrorConfig != NO_ERROR)
  {
    illumErrorConfig = illum.writeConfig(illumConfig); //retry
  }

  // accelerometer isn't used
  accel.setDataRate(LIS3DH_DATARATE_POWERDOWN);

  /* initialize other sensors */
}

void getSensors()
{
  // read temperature and humidity:
  // dataTemp = (float)smeHumidity.readTemperature();
  // dataHumid = (float)smeHumidity.readHumidity();
  ESP_LOGD(TAG, "Temperature: %2.1f℃", dataTemp);
  ESP_LOGD(TAG, "Humidity: %d%%", dataHumid);

  // read illuminance
  OPT3001 result = illum.readResult();
  if (result.error == NO_ERROR)
  {
    dataIllum = result.lux;
  }
  ESP_LOGD(TAG, "Illuminance: %dlx", (int)dataIllum);

  // read ADC registers:
  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR, 2);
  uint8_t adcVal1 = Wire.read();
  uint8_t adcVal2 = Wire.read();
  if (adcVal1 == 0xff && adcVal2 == 0xff)
  {
    // when ADC is not connected, read values are 0xFF:
    adcVal1 = adcVal2 = 0;
  }
  // voltage mV = adcVal * Vref(3.3V) / resolution(8bit) * Vdiv(2)
  double tempMillivolt = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  dataBatt = (float)(tempMillivolt / 1000);
  ESP_LOGD(TAG, "Battery Voltage: %1.2fV", dataBatt);

  /* read other sensors */
}

void sleepSensors()
{
  // Temp & Humid sensors
  // smeHumidity.deactivate();

  // Illuminance sensor
  // OPT3001_Config illumConfig;
  // illumConfig.ModeOfConversionOperation = B00; // Shutdown mode
  // illum.writeConfig(illumConfig);

  /* sleep or power-off sensors */
}

//*****************************************************************************
// Logging functions
//*****************************************************************************
void saveLogData()
{
  String logData = String(dataTemp, 1) + ',' +
                   String((int)dataHumid) + ',' +
                   String((int)dataIllum) + ',' +
                   String(dataBatt, 2) + '\n';
  if(!SPIFFS.begin(true)){
      ESP_LOGD(TAG, "SPIFFS Mount Failed");
      return;
  }

  // open the log file
  /* TODO: FILE_WRITE mode at the first time */
  /* TODO: Check the file size before writing */
  File file = SPIFFS.open(logpath, FILE_APPEND);
  // File file = SPIFFS.open(logpath, FILE_WRITE);
  if(!file || file.isDirectory()){
    ESP_LOGE(TAG, "- failed to open file for reading");
    return;
  }

  // write log data
  if(file.print(logData)){
      ESP_LOGD(TAG, "- file written: %s", logData.c_str());
  } else {
      ESP_LOGE(TAG, "- frite failed");
  }
}

void sendLogData()
{
  ESP_LOGD(TAG, "sendLogData()");
  inLogDataTransporting = true;
}



//*****************************************************************************
// BLE Server Callback
//*****************************************************************************
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    // when deviceConnected flag becomes true, then esp32 doesn't go to sleep.
    deviceConnected = true;
    ESP_LOGI(TAG, "BLE Server: Connected");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    ESP_LOGI(TAG, "BLE Server: Disconnected");

    // when disconnected then ESP32 goes to sleep.
    startSleep();
  }
};

//*****************************************************************************
// BLE Characteristic Callback
//*****************************************************************************
class MyCharCallbacks : public BLECharacteristicCallbacks
{
  void onRead(BLECharacteristic *pCharacteristic)
  {
    ESP_LOGD(TAG, "BLE Characteristic: onRead");
  }

  void onWrite(BLECharacteristic *pCharacteristic)
  {
    ESP_LOGD(TAG, "BLE Characteristic: onWrite");

    std::string value = pCharacteristic->getValue();
    ESP_LOGD(TAG, "New value: %s", value.c_str());

    decodeBLECommand(value);
  }
};

//*****************************************************************************
// BLE functions
//*****************************************************************************
void setupBLE()
{
  // This text will be sent by advertising.
  String advManufacturerData = String(dataTemp, 1) + String(',') +
                               String((int)dataHumid) + String(',') +
                               String((int)dataIllum) + String(',') +
                               String(dataBatt, 2);
  ESP_LOGD(TAG, "Advertising Data: %s (Length: %d)", advManufacturerData.c_str(), advManufacturerData.length());

  // Create the BLE Device
  BLEDevice::init(deviceName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create a BLE Characteristic
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE |
          BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setCallbacks(new MyCharCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  // Setup the advertising data
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setFlags(0b0001 | 0b0100); // LE Limited Discoverable Mode | BR_EDR_NOT_SUPPORTED
  oAdvertisementData.setName(deviceName);
  oAdvertisementData.setManufacturerData(advManufacturerData.c_str()); // String to std::__cxx11:string
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);

  pAdvertising->start();
  ESP_LOGI(TAG, "Advertising started");

  // continue advertising during this period.
  delay(TIME_TO_AWAKE * 1000);

  pAdvertising->stop();
  ESP_LOGI(TAG, "Advertising stopped");
}

void decodeBLECommand(std::string cmd)
{
  if (cmd.length() < 0)
    return;

  if (cmd == "get")
  {
    sendLogData();
  }
  else if (cmd == "set")
  {
    /* something to do for this command */
  }
  else
  {
    /* something to do for this command*/
  }
}


//*****************************************************************************
// Main functions
//*****************************************************************************
void setup()
{
  // Initialize serial port
  Serial.begin(115200);

  // Initialize ESP32 Power Management
  initPowerManager();

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  // Wakeup Sensors on AI01A Leaf
  setupSensors();
  getSensors();
  sleepSensors();

  // Write log data
  saveLogData();

  // Initialize BLE
  setupBLE();

  // Setup sleep reason
  enableSleep();

  // Go to sleep mode when BLE device is not connected;
  if (!deviceConnected)
  {
    startSleep();
  }
}

void loop()
{
  if (deviceConnected)
  {
    if (inLogDataTransporting) {
      String sendData = "";

      // open the log file
      File file = SPIFFS.open(logpath);
      if(!file || file.isDirectory()){
        ESP_LOGE(TAG, "- failed to open file for reading");
        return;
      }

      // Read and send log data
      while(file.available()){
        char c = file.read();

        if(c == '\n'){
          // Send data
          pCharacteristic->setValue(sendData.c_str());
          //pCharacteristic->indicate();
          pCharacteristic->notify();
          ESP_LOGD(TAG, "Send log data: %s", sendData.c_str());

          sendData = "";
        } else {
          // combine character
          sendData += c;
        }
      }

      // Finished reading logdata then
      inLogDataTransporting = false;
    }
  }
}