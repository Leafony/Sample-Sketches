//=====================================================================
//  Leafony Platform sample sketch
//     Application  : STM32 BLE Logger Beacon Example
//     Processor    : STM32L452RE (Nucleo-64/Nucleo L452RE)
//     Arduino IDE  : 1.8.12
//     STM32 Core   : Ver1.9.0
//
//     Leaf configuration
//       (1) AI01 4-Sensors
//       (2) AP03 STM32 MCU
//       (3) AZ01 USB
//       (4) AC02 BLE Sugar
//       (5) AV0X Battery Leaf
//
//     Description
//      センサデータをBeaconで送信しながらSTM32のFlashに書き込み、
//      Webbluetoothから接続要求があれば、Flashのデータをすべて送信するサンプルスケッチです。
//
//    (c) 2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.01 2021/04/01 First release
//=====================================================================
//  Required Libraries
//    https://github.com/ameltech/sme-hts221-library
//    https://github.com/closedcube/ClosedCube_OPT3001_Arduino
//    https://github.com/Leafony/TBGLib
//    https://github.com/tomozh/arduino_ST7032
//    https://github.com/stm32duino/STM32LowPower
//    https://github.com/stm32duino/STM32RTC
//=====================================================================
#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>

#include "STM32LowPower.h"
#include <STM32RTC.h>
#include "RTClib.h"

#include <Adafruit_LIS3DH.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>
#include "TBGLib.h"

//=====================================================================
// Sketch firmware version
//=====================================================================
const String FIRMWARE_VERSION = "2021.09.140";

//=====================================================================
// BLE Local device name
//=====================================================================
const String strDeviceNamePrefix = "Leaf_";
const String strDeviceNameUnique = "A";
String strDeviceName = strDeviceNamePrefix + strDeviceNameUnique;

//=====================================================================
// シリアルコンソールへのデバック出力
//      #define DEBUG = 出力あり
//　　//#define DEBUG = 出力なし（コメントアウトする）
//=====================================================================
#define DEBUG

//=====================================================================
// スリープ時間、送信時間の設定
//  DEFAULT_SLEEP_INTERVAL : スリープ時間 (秒)
//  DEFAULT_WAKE_INTERVAL　：Beacon送信時間 (秒)
//  DEFAULT_CLICK_WAKE_INTERVAL : ダブルタップをしたときの起動時間 (秒)
//=====================================================================
#define DEFAULT_SLEEP_INTERVAL 60
#define DEFAULT_WAKE_INTERVAL 1
#define DEFAULT_CLICK_WAKE_INTERVAL 20

//=====================================================================
// センサ測定間隔、データ保存間隔の設定
//  DEFAULT_SENS_FREQ : センサON頻度
//  DEFAULT_SAVE_RREQ ：データ保存頻度
//=====================================================================
#define DEFAULT_SENS_FREQ 1 // TODO: implement dynamic sens frequency
#define DEFAULT_SAVE_FREQ 1 // TODO: implement dynamic save frequency

//=====================================================================
// IO pins definition
//=====================================================================
// for STM32 29-pin leaf
#define BLE_WAKEUP PB12 // D7   PB12
#define BLE_RX PA0      // [A2] PA1
#define BLE_TX PA1      // [A1] PA0
#define INT_0 PC7       // INT0
#define INT_1 PB3       // INT1

// for STM32 58-pin leaf Bus-A
// #define BLE_WAKEUP PA8
// #define BLE_TX PA1
// #define BLE_RX PA0
// #define INT_0 PC7
// #define INT_1 PB3

// for STM32 58-pin leaf Bus-B
// #define BLE_WAKEUP PB11
// #define BLE_TX PC5
// #define BLE_RX PC4
// #define INT_0 PC7
// #define INT_1 PB3

//=====================================================================
// プログラム内で使用する定数定義
//
//=====================================================================
// I2C addresses
#define LIS2DH_ADDRESS        0x19 // Accelerometer (SD0/SA0 pin = VCC)
#define OPT3001_ADDRESS       0x45 // Ambient Light Sensor (ADDR pin = VCC)
#define LCD_I2C_EXPANDER_ADDR 0x1A // LCD I2C Expander
#define BATT_ADC_ADDR         0x50 // Battery ADC

// Adjust this number for the sensitivity of the 'click' force
// this strongly depend on the range! for 16G, try 5-10
// for 8G, try 10-20. for 4G try 20-40. for 2G try 40-80
#define CLICKTHRESHHOLD 80
#define SINGLETAP 1
#define DOUBLETAP 2

// BLE states
#define BLE_STATE_STANDBY          (0)
#define BLE_STATE_SCANNING         (1)
#define BLE_STATE_ADVERTISING      (2)
#define BLE_STATE_CONNECTING       (3)
#define BLE_STATE_CONNECTED_MASTER (4)
#define BLE_STATE_CONNECTED_SLAVE  (5)

// BLE Connecting Mode
#define MODE_IDLE               (0)
#define MODE_SEND_DATA          (1)
#define MODE_CLEAR_EEPROM       (2)

// BLE Advertising Interval
// Example: If the minimum advertisement interval is 40ms and the maximum advertisement interval is 100ms
// then the real advertisement interval will be mostly the middle value (70ms) plus a randomly added 20ms delay,
// which needs to be added according to the Bluetooth specification.
#define MIN_ADV_INTVAL 200 // 200 * 0.625ms = 125ms
#define MAX_ADV_INTVAL 400 // 400 * 0.625ms = 250ms

// Baudrate
#define SERIAL_BAUD 115200

//=====================================================================
// objects
//=====================================================================
// Sensors
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;

// BLE
HardwareSerial Serialble(BLE_TX, BLE_RX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0);

// RTC
STM32RTC& rtc = STM32RTC::getInstance();

//=====================================================================
// Variables
//=====================================================================
// LIS2DH : accelerometer
float dataX_g, dataY_g, dataZ_g;
float dataTilt = 0;

// HTS221 : Temperature/Humidity
float dataTemp = 0;
float dataHumid = 0;

// OPT3001 : Light
float dataLight = 0;

// Battery voltage
float dataBatt = 0;

// BLE
volatile bool bSystemBootBle = false;
volatile uint8_t ble_state = BLE_STATE_STANDBY;
bool bBleConnected = false;
uint8_t mode = MODE_IDLE;

// EEPROM ringbuffer
uint16_t rb_addr = 0;  // ringbuffer read address
const uint8_t RINGBUFF_OFFSET_ADDR = 20;
const uint8_t PACKET_LENGTH = 12;

uint16_t wake_intval = DEFAULT_WAKE_INTERVAL;   // Wake time
uint16_t click_wake_intval = DEFAULT_CLICK_WAKE_INTERVAL;  // Click interrupt wake time
uint16_t sleep_intval = DEFAULT_SLEEP_INTERVAL; // Sleep time
uint16_t sens_freq = DEFAULT_SENS_FREQ;         // Sensor ON frequency
uint16_t save_freq = DEFAULT_SAVE_FREQ;         // Data save frequency

// On-Clicked Interrupt
bool onClickedFlag = false;

//----------------------------------------------
// Accelerometer double-tap interrupt
//----------------------------------------------
void onClicked() {
  onClickedFlag = true;
}

//----------------------------------------------
// IO ports initialization
//----------------------------------------------
void setupPort() {
  pinMode(BLE_WAKEUP, OUTPUT);    // BLE Wakeup/Sleep
  digitalWrite(BLE_WAKEUP, HIGH); // BLE Wakeup
}

//-----------------------------------------------
// BLE initialization
//-----------------------------------------------
void setupBLE() {
#ifdef DEBUG
  Serial.println("setupBLE()");
#endif
  // set up internal status handlers
  ble112.onBusy = onBusy;
  ble112.onIdle = onIdle;
  ble112.onTimeout = onTimeout;

  // set up BGLib event handlers
  ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value;
  ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;
  ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed;
  ble112.ble_evt_system_boot = my_evt_system_boot;
  ble112.ble_evt_system_awake = my_evt_system_awake;
  ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;

  Serialble.begin(9600);

  uint8_t tm=0;
  while (!bSystemBootBle && tm <150){  // Wait for BLE Start-up
    ble112.checkActivity(100);
    tm++;
    delay(10);
  }

  // ble_rsp_system_get_bt_address handler is called.
  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(100));

  // set advertising parameters
  ble112.ble_cmd_le_gap_set_adv_parameters(MIN_ADV_INTVAL, MAX_ADV_INTVAL, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */
  while (ble112.checkActivity(100));
}


//-----------------------------------------------
// BLE Advertising data configuration
//-----------------------------------------------
void StartAdvData() {
  // char charTemp[7], charHumid[7], charBatt[7];
  uint16_t temp, humid, illum, battVolt;
  char userData[15];
  uint8_t dataLen;
  uint8_t stLen;
  uint8_t adv_data[25]; // advertising data (max 25bytes)
  uint8_t index = 0;

  //-------------------------
  // Sensors data
  //-------------------------
  temp     = (uint16_t)(dataTemp  * 256);
  humid    = (uint16_t)(dataHumid * 256);
  battVolt = (uint16_t)(dataBatt  * 256);
  illum    = (uint16_t)dataLight;

  //-------------------------
  // Advertising Packet
  //-------------------------
  // AD Structure 1 (Flags)
  adv_data[index++] = 0x02;                    // field length
  adv_data[index++] = BGLIB_GAP_AD_TYPE_FLAGS; // AD Type (Flags)
  adv_data[index++] = (1 << 1) | (1 << 2);     // LE General Discover Mode | BR/EDR Not Supported

  // AD Structure 2 (Complete Local Name)
  adv_data[index++] = strDeviceName.length() + 1;           // field length
  adv_data[index++] = BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE; // AD Type (Complete Local Name)
  for (uint8_t i = 0; i < strDeviceName.length(); i++) {
    adv_data[index++] = strDeviceName.charAt(i); // Local Name
  }

  // AD Structure 3 (Manufacturer Specific Data)
  adv_data[index++] = 9;    // field lengh
  adv_data[index++] = 0xff; // AD Type (Manufacturer Specific Data)
  adv_data[index++] = (illum >> 8) & 0xFF;    // Illuminance (Upper)
  adv_data[index++] = illum & 0xFF;           // Illuminance (Lower)
  adv_data[index++] = (temp >> 8) & 0xFF;     // Temperature (Upper)
  adv_data[index++] = temp & 0xFF;            // Temperature (Lower)
  adv_data[index++] = (humid >> 8) & 0xFF;    // Humidity (Upper)
  adv_data[index++] = humid & 0xFF;           // Humidity (Lower)
  adv_data[index++] = (battVolt >> 8) & 0xFF; // Battery Voltage (Upper)
  adv_data[index++] = battVolt & 0xFF;        // Battery Boltage (Lower)

  // register advertising packet
  stLen = index;
  ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, adv_data);
  while (ble112.checkActivity(1000));

  // index = 0  LE_GAP_SCANNABLE_NON_CONNECTABLE / LE_GAP_UNDIRECTED_CONNECTABLE
  ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);
  while (ble112.checkActivity(1000));
}


//-----------------------------------------------
// Sensors initialization
//-----------------------------------------------
void setupSensors() {
#ifdef DEBUG
  Serial.println("Initializing sensors...");
#endif

  // LIS2DH (accelerometer)
  accel.begin(LIS2DH_ADDRESS);
  accel.setRange(LIS3DH_RANGE_2_G);  // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_50_HZ);  // Data rate = 10Hz
  accel.setClick(DOUBLETAP, CLICKTHRESHHOLD);  // enable click interrupt

  // enable interrupt from accelerometer click event
  LowPower.attachInterruptWakeup(INT_1, onClicked, RISING, DEEP_SLEEP_MODE);

  // HTS221 (temperature /humidity)
  smeHumidity.begin();

  // OPT3001 (light)
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;
  light.begin(OPT3001_ADDRESS);
  newConfig.RangeNumber = B1100;             // automatic full scale
  newConfig.ConvertionTime = B0;             // convertion time = 100ms
  newConfig.ModeOfConversionOperation = B01; // single-shot conversion
  newConfig.Latch = B0;                      // hysteresis-style
  errorConfig = light.writeConfig(newConfig);

  delay(100); // wait until all sensors are ready

#ifdef DEBUG
  Serial.println("Sensors initialized.");
#endif
}


//--------------------------------------------------------------------
// Get sensors values
//--------------------------------------------------------------------
void getSensors() {
  // HTS221 (temperature & humidity)
  smeHumidity.begin();
  dataTemp = (float)smeHumidity.readTemperature();
  dataHumid = (float)smeHumidity.readHumidity();

  //--------------------
  // 2点補正用データ
  //--------------------
  // 温度補正用データ0
  float TL0 = 25.0;     // 4-Sensors温度測定値
  float TM0 = 25.0;     // 温度計等測定値
  // 温度補正用データ1
  float TL1 = 40.0;     // 4-Sensors温度測定値
  float TM1 = 40.0;     // 温度計等測定値
  // 湿度補正用データ0
  float HL0 = 60.0;     // 4-Sensors湿度測定値
  float HM0 = 60.0;     // 湿度計等測定値
  // 湿度補正用データ1
  float HL1 = 80.0;     // 4-Sensors湿度測定値
  float HM1 = 80.0;     // 湿度計等測定値
  //-------------------------
  // 温度と湿度の2点補正
  //-------------------------
  dataTemp=TM0+(TM1-TM0)*(dataTemp-TL0)/(TL1-TL0);      // 温度補正
  dataHumid=HM0+(HM1-HM0)*(dataHumid-HL0)/(HL1-HL0);    // 湿度補正

  // if (dataHumid >= 100)
  // {
  //  dataHumid=100;
  // }

  // OPT3001 (illuminance)
  delay(100);
  OPT3001 result = light.readResult();
  dataLight = result.lux;

  // ADC081C027（ADC) battery voltage
  uint8_t adcVal1 = 0;
  uint8_t adcVal2 = 0;

  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR, 2);
  adcVal1 = Wire.read();
  adcVal2 = Wire.read();

  //測定値がFFならバッテリリーフはつながっていない
  if (adcVal1 == 0xff && adcVal2 == 0xff) {
    adcVal1 = adcVal2 = 0;
  }

  //電圧計算　ADC　* （(リファレンス電圧(3.3V)/ ADCの分解能(256)) * 分圧比（2倍））
  double temp_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  dataBatt = (float)(temp_mv / 1000);

#ifdef DEBUG
  Serial.println("");
  Serial.println("--- sensors data ---");
  Serial.println("  Tmp[degC]     = " + String(dataTemp));
  Serial.println("  Hum[%]        = " + String(dataHumid));
  Serial.println("  Lum[lx]       = " + String(dataLight));
  Serial.println("  Bat[V]        = " + String(dataBatt));
  Serial.println("");
#endif
}

//-----------------------------------------
// sleep sensors
// センサーリーフをスリープさせる
//-----------------------------------------
void sleepSensors() {
  // HTS221 sleep
  smeHumidity.deactivate();

  // OPT3001 sleep
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;
  newConfig.ModeOfConversionOperation = B00;
  errorConfig = light.writeConfig(newConfig);
}

//-----------------------------------------
// wakeup sensors
// センサーリーフをスリープから復帰させる
//-----------------------------------------
void wakeupSensors() {
#ifdef DEBUG
  Serial.println(F("Wakeup Sensors."));
#endif

  // HTS221 wakeup
  smeHumidity.activate();

  // OPT3001 wakeup
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;
  newConfig.RangeNumber = B1100;  // automatic full scale
  newConfig.ModeOfConversionOperation = B01; //single-shot conversion
  errorConfig = light.writeConfig(newConfig);
  delay(300);
}

//---------------------------------------
// sleep BLE
// BLE リーフをスリープさせる
//---------------------------------------
void sleepBLE() {
#ifdef DEBUG
  Serial.println("Sleep BLE");
#endif

  ble112.ble_cmd_le_gap_stop_advertising(0);
  while (ble112.checkActivity());

  ble112.ble_cmd_system_halt(1);
  while (ble112.checkActivity());
  digitalWrite(BLE_WAKEUP, LOW);
}

//---------------------------------------
// wakeup BLE
// BLEリーフをスリープから復帰させる
//---------------------------------------
void wakeupBLE() {
#ifdef DEBUG
  Serial.println("Wakeup BLE");
#endif

  digitalWrite(BLE_WAKEUP, HIGH);
  delay(10);

  ble112.ble_cmd_system_halt(0);
  while (ble112.checkActivity());

  ble112.ble_cmd_le_gap_set_adv_parameters(MIN_ADV_INTVAL, MAX_ADV_INTVAL, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */
  while (ble112.checkActivity());
}

//---------------------------------------
// EEPROM
//
// Address Map
//   Addr, Description
//   0, EEPROM Configuration Check (if 0xAA: EEPROM is configured)
//   1, EEPROM Configuration Check (if 0xAA: EEPROM is configured)
//   2, Wake Interval (Upper)
//   3, Wake Interval (Lower)
//   4, Sleep Interval (Upper)
//   5, Sleep Interval (Lower)
//   6, Sens Frequency (Upper)
//   7, Sens Frequency (Lower)
//   8, Save Frequency (Upper)
//   9, Save Frequency (Lower)
//   10-11, Reserved
//   12, Device Name [0]
//   13, Device Name [1]
//   14, Device Name [2]
//   15, Device Name [3]
//   16, Device Name [4]
//   17-19, Reserved
//---------------------------------------
void setupEEPROM() {
  bool eeprom_configured = false;

  // EEPROM Configuration Check
  if (EEPROM.read(0) == 0xAA && EEPROM.read(1) == 0xAA) {
    #ifdef DEBUG
    Serial.println("EEPROM is already configured!");
    #endif
    eeprom_configured = true;
  } else {
    #ifdef DEBUG
    Serial.println("EEPROM is not configured. All variables are set to default.");
    #endif
  }

  if (eeprom_configured) {
    // Load saved registers
    // wake_intval  = (EEPROM.read(2) << 8) + EEPROM.read(3); // Reserved
    sleep_intval = (EEPROM.read(4) << 8) + EEPROM.read(5);
    // sens_freq    = (EEPROM.read(6) << 8) + EEPROM.read(7); // Reserved
    // save_freq    = (EEPROM.read(8) << 8) + EEPROM.read(9); // Reserved

    // Load device name
    strDeviceName = strDeviceNamePrefix;
    for (uint8_t i=0; i<5; i++){
      char c = EEPROM.read(12+i);
      // c is NOT alphabet or numeric
      if (!((c>='a' && c<='z') || (c>='A' && c<='Z') || (c>='0' && c<='9'))) {
        if (i==0) {  // Device Name is not set.
          strDeviceName += strDeviceNameUnique; // Set default unique name.
        }
        break;
      }
      // c is alphabet or numeric
      else {
        strDeviceName += String(c);
      }
    }
#ifdef DEBUG
    Serial.print("BLE Device Name is ");
    Serial.println(strDeviceName);
#endif

  } else {
    // Set default value
    wake_intval = DEFAULT_WAKE_INTERVAL;
    sleep_intval = DEFAULT_SLEEP_INTERVAL;
    sens_freq = DEFAULT_SENS_FREQ;
    save_freq = DEFAULT_SAVE_FREQ;

    // Save default value
    EEPROM.write(2, (wake_intval >> 8) & 0xFF);
    EEPROM.write(3, (wake_intval & 0xFF));
    EEPROM.write(4, (sleep_intval >> 8) & 0xFF);
    EEPROM.write(5, (sleep_intval & 0xFF));
    EEPROM.write(6, (sens_freq >> 8) & 0xFF);
    EEPROM.write(7, (sens_freq & 0xFF));
    EEPROM.write(8, (save_freq >> 8) & 0xFF);
    EEPROM.write(9, (save_freq & 0xFF));
    EEPROM.write(0, 0xAA); // EEPROM configured
    EEPROM.write(1, 0xAA); // EEPROM configured
  }

  // EEPROM Test
  for (uint16_t rb_work; rb_work < EEPROM.length(); rb_work++) {
    EEPROM.read(rb_work);
  }

  // when address is invalid;
  if (rb_addr >= EEPROM.length() || (rb_addr - RINGBUFF_OFFSET_ADDR) % PACKET_LENGTH != 0) {
    rb_addr = RINGBUFF_OFFSET_ADDR;
  }

#ifdef DEBUG
  Serial.print("EEPROM length: ");
  Serial.println(EEPROM.length());
  Serial.print("Ring buffer read address: ");
  Serial.println(rb_addr);
  Serial.print("WAKE_INTERVAL: ");
  Serial.print(wake_intval);
  Serial.println(" seconds");
  Serial.print("SLEEP_INTERVAL: ");
  Serial.print(sleep_intval);
  Serial.println(" seconds");
  // Serial.print("SENS_FREQ = ");
  // Serial.println(sens_freq);
  // Serial.print("SAVE_FREQ = ");
  // Serial.println(save_freq);
#endif
}

/**
 * 
 */
void writeEEPROM() {
  uint16_t temp, humid, illum, battVolt;
  uint32_t u_time;

  if (!rtc.isTimeSet()) {
    // RTC is not set, skip to save data.
    return;
  }

  // Get current time from RTC
  u_time = getTimestamp();

  // Reset the ring buffer address when the size is not enough;
  if (rb_addr + PACKET_LENGTH >= EEPROM.length()) {
    rb_addr = RINGBUFF_OFFSET_ADDR;
  }

#ifdef DEBUG
  Serial.println("");
  Serial.print("writeEEPROM(): ADDR=");
  Serial.print(rb_addr);
  Serial.print(", timestamp=");
  Serial.println(u_time);
#endif

  // Write the sensors data to EEPROM.
  temp = (uint16_t)(dataTemp * 256.0);
  humid = (uint16_t)(dataHumid * 256.0);
  illum = (uint16_t)dataLight;
  battVolt = (uint16_t)(dataBatt * 256.0);

  uint8_t data[12];
  data[0]  = (temp >> 8) & 0xFF;
  data[1]  = temp & 0xFF;
  data[2]  = (humid >> 8) & 0xFF;
  data[3]  = humid & 0xFF;
  data[4]  = (illum >> 8) & 0xFF;
  data[5]  = illum & 0xFF;
  data[6]  = (battVolt >> 8) & 0xFF;
  data[7]  = battVolt & 0xFF;
  data[8]  = (u_time >> 0) & 0xFF;
  data[9]  = (u_time >> 8) & 0xFF;
  data[10] = (u_time >> 16) & 0xFF;
  data[11] = (u_time >> 24) & 0xFF;

  for (uint8_t i = 0; i < 12; i++) {
    EEPROM.write(rb_addr + i, data[i]);
    delay(10);  // EEPROM書き込み後しばらく待つ
  }

  // write next ring buffer address to RTC backup register.
  rb_addr += PACKET_LENGTH;

#ifdef DEBUG
  Serial.print("{ temp = ");
  Serial.print(temp);
  Serial.print(", humid = ");
  Serial.print(humid);
  Serial.print(", illum = ");
  Serial.print(illum);
  Serial.print(", batt = ");
  Serial.print(battVolt);
  Serial.println(" }");
  Serial.print("Next ringbuffer address: ");
  Serial.println(rb_addr);
  Serial.println("");
#endif
}

/**
 * decode RTC to timestamp.
 */
uint32_t getTimestamp() {
  if (!rtc.isTimeSet()) {
    return 0;
  }

  uint8_t year = rtc.getYear();
  uint8_t month = rtc.getMonth();
  uint8_t day = rtc.getDay();
  uint8_t hours = rtc.getHours();
  uint8_t minutes = rtc.getMinutes();
  uint8_t seconds = rtc.getSeconds();

#ifdef DEBUG
  Serial.print("RTC Timestamp: ");
  Serial.print(year + 2000);
  Serial.print("/");
  Serial.print(month);
  Serial.print("/");
  Serial.print(day);
  Serial.print(" ");
  Serial.print(hours);
  Serial.print(":");
  Serial.print(minutes);
  Serial.print(":");
  Serial.print(seconds);
  Serial.println(" (GMT+0)");
#endif
  DateTime date (year, month, day, hours, minutes, seconds);
  return date.unixtime();
}

/**
 * 
 */
void sleepAllDevices() {
  sleepBLE();
  sleepSensors();

#ifdef DEBUG
  Serial.print(">>> STM32 deepsleep (restart after ");
  Serial.print(sleep_intval);
  Serial.println(" seconds) >>>");
  Serial.flush();
#endif

  LowPower.deepSleep(sleep_intval * 1000);
}

/**
 * 
 */
void setup() {
  Serial.begin(SERIAL_BAUD);
  LowPower.begin(); // Configure low power

  Wire.begin(); // I2C 100KHz
  rtc.begin(); // initialize RTC 24H format

#ifdef DEBUG
  Serial.print("FIRMWARE VERSION: ");
	Serial.println(FIRMWARE_VERSION);
  Serial.print("CPU Clock Frequency = ");
	Serial.println(HAL_RCC_GetHCLKFreq());
  Serial.println("=========================================");
  Serial.println("Setup start.");
#endif

  setupPort();
  setupEEPROM();
  setupSensors();
  setupBLE();

#ifdef DEBUG
  Serial.println("Setup finished.");
  Serial.println("=========================================");
#endif
}

/**
 * 
 */
void loop() {
  if (!bBleConnected) { // when BLE is not connected.
#ifdef DEBUG
    Serial.println("<<< Wake up <<<");
#endif

    wakeupSensors();
    wakeupBLE();

    getSensors();
    sleepSensors();

    writeEEPROM();

    StartAdvData();

    if (onClickedFlag) {  // onClicked Interrupt
      onClickedFlag = false;
#ifdef DEBUG
      Serial.println("STM32 on Clicked!");
      Serial.print("Start advertising ");
      Serial.print(click_wake_intval);
      Serial.println(" seconds.");
      Serial.flush();
#endif

      // Continue Advertising; (check BLE status every 0.1 secound.)
      for (int i = 0; i < click_wake_intval * 10; i++) {
        delay(100);
        // check ble status; if connection requested, my_evt_le_connection_opend handler is called.
        ble112.checkActivity();
      }

    } else {  // Normal operation
#ifdef DEBUG
      Serial.print("Start advertising (");
      Serial.print(wake_intval);
      Serial.println("s)");
#endif

      // Continue Advertising;
      for (int i = 0; i <= wake_intval * 10; i++) {
        delay(100);
        // don't check activity, if ble is not connectable during normal advertising operation.
        // ble112.checkActivity();
      }

      // bBleConnected turns true at this time, when the connection is requested;
    }

    // when the connection is not requested, shutdown all devices during SLEEP_INTERVAL seconds;
    if (!bBleConnected) {
      // accel.setClick(DOUBLETAP, CLICKTHRESHHOLD); // Enable Interrupt
      accel.getClick();  // Enable Interrupt
      sleepAllDevices();
    }
  } else { // when ble is connected, this scope will run continuously.
    if (mode == MODE_SEND_DATA) {
#ifdef DEBUG
      Serial.println("Start to send data.");
#endif

      for (int i = RINGBUFF_OFFSET_ADDR; i < EEPROM.length(); i += PACKET_LENGTH) {
        char sendData[PACKET_LENGTH];

        for (int j=0; j<PACKET_LENGTH; j++) {
          sendData[j] = EEPROM.read(i + j);
        }
        ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, PACKET_LENGTH, (const uint8_t *)sendData);
        while (ble112.checkActivity(1000));
      }

#ifdef DEBUG
      Serial.println("Finish to send data.");
#endif

      // after all the data trasnported,
      ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, 6, (const uint8_t *)"finish");
      while (ble112.checkActivity(1000));
      mode = MODE_IDLE;
    }
    else if (mode == MODE_CLEAR_EEPROM) {
      for(int i = RINGBUFF_OFFSET_ADDR; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
#ifdef DEBUG
        if (i % 10 == 0) {
          Serial.print(i);
          Serial.print("/");
          Serial.println(EEPROM.length() - RINGBUFF_OFFSET_ADDR);

          char sendData[PACKET_LENGTH];
          uint8_t len = sprintf(sendData, "%05d", i);
          ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, len, (const uint8_t *)sendData);
          while (ble112.checkActivity(1000));
        }
#endif
      }

      ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, 6, (const uint8_t *)"finish");
      while (ble112.checkActivity(1000));
      mode = MODE_IDLE;
    } else { // MODE_IDLE
      ble112.checkActivity(100);
    }
  }
}


// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================

// called when the module begins sending a command
void onBusy()
{
}

// called when the module receives a complete response or "system_boot" event
void onIdle()
{
}

// called when the parser does not read the expected response in the specified time limit
void onTimeout()
{
  ble_state = BLE_STATE_STANDBY;
}

// called immediately before beginning UART TX of a command
void onBeforeTXCommand()
{
}

// called immediately after finishing UART TX
void onTXCommandComplete()
{
  // allow module to return to sleep (assuming here that digital pin 5 is connected to the BLE wake-up pin)
}

// called when the attribute value changed
void my_evt_gatt_server_attribute_value(const struct ble_msg_gatt_server_attribute_value_evt_t *msg)
{
  uint16 attribute = (uint16)msg->attribute;
  uint16 offset = 0;
  uint8 value_len = msg->value.len;

  uint8 value_data[20];
  String rcv_data;
  rcv_data = "";

  for (uint8_t i = 0; i < value_len; i++)
  {
    rcv_data += (char)(msg->value.data[i]);
  }

#ifdef DEBUG
  Serial.print("###\tgatt_server_attribute_value: { ");
  Serial.print("connection: ");
  Serial.print(msg->connection, HEX);
  Serial.print(", attribute: ");
  Serial.print((uint16_t)msg->attribute, HEX);
  Serial.print(", att_opcode: ");
  Serial.print(msg->att_opcode, HEX);
  Serial.print(", offset: ");
  Serial.print((uint16_t)msg->offset, HEX);
  Serial.print(", value_len: ");
  Serial.print(msg->value.len, HEX);
  Serial.print(", value_data: ");
  Serial.print(rcv_data);
  Serial.println(" }");
#endif

  // Received BLE Commands
  if (rcv_data.startsWith("getData"))
  {
    // Start to send EEPROM data
    mode = MODE_SEND_DATA;
  }
  else if (rcv_data.startsWith("getRomSize"))
  {
    char sendData[8];
    uint8_t len = sprintf(sendData, "%06d", EEPROM.length());
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, len, (const uint8_t *)sendData);
    while (ble112.checkActivity(1000));
  }
  else if (rcv_data.startsWith("getSleep"))
  {
    // send SLEEP_INTERVAL
    char sendData[8];
    uint8_t len = sprintf(sendData, "%05d", (int)sleep_intval);
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, len, (const uint8_t *)sendData);
    while (ble112.checkActivity(1000));
  }
  else if (rcv_data.startsWith("getWake"))
  {
    // send WAKE_INTERVAL
    char sendData[8];
    uint8_t len = sprintf(sendData, "%05d", (int)wake_intval);
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, len, (const uint8_t *)sendData);
    while (ble112.checkActivity(1000));
  }
  else if (rcv_data.startsWith("getSensFreq"))
  {
    // send SENS_FREQ
    char sendData[8];
    uint8_t len = sprintf(sendData, "%05d", (int)sens_freq);
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, len, (const uint8_t *)sendData);
    while (ble112.checkActivity(1000));
  }
  else if (rcv_data.startsWith("getSaveFreq"))
  {
    // send SAVE_FREQ
    char sendData[8];
    uint8_t len = sprintf(sendData, "%05d", (int)save_freq);
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, len, (const uint8_t *)sendData);
    while (ble112.checkActivity(1000));
  }
  else if (rcv_data.startsWith("setSleep"))
  {
    // rcv_data = "setSleep <SLEEP_INTERVAL>"
    uint16_t num = rcv_data.substring(9).toInt();
    EEPROM.write(4, (num >> 8) & 0xFF); // sleep_intval
    EEPROM.write(5, (num & 0xFF));      // sleep_intval
    sleep_intval = num;
    #ifdef DEBUG
    Serial.print("Sleep interval is changed. (");
    Serial.print(num);
    Serial.println("s)");
    #endif
  }
  else if (rcv_data.startsWith("setWake"))
  {
    // rcv_data = "setWake <WAKE_INTERVAL>"
    uint16_t num = rcv_data.substring(8).toInt();
    EEPROM.write(2, (num >> 8) & 0xFF);  // wake_intval
    EEPROM.write(3, (num & 0xFF));       // wake_intval
    wake_intval = num;
    #ifdef DEBUG
    Serial.print("Wake interval is changed. (");
    Serial.print(num);
    Serial.println("s)");
    #endif
  }
  else if (rcv_data.startsWith("setSensFreq"))
  {
    // rcv_data = "setSensFreq <SENS_FREQUENCY>"
    uint16_t num = rcv_data.substring(12).toInt();
    EEPROM.write(6, (num >> 8) & 0xFF);  // sens_freq
    EEPROM.write(7, (num & 0xFF));       // sens_freq
    #ifdef DEBUG
    Serial.print("Sens frequency is changed. (");
    Serial.print(num);
    Serial.println("s)");
    #endif
  }
  else if (rcv_data.startsWith("setSaveFreq"))
  {
    // rcv_data = "setSaveFreq <SAVE_FREQUENCY>"
    uint16_t num = rcv_data.substring(12).toInt();
    EEPROM.write(8, (num >> 8) & 0xFF);  // save_freq
    EEPROM.write(9, (num & 0xFF));  // save_freq
    #ifdef DEBUG
    Serial.print("Save frequency is changed. (");
    Serial.print(num);
    Serial.println("s)");
    #endif
  }
  else if (rcv_data.startsWith("setDevName"))
  {
    String rcvUniqueName = rcv_data.substring(11);

    // Set device name
    strDeviceName = strDeviceNamePrefix + rcvUniqueName;

    // Save device name
    for(uint8_t i=0; i<5; i++){
      if (i<rcvUniqueName.length()) {
        EEPROM.write(12+i, (uint8_t)rcvUniqueName.charAt(i));
      }
      else {
        EEPROM.write(12+i, 0);
      }
    }

    #ifdef DEBUG
    Serial.print("Device name is changed. (");
    Serial.print(strDeviceName);
    Serial.println(")");
    #endif
  }
  else if (rcv_data.startsWith("getDevName"))
  {
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, strDeviceName.length(), (const uint8_t *)strDeviceName.c_str());
    while (ble112.checkActivity(1000));
  }
  else if (rcv_data.startsWith("version"))
  {
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, FIRMWARE_VERSION.length(), (const uint8_t *)FIRMWARE_VERSION.c_str());
    while (ble112.checkActivity(1000));
  }
  else if (rcv_data.startsWith("setTime"))
  {
    // setTime <timestamp uint32>
    // read timestamp
    uint32_t unixtime = rcv_data.substring(8).toInt();
    DateTime timestamp (unixtime);
    rtc.setYear(timestamp.year() - 2000);
    rtc.setMonth(timestamp.month());
    rtc.setDay(timestamp.day());
    rtc.setHours(timestamp.hour());
    rtc.setMinutes(timestamp.minute());
    rtc.setSeconds(timestamp.second());

    EEPROM.write(2, (wake_intval >> 8) & 0xFF);
    EEPROM.write(3, (wake_intval & 0xFF));
    EEPROM.write(4, (sleep_intval >> 8) & 0xFF);
    EEPROM.write(5, (sleep_intval & 0xFF));
    EEPROM.write(6, (sens_freq >> 8) & 0xFF);
    EEPROM.write(7, (sens_freq & 0xFF));
    EEPROM.write(8, (save_freq >> 8) & 0xFF);
    EEPROM.write(9, (save_freq & 0xFF));

#ifdef DEBUG
    Serial.print("STM32 received timestamp: ");
    Serial.print(timestamp.year());
    Serial.print("/");
    Serial.print(timestamp.month());
    Serial.print("/");
    Serial.print(timestamp.day());
    Serial.print(" ");
    Serial.print(timestamp.hour());
    Serial.print(":");
    Serial.print(timestamp.minute());
    Serial.print(":");
    Serial.print(timestamp.second());
    Serial.print(" (Unixtime: ");
    Serial.print(unixtime);
    Serial.println(")");
#endif
  }
  else if (rcv_data.startsWith("clearEEPROM")){
    mode = MODE_CLEAR_EEPROM;
    // // Clear All
    // for(int i = RINGBUFF_OFFSET_ADDR; i < EEPROM.length(); i++) {
    //   EEPROM.write(i, 0);
    // }
  }
}

// called when the connection is opened
void my_evt_le_connection_opend(const ble_msg_le_connection_opend_evt_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tconnection_opend: { ");
  Serial.print("address: ");
  // this is a "bd_addr" data type, which is a 6-byte uint8_t array
  for (uint8_t i = 0; i < 6; i++)
  {
    if (msg->address.addr[i] < 16)
      Serial.write('0');
    Serial.print(msg->address.addr[i], HEX);
  }
  Serial.println(" }");
#endif

  bBleConnected = true;
  ble_state = BLE_STATE_CONNECTED_SLAVE;
}

// called when connection is closed
void my_evt_le_connection_closed(const struct ble_msg_le_connection_closed_evt_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tconnection_closed: { ");
  Serial.print("reason: ");
  Serial.print((uint16_t)msg->reason, HEX);
  Serial.print(", connection: ");
  Serial.print(msg->connection, HEX);
  Serial.println(" }");
#endif

  bBleConnected = false;
  // set state to ADVERTISING
  ble_state = BLE_STATE_STANDBY;
}

// called when the system booted
void my_evt_system_boot(const ble_msg_system_boot_evt_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tsystem_boot: { ");
  Serial.print("major: ");
  Serial.print(msg->major, HEX);
  Serial.print(", minor: ");
  Serial.print(msg->minor, HEX);
  Serial.print(", patch: ");
  Serial.print(msg->patch, HEX);
  Serial.print(", build: ");
  Serial.print(msg->build, HEX);
  Serial.print(", bootloader_version: ");
  Serial.print(msg->bootloader, HEX);
  Serial.print(", hw: ");
  Serial.print(msg->hw, HEX);
  Serial.println(" }");
#endif

  bSystemBootBle = true;
  ble_state = BLE_STATE_ADVERTISING;
}

void my_evt_system_awake(void)
{
#ifdef DEBUG
  Serial.print("###\tsystem_awake: { ");
  Serial.println(" }");
#endif
}

//
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tsystem_get_bt_address: { ");
  Serial.print("address: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.print(msg->address.addr[i], HEX);
  }
  Serial.println(" }");
#endif
  unsigned short addr = 0;
  char cAddr[30];
  addr = msg->address.addr[0] + (msg->address.addr[1] * 0x100);
  sprintf(cAddr, "#%05d", addr);
  Serial.print("Device name is ");
  Serial.print(strDeviceName);
  Serial.print("_");
  Serial.println(cAddr);
}
