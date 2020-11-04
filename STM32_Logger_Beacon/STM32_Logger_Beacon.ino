//=====================================================================
//  Leafony Platform sample sketch
//     Application  : STM32 Simple BLE Beacon Example
//     Processor    : STM32L452RE (Nucleo-64/Nucleo L452RE)
//     Arduino IDE  : 1.8.12
//     STM32 Core   : Ver1.9.0
//
//     Leaf configuration
//       (1) AC02 BLE Sugar     :Bus-A
//       (2) AI01 4-Sensors     :Bus-A
//       (3) AP03 STM32 MCU
//       (4) AZ01 USB           :Bus-A
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/10/08 First release
//=====================================================================
//  Required Libraries
//    https://github.com/adafruit/Adafruit_LIS3DH
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

#include <Adafruit_LIS3DH.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>
#include "TBGLib.h"

//=====================================================================
// BLE Local device name
// 長さは必ず6文字
//=====================================================================
String strDeviceName = "Leaf_Z";

//=====================================================================
// シリアルコンソールへのデバック出力
//      #define DEBUG = 出力あり
//　　//#define DEBUG = 出力なし（コメントアウトする）
//=====================================================================
#define DEBUG

//=====================================================================
// スリープ時間、送信時間の設定
//  SLEEP_INTERVAL : スリープ時間 (秒)
//  WAKE_INTERVAL　：パケット送信時間 (秒)
//=====================================================================
#define SLEEP_INTERVAL (5)
#define WAKE_INTERVAL  (10)

//=====================================================================
// IOピンの名前定義
//=====================================================================
// Bus-Aに接続する場合
// #define BLE_WAKEUP PA8
// #define BLE_TX PA1
// #define BLE_RX PA0

// Bus-Bに接続する場合
#define BLE_WAKEUP PB11
#define BLE_TX PC5
#define BLE_RX PC4

//=====================================================================
// プログラム内で使用する定数定義
//
//=====================================================================
// I2C addresses
#define LIS2DH_ADDRESS 0x19        // Accelerometer (SD0/SA0 pin = VCC)
#define OPT3001_ADDRESS 0x45       // Ambient Light Sensor (ADDR pin = VCC)
#define LCD_I2C_EXPANDER_ADDR 0x1A // LCD I2C Expander
#define BATT_ADC_ADDR 0x50         // Battery ADC

// BLE states
#define BLE_STATE_STANDBY          (0)
#define BLE_STATE_SCANNING         (1)
#define BLE_STATE_ADVERTISING      (2)
#define BLE_STATE_CONNECTING       (3)
#define BLE_STATE_CONNECTED_MASTER (4)
#define BLE_STATE_CONNECTED_SLAVE  (5)

//=====================================================================
// objects
//=====================================================================
// Sensors
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;

// BLE
HardwareSerial Serialble(BLE_TX, BLE_RX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0);

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
bool bBleSendData = false;

// EEPROM Ring Buffer
uint16_t rb_addr = 0;  // EEPROMリングバッファのTAILアドレス

//=====================================================================
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//=====================================================================
void setupPort()
{
  pinMode(BLE_WAKEUP, OUTPUT);    // BLE Wakeup/Sleep
  digitalWrite(BLE_WAKEUP, HIGH); // BLE Wakeup
}

//-----------------------------------------------
// sensor
//-----------------------------------------------
void setupSensor()
{
  // LIS2DH (accelerometer)
  accel.begin(LIS2DH_ADDRESS);
  accel.setClick(0, 0);                    // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);        // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_1_HZ); // Data rate = 1Hz

  // HTS221 (temperature /humidity)
  smeHumidity.begin();

  // OPT3001 (light)
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;
  light.begin(OPT3001_ADDRESS);

  newConfig.RangeNumber = B1100;             // automatic full scale
  newConfig.ConvertionTime = B1;             // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11; // continous conversion
  newConfig.Latch = B0;                      // hysteresis-style

  errorConfig = light.writeConfig(newConfig);

  if (errorConfig != NO_ERROR)
  {
    errorConfig = light.writeConfig(newConfig); //retry
  }
}

//-----------------------------------------------
// BLE
//-----------------------------------------------
void setupBLE()
{
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

  uint8_t tm = 0;
  Serialble.begin(9600);
  while (!Serialble && tm < 150)
  { // Serial起動待ち タイムアウト1.5s
    tm++;
    delay(10);
  }

  while (!bSystemBootBle)
  { // BLE起動待ち
    ble112.checkActivity(100);
  }

  // BLEの
  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));

  /* interval_min : 250ms( = 400 x 0.625ms ) */
  /* interval_max : 500ms( = 800 x 0.625ms ) */
  ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */
  while (ble112.checkActivity(1000));
}

//-----------------------------------------------
// アドバタイズするデータの設定
//-----------------------------------------------
void StartAdvData()
{
  uint8 stLen;
  float value;
  short int temp, humid, light, battVolt;
  char code[4];
  char sendData[15];

  /* setting */
  /* [set Advertising Data]  25byte MAX*/
  uint8 adv_data[] = {
      (2),                                  //0:  field length
      BGLIB_GAP_AD_TYPE_FLAGS,              //1:  field type (0x01)
      (6),                                  //2:  data
      (1),                                  //3:  field length (1は仮の初期値) BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE(LOCAL DEVICE NAMEのデータ長
      BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE, //4:  field type (0x09)
      (0),                                  //5:  L  1
      (0),                                  //6:  e  2
      (0),                                  //7:  a  3
      (0),                                  //8:  f  4
      (0),                                  //9:  _  5
      (0),                                  //10: A  6
      (0),                                  //11: field length
      (0xff),                               //12: field type (0xff)
      (0),                                  //13: Temp (Upper Byte)
      (0),                                  //14: Temp (Lower Byte)
      (0),                                  //15: Humid (Upper Byte)
      (0),                                  //16: Humid (Lower Byte)
      (0),                                  //17: Light (Upper Byte)
      (0),                                  //18: Light (Lower Byte)
      (0),                                  //19: Battery (Upper Byte)
      (0),                                  //20: Battery (Lower Byte)
      (0),                                  //21: reserved
      (0),                                  //22: reserved
      (0),                                  //23: reserved
      (0),                                  //24: reserved
  };

  // Sensors data
  temp = (short int)(dataTemp * 256);
  humid = (short int)(dataHumid * 256);
  light = (short int)dataLight;
  battVolt = (short int)(dataBatt * 256);

  size_t lenStr2 = strDeviceName.length();
  // BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETEのfield lengthを設定
  adv_data[3] = (lenStr2 + 1); // field length
  //アドバタイズデータにローカルデバイス名を入れる
  uint8_t u8Index;
  for (u8Index = 0; u8Index < lenStr2; u8Index++)
  {
    adv_data[5 + u8Index] = strDeviceName.charAt(u8Index);
  }
  adv_data[5 + u8Index] = 12;
  adv_data[5 + u8Index + 1] = 0xFF; // field type
  adv_data[5 + u8Index + 2] = (temp >> 8) & 0xFF;
  adv_data[5 + u8Index + 3] = temp & 0xFF;
  adv_data[5 + u8Index + 4] = (humid >> 8) & 0xFF;
  adv_data[5 + u8Index + 5] = humid & 0xFF;
  adv_data[5 + u8Index + 6] = (light >> 8) & 0xFF;
  adv_data[5 + u8Index + 7] = light & 0xFF;
  adv_data[5 + u8Index + 8] = (battVolt >> 8) & 0xFF;
  adv_data[5 + u8Index + 9] = battVolt & 0xFF;
  adv_data[5 + u8Index + 10] = 0;
  adv_data[5 + u8Index + 11] = 0;
  adv_data[5 + u8Index + 12] = 0;

  //アドバタイズデータを登録
  stLen = (5 + lenStr2 + 13);
  ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, adv_data); //SCAN_RSP_ADVERTISING_PACKETS
  while (ble112.checkActivity(1000));

  // index = 0  LE_GAP_SCANNABLE_NON_CONNECTABLE / LE_GAP_UNDIRECTED_CONNECTABLE
  ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);
  while (ble112.checkActivity(1000));
}

//--------------------------------------------------------------------
// センサの値を取得
//--------------------------------------------------------------------
void getSensor()
{
  double temp_mv;

  // LIS2DH 3軸センサーのデータ取得
  accel.read();
  dataX_g = accel.x_g;
  dataY_g = accel.y_g;
  dataZ_g = accel.z_g;

  if (dataZ_g >= 1.0)
  {
    dataZ_g = 1.00;
  }
  else if (dataZ_g <= -1.0)
  {
    dataZ_g = -1.00;
  }

  dataTilt = acos(dataZ_g) / PI * 180;

  // HTS221 温湿度センサーデータ取得
  dataTemp = (float)smeHumidity.readTemperature();
  dataHumid = (float)smeHumidity.readHumidity();

  // OPT3001 照度センサーデータ取得
  OPT3001 result = light.readResult();

  if (result.error == NO_ERROR)
  {
    dataLight = result.lux;
  }

  // ADC081C027（ADC) 電池リーフ電池電圧取得
  uint8_t adcVal1 = 0;
  uint8_t adcVal2 = 0;

  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR, 2);
  adcVal1 = Wire.read();
  adcVal2 = Wire.read();

  if (adcVal1 == 0xff && adcVal2 == 0xff)
  {
    //測定値がFFならバッテリリーフはつながっていない
    adcVal1 = adcVal2 = 0;
  }

  //電圧計算　ADC　* （(リファレンス電圧(3.3V)/ ADCの分解能(256)) * 分圧比（2倍））
  temp_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  dataBatt = (float)(temp_mv / 1000);

#ifdef DEBUG
  Serial.println("");
  Serial.println("--- sensor data ---");
  Serial.println("  Tmp[degC]     = " + String(dataTemp));
  Serial.println("  Hum[%]        = " + String(dataHumid));
  Serial.println("  Lum[lx]       = " + String(dataLight));
  Serial.println("  Accel X,Y,Z   = " + String(dataX_g) + " " + String(dataY_g) + " " + String(dataZ_g));
  Serial.println("  Ang[arc deg]  = " + String(dataTilt));
  Serial.println("  Bat[V]        = " + String(dataBatt));
  Serial.println("");
#endif
}

//-----------------------------------------
// sleep sensor
// センサーリーフをスリープさせる
//-----------------------------------------
void sleepSensor()
{
  // LIS2DH sleep
  accel.setDataRate(LIS3DH_DATARATE_POWERDOWN);

  // HTS221 sleep
  smeHumidity.deactivate();

  // OPT3001 sleep
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  newConfig.ModeOfConversionOperation = B00;
  errorConfig = light.writeConfig(newConfig);
  if (errorConfig != NO_ERROR)
  {
    errorConfig = light.writeConfig(newConfig);
  }
}

//-----------------------------------------
// wakeup sensor
// センサーリーフをスリープから復帰させる
//-----------------------------------------
void wakeupSensor()
{
  #ifdef DEBUG
  Serial.println(F("Wakeup Senser"));
  #endif
  
  // LIS2DH wakeup
  accel.setDataRate(LIS3DH_DATARATE_1_HZ);

  // HTS221 wakeup
  smeHumidity.activate();

  // OPT3001 wakeup
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  newConfig.RangeNumber = B1100;             //automatic full scale
  newConfig.ConvertionTime = B1;             //convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11; //continous conversion
  newConfig.Latch = B1;                      //latch window style

  errorConfig = light.writeConfig(newConfig);
  if (errorConfig != NO_ERROR)
  {

    errorConfig = light.writeConfig(newConfig); //retry
  }
}

//---------------------------------------
// sleep BLE
// BLE リーフをスリープさせる
//---------------------------------------
void sleepBLE()
{
  #ifdef DEBUG
  Serial.println(F("Sleep BLE"));
  #endif
  
  ble112.ble_cmd_le_gap_stop_advertising(0);
  while (ble112.checkActivity());

  ble112.ble_cmd_system_halt(1);
  while (ble112.checkActivity());

  digitalWrite(BLE_WAKEUP, LOW);
  delay(500);
}

//---------------------------------------
// wakeup BLE
// BLEリーフをスリープから復帰させる
//---------------------------------------
void wakeupBLE()
{
  #ifdef DEBUG
  Serial.println(F("Wakeup BLE"));
  #endif
  
  uint8_t *last;
  digitalWrite(BLE_WAKEUP, HIGH);
  delay(500);

  ble112.ble_cmd_system_halt(0);
  while (ble112.checkActivity());

  ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */
  while (ble112.checkActivity(1000)); /* [BGLIB] 受信チェック */
}

//---------------------------------------
// EEPROM
//---------------------------------------
void setupRingBuffer(){
  // rb_addr = (uint16_t)(EEPROM.read(1) << 8) + (uint16_t)(EEPROM.read(0));
  rb_addr = EEPROM.read(0);
  rb_addr += EEPROM.read(1) * 256;
  if(rb_addr >= EEPROM.length() || (rb_addr - 2) % 8 != 0) {
    rb_addr = 2;
  }
#ifdef DEBUG
  Serial.print("EEPROM length = ");
  Serial.println(EEPROM.length());
  Serial.print("rb_addr = ");
  Serial.println(rb_addr);
#endif
}

void writeEEPROM(){
  uint8_t temp, humid, light, battVolt;
  
  if(rb_addr + 8 >= EEPROM.length()){
    rb_addr = 2;
  }

#ifdef DEBUG
  Serial.print(F("writeEEPROM(): ADDR="));
  Serial.println(rb_addr);
#endif

  temp = (uint8_t)(dataTemp * 256);
  humid = (uint8_t)(dataHumid * 256);
  light = (uint8_t)dataLight;
  battVolt = (uint8_t)(dataBatt * 256);

  EEPROM.write(rb_addr + 0, (temp >> 8) & 0xFF);
  EEPROM.write(rb_addr + 1, temp & 0xFF);
  EEPROM.write(rb_addr + 2, (humid >> 8) & 0xFF);
  EEPROM.write(rb_addr + 3, humid & 0xFF);
  EEPROM.write(rb_addr + 4, (light >> 8) & 0xFF);
  EEPROM.write(rb_addr + 5, light & 0xFF);
  EEPROM.write(rb_addr + 6, (battVolt >> 8) & 0xFF);
  EEPROM.write(rb_addr + 7, battVolt & 0xFF);

  // write next ring buffer address
  rb_addr += 8;
  EEPROM.write(0, rb_addr & 0xFF);
  EEPROM.write(1, rb_addr >> 8);
#ifdef DEBUG
  Serial.print("next addr = ");
  Serial.println(rb_addr);
#endif
}

void shutdownAllDevices(){
  sleepBLE();

#ifdef DEBUG
  Serial.print(F("Shutdown STM32"));
  Serial.print(F(" (restart after "));
  Serial.print(SLEEP_INTERVAL);
  Serial.println(F(" seconds)"));
  Serial.flush();
#endif
  LowPower.shutdown(SLEEP_INTERVAL * 1000);

#ifdef DEBUG
  Serial.println(F("This will never been called!"));
#endif
}

void setup()
{
  Serial.begin(115200);
  LowPower.begin(); // Configure low power

  Wire.begin(); // I2C 100KHz

#ifdef DEBUG
  Serial.println(F("<<< Wake up <<<"));
  Serial.println(F("========================================="));
  Serial.println(F("setup start"));
#endif

  setupPort();
  delay(10);

  setupRingBuffer();

  setupSensor();
  setupBLE();

#ifdef DEBUG
  Serial.println(F("setup end"));
  Serial.println(F("========================================="));
#endif

  wakeupBLE();
  
  wakeupSensor();
  getSensor();
  sleepSensor();

  writeEEPROM();

  StartAdvData();
#ifdef DEBUG
  Serial.print(F("Start advertising"));
  Serial.print(F(" ("));
  Serial.print(WAKE_INTERVAL);
  Serial.println(F("s)"));
  Serial.flush();
#endif

  // Continue Advertising
  for(int i=0; i<WAKE_INTERVAL; i++){
    delay(1000);
    ble112.checkActivity(1);
  }
  // bBleConnectedがtrueになるのはこのタイミング

  // BLE ConnectedでなければSleep & Shutdown
  if(!bBleConnected){
    shutdownAllDevices();
  }
}

void loop()
{
  if(bBleConnected){
    // when ble is connected, this scope will run continuously.
    if(bBleSendData){
      #ifdef DEBUG
      Serial.println("Start to send data.");
      #endif

      /*
      char sendData[40];
      uint8_t sendLen = sprintf(sendData, "Hello Leafony\n");
      ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)sendData );
      while (ble112.checkActivity(1000));
      */

      for (int i=2; i<rb_addr; i+=8) {
        char sendData[40];
        char c_temp[7], c_humid[7], c_illum[7], c_batt[0];
        float temp =  (EEPROM.read(i + 0) << 8 + EEPROM.read(i + 1)) / 256.0;
        // float humid = (EEPROM.read(i + 2) << 8 + EEPROM.read(i + 3)) / 256.0;
        // float illum = (EEPROM.read(i + 4) << 8 + EEPROM.read(i + 5));
        // float batt =  (EEPROM.read(i + 6) << 8 + EEPROM.read(i + 7)) / 256;
        
        dtostrf(temp,4,1,c_temp);
        //dtostrf(humid,2,0,c_humid);
        //dtostrf(illum,5,0,c_illum);
        //dtostrf(batt,3,2,c_batt);
        
        uint8_t sendLen = sprintf(sendData, "%s,%s,%s,%s", c_temp, "12", "34567", "89.0");
        ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)sendData );
        while (ble112.checkActivity(1000));
      }

      #ifdef DEBUG
      Serial.println("Finish to send data.");
      #endif
      
      // after all the data trasnported,
      bBleSendData = false;
    } else {
      ble112.checkActivity(100); 
    }
  } else {
    // when ble is disconnected, shutdown all devices.
    shutdownAllDevices();
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
  Serial.print(F("###\tgatt_server_attribute_value: { "));
  Serial.print(F("connection: ")); Serial.print(msg -> connection, HEX);
  Serial.print(F(", attribute: ")); Serial.print((uint16_t)msg -> attribute, HEX);
  Serial.print(F(", att_opcode: ")); Serial.print(msg -> att_opcode, HEX);
  Serial.print(", offset: "); Serial.print((uint16_t)msg -> offset, HEX);
  Serial.print(", value_len: "); Serial.print(msg -> value.len, HEX);
  Serial.print(", value_data: "); Serial.print(rcv_data);
  Serial.println(F(" }"));
#endif

  // Command
  if(rcv_data.indexOf("get") == 0){
    bBleSendData = true;
  } else if(rcv_data.indexOf("set") == 0){
    bBleSendData = false;
  }
}

// called when the connection is opened
void my_evt_le_connection_opend(const ble_msg_le_connection_opend_evt_t *msg)
{
#ifdef DEBUG
  Serial.print(F("###\tconnection_opend: { "));
  Serial.print(F("address: "));
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
  Serial.print(F("###\tconnection_closed: { "));
  Serial.print(F("reason: "));
  Serial.print((uint16_t)msg->reason, HEX);
  Serial.print(F(", connection: "));
  Serial.print(msg->connection, HEX);
  Serial.println(F(" }"));
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
  sprintf(cAddr, "Device name is Leaf_A_#%05d ", addr);
  Serial.println(cAddr);
}
