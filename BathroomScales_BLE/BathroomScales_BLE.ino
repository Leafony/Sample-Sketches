//=====================================================================
//  Leafony Platform sample sketch
//     Application  : Bathroom Scales BLE demo
//     Processor    : ATmega328P (3.3V /8MHz)
//     Confirmed in Arduino IDE 1.8.7
//
//     Leaf configuration
//       (1) AP01 AVR MCU
//       (2) AC02 BLE Sugar
//       (3) AZ01 USB
//       (4) HX711
//
//    (c)2021 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/03/15 First release
//=====================================================================
//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <MsTimer2.h>                       // Timer
#include <SoftwareSerial.h>                 // Software UART
#include <Wire.h>                           // I2C
#include "TBGLib.h"                         // BLE
#include <HX711_ADC.h>
#include <EEPROM.h>

//===============================================
// BLE Unique Name (Local device name)
// 最大16文字（ASCIIコード）まで
//===============================================
//                     |1234567890123456|
String strDeviceName = "Leafony_AW";

//===============================================
// シリアルモニタへの出力
//      #define SERIAL_MONITOR = 出力あり
//    //#define SERIAL_MONITOR = 出力なし（コメントアウトする）
//===============================================
#define SERIAL_MONITOR

//===============================================
// シリアルモニタへのデバック出力
//      #define DEBUG = 出力あり
//    //#define DEBUG = 出力なし（コメントアウトする）
//===============================================
//#define DEBUG

//-----------------------------------------------
// 送信間隔の設定
//  SEND_INTERVAL  :送信間隔（センサーデータを送る間隔  1秒単位
//-----------------------------------------------
#define SEND_INTERVAL   (1)                 // 1s

//-----------------------------------------------
// IOピンの名前定義
// 接続するリーフに合わせて定義する
//-----------------------------------------------
const int HX711_dout    =  4;      //mcu > HX711 dout pin
const int HX711_sck     =  5;      //mcu > HX711 sck pin
const int D7_BLE_WAKEUP =  7;
const int D15_BLE_TX    = 15;
const int D16_BLE_RX    = 16;

//-----------------------------------------------
// プログラム内で使用する定数定義
//-----------------------------------------------
//------------------------------
// I2Cアドレス
//------------------------------
#define BATT_ADC_ADDR   0x50                // Battery ADC

//------------------------------
// Loop interval
// MsTimer2のタイマー割り込み発生間隔(ms)
//------------------------------
#define LOOP_INTERVAL 125                   // 125ms interval

//------------------------------
// BLE
//------------------------------
#define BLE_STATE_STANDBY               (0)
#define BLE_STATE_SCANNING              (1)
#define BLE_STATE_ADVERTISING           (2)
#define BLE_STATE_CONNECTING            (3)
#define BLE_STATE_CONNECTED_MASTER      (4)
#define BLE_STATE_CONNECTED_SLAVE       (5)

//---------------------------------------------------------------------
// object
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);


//------------------------------
// BLE
//------------------------------
SoftwareSerial Serialble(D16_BLE_RX, D15_BLE_TX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0 );

//---------------------------------------------------------------------
// プログラムで使用する変数定義
//---------------------------------------------------------------------
//------------------------------
// Loop counter
//------------------------------
uint8_t iLoop1s = 0;
uint8_t iSendCounter = 0;

//------------------------------
// Event
//------------------------------
bool event1s = false;

//------------------------------
// Interval Timer interrupt
//------------------------------
volatile bool bInterval = false;

//------------------------------
// HX711
//------------------------------
const int calVal_eepromAdress = 0;
unsigned long t = 0;
float dataWeight = 0;
float totalWeight = 0;
int addTimes = 0;

//------------------------------
// Battery
//------------------------------
float dataBatt = 0;

//------------------------------
// BLE
//------------------------------
bool bBLEconnect = false;
bool bBLEsendData = false;

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0;         // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF;        // 0xFF = no bonding, otherwise = bonding handle

//=====================================================================
// setup
//=====================================================================
void setup() {
  delay(500);

  Serial.begin(115200);     // UART 115200bps
  Wire.begin();             // I2C 100kHz
#ifdef SERIAL_MONITOR
    Serial.println(F("========================================="));
    Serial.println("Starting...");
#endif

  setupPort();
  delay(10);
  setupSensor();
  setupBLE();

  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));
  delay(1000);

  noInterrupts();
  setupTCInt();
  interrupts();

  MsTimer2::start();                        // Timer2 inverval start
#ifdef SERIAL_MONITOR
    Serial.println();
    Serial.println("=========================================");
    Serial.println(F("loop start"));
    Serial.println();
#endif
}

//-----------------------------------------------
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//-----------------------------------------------
void setupPort(){
  pinMode(D7_BLE_WAKEUP, OUTPUT);           // PD7 : digital 7 = BLE Wakeup/Sleep
  digitalWrite(D7_BLE_WAKEUP, HIGH);        // BLE Wakeup

}

//---------------------------------------------------------------------
// 各デバイスの初期設定
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
void setupSensor(){
// LoadCell Setup
  LoadCell.begin();
  unsigned long stabilizingtime = 2000;   // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                   //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
//    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    Serial.println("Timeout, check MCU>HX711");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0);           // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate();                            //start calibration procedure
}

//=====================================================================
// 割り込み処理
//=====================================================================
//----------------------------------------------
// 割り込み処理初期設定
// Timer interrupt (interval=125ms, int=overflow)
// メインループのタイマー割り込み設定
//----------------------------------------------
void setupTCInt(){
  MsTimer2::set(LOOP_INTERVAL, intTimer);
}

//----------------------------------------------
// Timer INT
// タイマー割り込み関数
//----------------------------------------------
void intTimer(){
  bInterval = true;
}

//=====================================================================
// Loop
//=====================================================================
//---------------------------------------------------------------------
// Main loop
//---------------------------------------------------------------------
void loop(){
  loopHX711();

  //-----------------------------------------------------
  // Timer interval 125ms で1回ループ
  //-----------------------------------------------------
  if (bInterval == true){
     bInterval = false;

    //--------------------------------------------
    loopCounter();                    // loop counter
    //--------------------------------------------
    // 1sに1回実行する
    //--------------------------------------------
    if(event1s == true){
      event1s = false;                // initialize parameter
      loopSensor();                   // sensor read
      bt_sendData();                  // Data send
    }
  }
  loopBleRcv();
}

//---------------------------------------------------------------------
// Counter
// メインループのループ回数をカウントし
// 1秒間隔でセンサーデータの取得とBLEの送信をONにする
//---------------------------------------------------------------------
void loopCounter(){
  iLoop1s += 1;
  //--------------------
  // 1s period
  //--------------------
  if (iLoop1s >=  8){                 // 125ms x 8 = 1s
    iLoop1s = 0;

    iSendCounter  += 1;
    if (iSendCounter >= SEND_INTERVAL){
      iSendCounter = 0;
      event1s = true;
    }
  }
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
void loopHX711(){
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;                //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();

      if (addTimes < 10){
        ++addTimes;
        totalWeight = totalWeight + i;
      }
      else {
        dataWeight = totalWeight / addTimes;
        totalWeight = 0;
        addTimes = 0;
        }

//      Serial.println(i);
//      Serial.print("total val: ");
//      Serial.println(totalWeight);
//      Serial.print("addTimes: ");
//      Serial.println(addTimes);

      newDataReady = 0;
      t = millis();

      }
    }
 
  // receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();      //tare
    else if (inByte == 'r') calibrate();            //calibrate
    else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}

//---------------------------------------------------------------------
// Sensor
// センサーデータ取得がONのとき、各センサーからデータを取得
// コンソール出力がONのときシリアルに測定値と計算結果を出力する
//---------------------------------------------------------------------
void loopSensor(){

  //-------------------------
  // ADC081C027（ADC)
  // 電池リーフ電池電圧取得
  //-------------------------
  double temp_mv;
  uint8_t adcVal1 = 0;
  uint8_t adcVal2 = 0;

  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR,2);
  adcVal1 = Wire.read();
  adcVal2 = Wire.read();

  if (adcVal1 == 0xff && adcVal2 == 0xff) {
    //測定値がFFならバッテリリーフはつながっていない
    adcVal1 = adcVal2 = 0;
  }

  //電圧計算　ADC　* （(リファレンス電圧(3.3V)/ ADCの分解能(256)) * 分圧比（２倍））
  temp_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  dataBatt = (float)(temp_mv / 1000);
}

//---------------------------------------------------------------------
// Send sensor data
// センサーデータをセントラルに送る文字列に変換してBLEリーフへデータを送る
//---------------------------------------------------------------------
void bt_sendData(){
  float value;
  char weight[7], battVolt[7];
  char sendData[40];
  uint8 sendLen;

  //-------------------------
  //センサーデータを文字列に変換
  //dtostrf(変換する数字,変換される文字数,小数点以下の桁数,変換した文字の格納先);
  //変換される文字数を-にすると変換される文字は左詰め、+なら右詰めとなる
  //-------------------------

  //-------------------------
  // Weight
  //-------------------------
  value = dataWeight;
  if (value < 0){
   value = 0;
  }
  dtostrf(value,4,1,weight);

  //-------------------------
  // Battery Voltage (4Byte)
  //-------------------------
  value = dataBatt;
  if (value >= 10){
   value = 9.99;
  }
  dtostrf(value, 4, 2, battVolt);

  //-------------------------
  trim(weight);
  trim(battVolt);

  //-------------------------
  // BLE Send Data
  //-------------------------
  if( bBLEsendData == true ){     // BLE送信
    // WebBluetoothアプリ用フォーマット
    sendLen = sprintf(sendData, "%04s,%04s\n", weight, battVolt);
    // BLEデバイスへの送信
    ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)sendData );
    while (ble112.checkActivity(1000));
  }
    //-------------------------
    // シリアルモニタ表示
    //-------------------------
#ifdef SERIAL_MONITOR
/*
    // 複数行に表示する場合
    Serial.println("--- sensor data ---");    
    Serial.println("  Weight[kg]     = " + String(dataWeight));
    Serial.println("  Bat[V]        = " + String(dataBatt));
*/
  Serial.println("SensorData: Weight=" + String(weight) + ", Vbat=" + String(battVolt));
#endif
}
//====================================================================

//---------------------------------------
// trim
// 文字列配列からSPを削除する
//---------------------------------------
void trim(char * data){
  int i = 0, j = 0;

  while (*(data + i) != '\0'){
    if (*(data + i) != ' '){
      *(data + j) = *(data + i);
      j++;
    }
    i++;
  }
  *(data + j) = '\0';
}

//=====================================================================
// HX711
//=====================================================================
//-----------------------------------------------
// 
//-----------------------------------------------
void calibrate() {
  Serial.println("Start calibration:");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("set the tare offset.");

  boolean _resume = false;
  boolean _resume2 = false;

  while (_resume == false) {
    LoadCell.update();

    if (_resume2 == false) {
      LoadCell.tareNoDelay();
      _resume2 =true;
    }

    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  float newCalibrationValue = 18793;
  LoadCell.setCalFactor(newCalibrationValue);

  EEPROM.put(calVal_eepromAdress, newCalibrationValue);
  EEPROM.get(calVal_eepromAdress, newCalibrationValue);

  Serial.print("New calibration value is: ");
  Serial.println(newCalibrationValue);
  Serial.println("End calibration");
}

//-----------------------------------------------
// 
//-----------------------------------------------
void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value ");
  float newCalibrationValue =0;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;

  Serial.print("Save this value to EEPROM ");

  EEPROM.put(calVal_eepromAdress, newCalibrationValue);
  EEPROM.get(calVal_eepromAdress, newCalibrationValue);

  Serial.print("New calibration value is: ");
  Serial.println(newCalibrationValue);

  Serial.println("End change value");
}

//=====================================================================
// I2C　制御関数
//=====================================================================
//-----------------------------------------------
//I2C スレーブデバイスに1バイト書き込む
//-----------------------------------------------
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}

//-----------------------------------------------
//I2C スレーブデバイスから1バイト読み込む
//-----------------------------------------------
unsigned char i2c_read_byte(int device_address, int reg_address){
  int read_data = 0;

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, 1);
  read_data = Wire.read();

  return read_data;
}

//=====================================================================
// BLE
//=====================================================================
//-----------------------------------------------
//  Setup BLE
//-----------------------------------------------
void setupBLE(){
    uint8  stLen;
    uint8 adv_data[31];

    // set up internal status handlers (these are technically optional)
    ble112.onBusy = onBusy;
    ble112.onIdle = onIdle;
    ble112.onTimeout = onTimeout;
    // ONLY enable these if you are using the <wakeup_pin> parameter in your firmware's hardware.xml file
    // BLE module must be woken up before sending any UART data

    // set up BGLib response handlers (called almost immediately after sending commands)
    // (these are also technicaly optional)

    // set up BGLib event handlers
    /* [gatt_server] */
    ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value;    /* [BGLib] */
    /* [le_connection] */
    ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;                /* [BGLib] */
    ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed;              /* [BGLib] */
    /* [system] */
    ble112.ble_evt_system_boot = my_evt_system_boot;                                /* [BGLib] */

    ble112.ble_evt_system_awake = my_evt_system_awake;
    ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;
    /*  */

    Serialble.begin(9600);

    /* setting */
    /* [set Advertising Data] */
    uint8 ad_data[21] = {
        (2),                                    // field length
        BGLIB_GAP_AD_TYPE_FLAGS,                // field type (0x01)
        (6),                                    // data
        (1),                                    // field length (1は仮の初期値)
        BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE    // field type (0x09)
    };
    /*  */
    size_t lenStr2 = strDeviceName.length();
    ad_data[3] = (lenStr2 + 1);                     // field length
    uint8 u8Index;
    for( u8Index=0; u8Index < lenStr2; u8Index++){
      ad_data[5 + u8Index] = strDeviceName.charAt(u8Index);
    }
    /*   */
    stLen = (5 + lenStr2);
    ble112.ble_cmd_le_gap_set_adv_data( SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data );
    while (ble112.checkActivity(1000));                 /* 受信チェック */

    /* interval_min :   40ms( =   64 x 0.625ms ) */
    /* interval_max : 1000ms( = 1600 x 0.625ms ) */
    ble112.ble_cmd_le_gap_set_adv_parameters( 64, 1600, 7 );    /* [BGLIB] <interval_min> <interval_max> <channel_map> */
    while (ble112.checkActivity(1000));                         /* [BGLIB] 受信チェック */

    /* start */
    //ble112.ble_cmd_le_gap_start_advertising(1, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);
    ble112.ble_cmd_le_gap_start_advertising( 0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE );    // index = 0
    while (ble112.checkActivity(1000));                 /* 受信チェック */
    /*  */
}

//-----------------------------------------
// BLEからデータが送信されていたらデータを取得し、取得データに従い
// 処理を実施
//-----------------------------------------
void loopBleRcv( void ){
    // keep polling for new data from BLE
    ble112.checkActivity(0);                    /* 受信チェック */

    /*  */
    if (ble_state == BLE_STATE_STANDBY) {
        bBLEconnect = false;                    /* [BLE] 接続状態 */
    } else if (ble_state == BLE_STATE_ADVERTISING) {
        bBLEconnect = false;                    /* [BLE] 接続状態 */
    } else if (ble_state == BLE_STATE_CONNECTED_SLAVE) {
        /*  */
        bBLEconnect = true;                     /* [BLE] 接続状態 */
        /*  */
    }
}

//=====================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
//=====================================================================
//-----------------------------------------------
// called when the module begins sending a command
void onBusy() {
    // turn LED on when we're busy
    //digitalWrite( D13_LED, HIGH );
}

//-----------------------------------------------
// called when the module receives a complete response or "system_boot" event
void onIdle() {
    // turn LED off when we're no longer busy
    //digitalWrite( D13_LED, LOW );
}

//-----------------------------------------------
// called when the parser does not read the expected response in the specified time limit
void onTimeout() {
    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false;                    /* [BLE] 接続状態 */
    bBLEsendData = false;
}

//-----------------------------------------------
// called immediately before beginning UART TX of a command
void onBeforeTXCommand() {
}

//-----------------------------------------------
// called immediately after finishing UART TX
void onTXCommandComplete() {
    // allow module to return to sleep (assuming here that digital pin 5 is connected to the BLE wake-up pin)
}
/*  */

//-----------------------------------------------
void my_evt_gatt_server_attribute_value( const struct ble_msg_gatt_server_attribute_value_evt_t *msg ) {
    uint16 attribute = (uint16)msg -> attribute;
    uint16 offset = 0;
    uint8 value_len = msg -> value.len;

    uint8 value_data[20];
    String rcv_data;
    rcv_data = "";
    for (uint8_t i = 0; i < value_len; i++) {
        rcv_data += (char)(msg -> value.data[i]);
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

    if( rcv_data.indexOf("SND") == 0 ){
        bBLEsendData = true;
    } else if( rcv_data.indexOf("STP") == 0 ){
        bBLEsendData = false;
    } else if(rcv_data.indexOf("PLS") == 0){
    } else if(rcv_data.indexOf("MNS") == 0){
    }
}
/*  */

//-----------------------------------------------
void my_evt_le_connection_opend( const ble_msg_le_connection_opend_evt_t *msg ) {
    #ifdef DEBUG
        Serial.print(F("###\tconnection_opend: { "));
        Serial.print(F("address: "));
        // this is a "bd_addr" data type, which is a 6-byte uint8_t array
        for (uint8_t i = 0; i < 6; i++) {
            if (msg -> address.addr[i] < 16) Serial.write('0');
            Serial.print(msg -> address.addr[i], HEX);
        }
        Serial.println(" }");
    #endif
    /*  */
    ble_state = BLE_STATE_CONNECTED_SLAVE;
}
/*  */
//-----------------------------------------------
void my_evt_le_connection_closed( const struct ble_msg_le_connection_closed_evt_t *msg ) {
    #ifdef DEBUG
        Serial.print(F("###\tconnection_closed: { "));
        Serial.print(F("reason: ")); Serial.print((uint16_t)msg -> reason, HEX);
        Serial.print(F(", connection: ")); Serial.print(msg -> connection, HEX);
        Serial.println(F(" }"));
    #endif

    // after disconnection, resume advertising as discoverable/connectable (with user-defined advertisement data)
    //ble112.ble_cmd_le_gap_set_mode( LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE );
    //ble112.ble_cmd_le_gap_start_advertising(1, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);
    ble112.ble_cmd_le_gap_start_advertising( 0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE );    // index = 0
    while (ble112.checkActivity(1000));

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false;                    /* [BLE] 接続状態 */
    bBLEsendData = false;
}
/*  */

//-----------------------------------------------
void my_evt_system_boot( const ble_msg_system_boot_evt_t *msg ) {
    #ifdef DEBUG
        Serial.print( "###\tsystem_boot: { " );
        Serial.print( "major: " ); Serial.print(msg -> major, HEX);
        Serial.print( ", minor: " ); Serial.print(msg -> minor, HEX);
        Serial.print( ", patch: " ); Serial.print(msg -> patch, HEX);
        Serial.print( ", build: " ); Serial.print(msg -> build, HEX);
        Serial.print( ", bootloader_version: " ); Serial.print( msg -> bootloader, HEX );           /*  */
        Serial.print( ", hw: " ); Serial.print( msg -> hw, HEX );
        Serial.println( " }" );
    #endif

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;
}

//-----------------------------------------------
void my_evt_system_awake(const ble_msg_system_boot_evt_t *msg ) {
   ble112.ble_cmd_system_halt( 0 );
   while (ble112.checkActivity(1000));
}

//-----------------------------------------------
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg ){
#ifdef DEBUG
  Serial.print( "###\tsystem_get_bt_address: { " );
  Serial.print( "address: " );
  for (int i = 0; i < 6 ;i++){
    Serial.print(msg->address.addr[i],HEX);
  }
  Serial.println( " }" );
#endif
  unsigned short addr = 0;
  char cAddr[30];
  addr = msg->address.addr[0] + (msg->address.addr[1] *0x100);
  sprintf(cAddr, "Device name is Leaf_A_#%05d ",addr);
  Serial.println(cAddr);
}
