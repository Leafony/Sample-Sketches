//=====================================================================
//  Leafony Platform sample sketch
//     Application  : Motor BLE demo
//     Processor    : ATmega328P (3.3V/8MHz)
//     Confirmed in Arduino IDE 1.8.13
//
//     Leaf configuration
//       (1) AC02 BLE Sugar
//       (2) AI01 4-Sensors
//       (4) AP01 AVR MCU
//       (5) AZ01 USB
//       (6) Motor
//
//    (c)2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/11/06 First release
//=====================================================================
//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <MsTimer2.h>                       // Timer
#include <SoftwareSerial.h>                 // Software UART
#include <Wire.h>                           // I2C

#include "TBGLib.h"                         // BLE
#include <Adafruit_HTS221.h>                // humidity and temperature sensor
#include <ClosedCube_OPT3001.h>             // Ambient Light Sensor

//===============================================
// BLE Unique Name (Local device name)
// 最大16文字（ASCIIコード）まで
//===============================================
//                     |1234567890123456|
String strDeviceName = "Motor_AC02";

//===============================================
// シリアルモニタへの出力
//      #define SERIAL_MONITOR = 出力あり
//    //#define SERIAL_MONITOR = 出力なし（コメントアウトする）
//===============================================
//#define SERIAL_MONITOR

//===============================================
// シリアルモニタへのデバッグ出力
//      #define DEBUG = 出力あり
//    //#define DEBUG = 出力なし（コメントアウトする）
//===============================================
//#define DEBUG

//-----------------------------------------------
// 送信間隔の設定
//  SEND_INTERVAL  :送信間隔（センサーデータを送る間隔  1秒単位
//-----------------------------------------------
#define SEND_INTERVAL   (1)             // 1s

//-----------------------------------------------
// IOピン一覧
//-----------------------------------------------
//  D0              0                   // PD0  (RXD)
//  D1              1                   // PD1  (TXD)
//  D2              2                   // PD2  (INT0)
//  D3              3                   // PD3  (INT1)
//  D4              4                   // PD4
//  D5              5                   // PD5
//  D6              6                   // PD6
//  D7              7                   // PD7
//  D8              8                   // PB0  (S-UART2_RX)
//  D9              9                   // PB1  (S-UART2_TX)
//  D10             10                  // PB2  (SS)
//  D11             11                  // PB3  (MOSI)
//  D12             12                  // PB4  (MISO)
//  D13             13                  // PB5  (SCK/LED)

//  D14             14                  // [A0] PC0
//  D15             15                  // [A1] PC1
//  D16             16                  // [A2] PC2
//  D17             17                  // [A3] PC3

//-----------------------------------------------
// IOピンの名前定義
// 接続するリーフに合わせて定義する
//-----------------------------------------------
#define VM_ON           3                   // モータ電源制御
#define A_IN1           12                  // 回転制御1
#define A_IN2           4                   // 回転制御2
#define A_PWM           5                   // 速度制御PWM

#define D7_BLE_WAKEUP   7                   // PD7
#define D13_LED         13                  // PB5  (SCK/LED)

#define rdMoisture      A0                  // 土壌水分センサ入力
#define D15_BLE_TX      15                  // [A1] PC1
#define D16_BLE_RX      16                  // [A2] PC2

//-----------------------------------------------
// プログラム内で使用する定数定義
//-----------------------------------------------
//------------------------------
// I2Cアドレス
//------------------------------
#define OPT3001_ADDRESS         0x45        // Ambient Light Sensor (ADDR pin = VCC)
#define BATT_ADC_ADDR           0x50        // Battery ADC

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
// BLE
//------------------------------
SoftwareSerial Serialble(D16_BLE_RX, D15_BLE_TX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0 );

//------------------------------
// Sensor
//------------------------------
ClosedCube_OPT3001 light;

//---------------------------------------------------------------------
// プログラムで使用する変数定義
//---------------------------------------------------------------------
String receiveData = "MNS";                   // MNS:モータ停止 PLS:モータ回転

//------------------------------
// シリアルモニタ表示用データ
//------------------------------
#ifdef SERIAL_MONITOR
String titleData1 = "Moi";
String titleData2 = "Tmp";
String titleData3 = "Hum";
String titleData4 = "Lum";
String titleData5 = "Bat";

String unitData1 = "%";
String unitData2 = "degC";
String unitData3 = "%RH";
String unitData4 = "lx";
String unitData5 = "V";
#endif

//------------------------------
// センサデータ
//------------------------------
float data1 = 0;          // Moisture
float data2 = 0;          // Temperature
float data3 = 0;          // Humidity
float data4 = 0;          // Illuminance
float data5 = 0;          // Battery Voltage

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

//---------------------------
// HTS221 : Temperature/Humidity
//---------------------------
Adafruit_HTS221 hts;

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
  Wire.begin();             // I2C 100kHz
  Serial.begin(115200);     // UART 115200bps
#ifdef SERIAL_MONITOR
    Serial.println(F("========================================="));
    Serial.println(F("setup start"));
#endif

  setupPort();
  delay(10);

  noInterrupts();
  setupTCInt();
  interrupts();

  setupSensor();
  setupBLE();

  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));
  delay(1000);

  MsTimer2::start();                        // Timer2 inverval start
#ifdef SERIAL_MONITOR
    Serial.println(F(""));
    Serial.println("=========================================");
    Serial.println(F("loop start"));
    Serial.println(F(""));
#endif
}

//-----------------------------------------------
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//-----------------------------------------------
void setupPort(){
  pinMode(VM_ON, OUTPUT);                   // モータ電源制御
  digitalWrite(VM_ON, LOW);                 // モータ電源Off

  // ch Aの制御用ピン設定
  pinMode(A_IN1, OUTPUT);                   // 回転制御1
  pinMode(A_IN2, OUTPUT);                   // 回転制御2
  pinMode(A_PWM, OUTPUT);                   // PWMによるスピード制御 (0-255)

  pinMode(D7_BLE_WAKEUP, OUTPUT);           // D7  PD7 : BLE Wakeup/Sleep
  digitalWrite(D7_BLE_WAKEUP, HIGH);        // BLE Wakeup

  pinMode(D13_LED, OUTPUT);                 // D13 PB5 : LED
  digitalWrite(D13_LED, LOW);               // LED off

  pinMode(rdMoisture, INPUT);               // A0 土壌水分センサ
}

//---------------------------------------------------------------------
// 各デバイスの初期設定
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
void setupSensor(){
  //----------------------------
  // HTS221 : Humidity and Temperature sensor
  //----------------------------
  while (!hts.begin_I2C()) {
#ifdef DEBUG
    Serial.println("Failed to find HTS221");
#endif
    delay(10);
  }
  delay(10);

  //----------------------------
  // OPT3001 : Ambient Light Sensor
  //----------------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  light.begin(OPT3001_ADDRESS);

  newConfig.RangeNumber = B1100;                    // automatic full scale
  newConfig.ConvertionTime = B1;                    // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;        // continous conversion
  newConfig.Latch = B0;                             // hysteresis-style

  errorConfig = light.writeConfig(newConfig);

  if(errorConfig != NO_ERROR){
    errorConfig = light.writeConfig(newConfig);     // retry
  }
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

      if(receiveData == "PLS"){
        digitalWrite(VM_ON, HIGH);          // モータ電源On
        delay(10);
        chA_CW();                           // モータA: 正転
        analogWrite(A_PWM, 255);            // モータA: フルスピード
      }
      else if(receiveData == "MNS"){
        analogWrite(A_PWM, 0);              // モータA: 0
        chA_stop();
        delay(10);
        digitalWrite(VM_ON, LOW);          // モータ電源Off
      }
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
// Sensor
// センサーデータ取得がONのとき、各センサーからデータを取得
// コンソール出力がONのときシリアルに測定値と計算結果を出力する
//---------------------------------------------------------------------
void loopSensor(){
    //----------------------------
    // Moisture
    //----------------------------
    data1 = analogRead(rdMoisture);

    //-------------------------
    // HTS221
    // 温湿度センサーデータ取得
    //-------------------------
    sensors_event_t humidity;
    sensors_event_t temp;
    hts.getEvent(&humidity, &temp);               // populate temp and humidity objects with fresh data
    data2 = temp.temperature;                     // 温度
    data3 =humidity.relative_humidity;            // 湿度

    //-------------------------
    // 温度と湿度の2点補正
    //-------------------------
    data2=TM0+(TM1-TM0)*(data2-TL0)/(TL1-TL0);    // 温度補正
    data3=HM0+(HM1-HM0)*(data3-HL0)/(HL1-HL0);    // 湿度補正

    //-------------------------
    // OPT3001
    // 照度センサーデータ取得
    //-------------------------
    OPT3001 result = light.readResult();

    if(result.error == NO_ERROR){
      data4 = result.lux;
    }

  //-------------------------
  // ADC081C027（ADC)
  // 電池リーフ電池電圧取得
  //-------------------------
  double Vbat_mv;
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

  //電圧計算　ADC * ((リファレンス電圧(3.3V)/ ADCの分解能(256)) * 分圧比(2倍))
  Vbat_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  data5 = (float)(Vbat_mv / 1000);
}

//---------------------------------------------------------------------
// Send sensor data
// センサーデータをセントラルに送る文字列に変換してBLEリーフへデータを送る
//---------------------------------------------------------------------
void bt_sendData(){
  float value;
  char sendData1[7], sendData2[7], sendData3[7], sendData4[7],sendData5[7];
  char sendData[40];
  uint8 sendLen;

  //-------------------------
  //センサーデータを文字列に変換
  //dtostrf(変換する数字,変換される文字数,小数点以下の桁数,変換した文字の格納先);
  //変換される文字数を-にすると変換される文字は左詰め、+なら右詰めとなる
  //-------------------------
  //-------------------------
  // Data1 Moisture
  //-------------------------
  value = data1;
//  if(value >= 100){
//    value = 99.9;
//  }
  dtostrf(value,4,0,sendData1);

  //-------------------------
  // Data2 Temperature (4Byte)
  //-------------------------
  value = data2;
  if(value >= 100){
    value = 99.9;
  }
  else if(value <= -10){
    value = -9.9;
  }
  dtostrf(value,4,1,sendData2);

  //-------------------------
  // Data3 Humidity (4Byte)
  //-------------------------
  value = data3;
  dtostrf(value,4,1,sendData3);

  //-------------------------
  // Data4 Ambient Light (5Byte)
  //-------------------------
  value = data4;
  if(value >= 100000){
    value = 99999;
  }
  dtostrf(value,5,0,sendData4);

  //-------------------------
  // Data5 Battery Voltage (4Byte)
  //-------------------------
  value = data5;
  if (value >= 10){
   value = 9.99;
  }
  dtostrf(value, 4, 1, sendData5);

  //-------------------------
  trim(sendData1);
  trim(sendData2);
  trim(sendData3);
  trim(sendData4);
  trim(sendData5);

  //-------------------------
  // BLE Send Data
  //-------------------------
  if( bBLEsendData == true ){
    // WebBluetoothアプリ用フォーマット
    sendLen = sprintf(sendData, "%04s,%04s,%04s,%04s,%04s\n", sendData1, sendData2, sendData3, sendData4, sendData5);
    // BLEデバイスへの送信
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, sendLen, (const uint8 *)sendData);

    while (ble112.checkActivity(1000));
  }
    //-------------------------
    // シリアルモニタ表示
    //-------------------------
#ifdef SERIAL_MONITOR
    Serial.println("--- sensor data ---");    
    Serial.println(titleData1 + " =" + "\t" + String(data1) + " [" + unitData1 +"]");
    Serial.println(titleData2 + " =" + "\t" + String(data2) + " [" + unitData2 +"]");
    Serial.println(titleData3 + " =" + "\t" + String(data3) + " [" + unitData3 +"]");
    Serial.println(titleData4 + " =" + "\t" + String(data4) + " [" + unitData4 +"]");
    Serial.println(titleData5 + " =" + "\t" + String(data5) + " [" + unitData5 +"]");

/*
    // 1行で表示する場合
    Serial.print("###\tSensor data: { ");
    Serial.print(titleData1 + "=" + String(data1) + ", ");
    Serial.print(titleData2 + "=" + String(data2) + ", ");
    Serial.print(titleData3 + "=" + String(data3) + ", ");
    Serial.print(titleData4 + "=" + String(data4) + ", ");
    Serial.print(titleData5 + "=" + String(data5) + ", ");
*/
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
// Motor　制御関数
//=====================================================================
//-----------------------------------------------
// ch A 正転
//-----------------------------------------------
void chA_CW(){
  digitalWrite(A_IN1, HIGH);
  digitalWrite(A_IN2, LOW);
}

//-----------------------------------------------
// ch A 逆転
//-----------------------------------------------
void chA_CCW(){
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, HIGH);
}

//-----------------------------------------------
// ch A ブレーキ
//-----------------------------------------------
void chA_brake(){
  digitalWrite(A_IN1, HIGH);
  digitalWrite(A_IN2, HIGH);
}

//-----------------------------------------------
// ch A Stop (フリー)
//-----------------------------------------------
void chA_stop(){
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, LOW);
}

//=====================================================================
// I2C　制御関数
//=====================================================================
//-----------------------------------------------
// I2C スレーブデバイスに1バイト書き込む
//-----------------------------------------------
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}

//-----------------------------------------------
// I2C スレーブデバイスから1バイト読み込む
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
    digitalWrite( D13_LED, HIGH );              // ビジーの場合LED点灯
}

//-----------------------------------------------
// called when the module receives a complete response or "system_boot" event
void onIdle() {
    // turn LED off when we're no longer busy
    digitalWrite( D13_LED, LOW );
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
#if 0
        Serial.print(", offset: "); Serial.print((uint16_t)msg -> offset, HEX);
        Serial.print(", value_len: "); Serial.print(msg -> value.len, HEX);
        Serial.print(", value_data: "); Serial.print(rcv_data);
#endif
        Serial.println(F(" }"));
#endif

    if( rcv_data.indexOf("SND") == 0 ){
        bBLEsendData = true;
    } else if( rcv_data.indexOf("STP") == 0 ){
        bBLEsendData = false;
    } else if(rcv_data.indexOf("PLS") == 0){
      receiveData = "PLS";
    } else if(rcv_data.indexOf("MNS") == 0){
      receiveData = "MNS";
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
#if 0
        Serial.print(", address_type: "); Serial.print(msg -> address_type, HEX);
        Serial.print(", master: "); Serial.print(msg -> master, HEX);
        Serial.print(", connection: "); Serial.print(msg -> connection, HEX);
        Serial.print(", bonding: "); Serial.print(msg -> bonding, HEX);
        Serial.print(", advertiser: "); Serial.print(msg -> advertiser, HEX);
#endif
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
#if 0
    #ifdef DEBUG
        Serial.print( "###\tsystem_boot: { " );
        Serial.print( "major: " ); Serial.print(msg -> major, HEX);
        Serial.print( ", minor: " ); Serial.print(msg -> minor, HEX);
        Serial.print( ", patch: " ); Serial.print(msg -> patch, HEX);
        Serial.print( ", build: " ); Serial.print(msg -> build, HEX);
    //    SerialUSB.print(", ll_version: "); Serial.print(msg -> ll_version, HEX);
        Serial.print( ", bootloader_version: " ); Serial.print( msg -> bootloader, HEX );           /*  */
    //    Serial.print(", protocol_version: "); Serial.print(msg -> protocol_version, HEX);
        Serial.print( ", hw: " ); Serial.print( msg -> hw, HEX );
        Serial.println( " }" );
    #endif
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
#ifdef SERIAL_MONITOR
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
