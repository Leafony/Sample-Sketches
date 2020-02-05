//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : MIC&VR&LED
//     Processor    : ATmega328P (3.3V /8MHz)
//     Application  : LED MIC demo
//
//     Leaf configuration
//       (1) AI03 MIC&VR&LED
//       (2) AP01 AVR MCU
//       (3) AZ01 USB
//
//		(c) 2019  Trillion-Node Study Group
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2019/08/01 First release
//=====================================================================
// LED4-6:VRのレベル表示
// LED1-3:MICの音圧レベル表示
//=====================================================================
//use libraries
//
//=====================================================================

//=====================================================================
// difinition
//=====================================================================
#include <MsTimer2.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#include <Wire.h>

//=====================================================================
// IOピンの名前定義
// 接続するリーフに合わせて定義する
//=====================================================================
// --------------------------------------------
// PD port
//     digital 0: PD0 = PCRX    (HW UART)
//     digital 1: PD1 = PCTX    (HW UART)
//     digital 2: PD2 = PIR INT0
//     digital 3: PD3 = MIC&VR&LED Leaf LED1
//     digital 4: PD4 = MIC&VR&LED Leaf LED2
//     digital 5: PD5 = MIC&VR&LED Leaf LED3 SP
//     digital 6: PD6 = MIC&VR&LED Leaf LED4
//     digital 7: PD7 = MIC&VR&LED Leaf LED5
// --------------------------------------------
#define PCTX              0
#define PCRX              1
#define INT0              2
#define INT1              3
#define D4                4       
#define SP                5
#define LED1              6
#define LED2              7

// --------------------------------------------
// PB port
//     digital 8: PB0 = MIC&VR&LED Leaf LED6
//     digital 9: PB1 = D9 /* not use */
//     digital 10:PB2 = SS#
//     digital 11:PB3 = MOSI
//     digital 12:PB4 = MISO
//     digital 13:PB5 = SCK (LED)
//                PB6 = XTAL1
//                PB7 = XTAL2
//---------------------------------------------
#define LED3             8
#define LED4             9
#define LED5            10
#define LED6            11
#define MISO            12
#define AVR_LED         13

// --------------------------------------------
// PC port
//     digital 14/ Analog0: PC0 = D14
//     digital 15/ Analog1: PC1 = D15
//     digital 16/ Analog2: PC2 = MIC
//     digital 17/ Analog3: PC3 = VR
//     digital 18/ SDA    : PC4 = SDA   (I2C)
//     digital 19/ SCL    : PC5 = SCL   (I2C)
//     RESET              : PC6 = RESET#
//-----------------------------------------------
#define D14         14
#define D15         15
#define MIC         16
#define VR          17
#define SDA         18
#define SCL         19

//=====================================================================
// プログラム内で使用する定数定義
// 
//=====================================================================
//-----------------------------------------------
//３軸センサ、輝度センサ I2Cアドレス
//-----------------------------------------------
#define I2C_ADC_ADDR_BAT 0x50

#define I2C_SEND_BUF_LENGTH 10
unsigned char i2c_sendBuf[I2C_SEND_BUF_LENGTH];

#define I2C_RECEIVE_BUF_LENGTH 10
unsigned char i2c_receiveBuf[I2C_RECEIVE_BUF_LENGTH];

//-----------------------------------------------
// loop() interval
// MsTimer2のタイマー割り込み発生間隔(ms)
//-----------------------------------------------
#define LOOP_INTERVAL 100         // interval

//=====================================================================
// object
//=====================================================================
    
//=====================================================================
// プログラムで使用する変数定義
// 
//=====================================================================

unsigned int dataVR;
unsigned int dataMic;
float dataBatt = 0;

volatile bool bInterval = false;

//=====================================================================
// setup
//=====================================================================
//-----------------------------------------------
// port
//-----------------------------------------------
//=====================================================================
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//=====================================================================
void setupPort(){

  //---------------------
  // PD port
  //---------------------
  // PD0 : digital 0 = RX
  // PD1 : digital 1 = TX

  pinMode(INT0, INPUT);        // PD2 : digital 2 = INT0#
  pinMode(INT1, INPUT);        // PD3 : digital 3 = INT1#
  pinMode(D4, INPUT);          // PD4 : digital 4 = not used
  pinMode(SP, OUTPUT);         // PD5 : digital 5 = SP not used
  digitalWrite(SP, LOW);
  pinMode(LED1, OUTPUT);       // PD6 : digital 6 = LED1
  digitalWrite(LED1, LOW);
  pinMode(LED2, OUTPUT);       // PD7 : digital 7 = LED2
  digitalWrite(LED2, LOW);

  //---------------------
  // PB port
  //---------------------
  pinMode(LED3, OUTPUT);        // PB0 : digital 8 = LED3
  digitalWrite(LED3, LOW);
  pinMode(LED4, OUTPUT);        // PB1 : digital 9 = LED4
  digitalWrite(LED4, LOW);
  pinMode(LED5, OUTPUT);        // PB2 : digital 10 = LED5
  digitalWrite(LED5, LOW);
  pinMode(LED6, OUTPUT);        // PB3 : digital 11 = LED6
  digitalWrite(LED6, LOW);
  pinMode(MISO, INPUT);         // PB4 : digital 12 = not used
  pinMode(AVR_LED, OUTPUT);     // PB5 : digital 13 =LED on 8bit-Dev. Leaf
  digitalWrite(AVR_LED, LOW);

  //---------------------
  // PC port
  //---------------------
  pinMode(D14, INPUT);         // PC0 : digital 14 = not used
  pinMode(D15, INPUT);         // PC1 : digital 14 = not used

  pinMode(MIC, INPUT);         // PC2 : digital 14 = not used
  pinMode(VR, INPUT);          // PC3 : digital 14 = not used

  // PC4 : digital 18 = I2C SDA
  // PC5 : digital 19 = I2C SCL 
}
//=====================================================================
// 割り込み処理初期設定
// 
//=====================================================================
//-----------------------------------------------
// external interrupt
// 外部割り込み設定
//-----------------------------------------------
void setupExtInt(){

  detachInterrupt(0);                         // INT0# = disabled
  detachInterrupt(1);                         // INT1# = disabled
}

//-----------------------------------------------
// timer2 interrupt (interval=125ms, int=overflow)
// メインループのタイマー割り込み設定
//-----------------------------------------------
void setupTC2Int(){
  MsTimer2::set(LOOP_INTERVAL, intTimer2);
}

//----------------------------------------------
// INT0
// INT0割り込み関数
//----------------------------------------------
void intExtInt0(){
   Serial.println("int");
}
//----------------------------------------------
// Timer2 INT
// タイマー割り込み関数
//----------------------------------------------
void intTimer2(){
  bInterval = true;
}
//=====================================================================
// I2C　制御関数
// 
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
//-----------------------------------------------
// I2C スレーブデバイスに複数バイト書き込む
//-----------------------------------------------
void i2c_write(int device_address, int reg_address, int lengrh, unsigned char* write_byte){

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  for (int i = 0; i < lengrh; i++){
    Wire.write(write_byte[i]);
  }
  Wire.endTransmission();
}
//-----------------------------------------------
//I2C スレーブデバイスから複数バイト読み込む
//-----------------------------------------------
void i2c_read(int device_address, int reg_address, int lengrh, unsigned char* read_byte){

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, lengrh);
  for (int i = 0; i < lengrh; i++){
    read_byte[i] = Wire.read();
  }
}
//-----------------------------------------------
// I2C 受信バッファクリア
//-----------------------------------------------
void clearI2CReadbuf(){
  memcpy(i2c_receiveBuf, 0x00, I2C_RECEIVE_BUF_LENGTH);
}
//-----------------------------------------------
// VRとMIC、電池のデータを取得する
//-----------------------------------------------
void getSencerData()
{
    dataVR = analogRead(A3);    
    dataMic = analogRead(A2);

    //-------------------------
    // ADC081C027（ADC)
    // 電池リーフ電池電圧取得
    //-------------------------
    uint8_t adcVal1 = 0;
    uint8_t adcVal2 = 0;
  
    Wire.beginTransmission(I2C_ADC_ADDR_BAT);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_ADC_ADDR_BAT,2);
    adcVal1 = Wire.read();
    adcVal2 = Wire.read();
  
    if (adcVal1 == 0xff && adcVal2 == 0xff) { 
      //測定値がFFならバッテリリーフはつながっていない
      adcVal1 = adcVal2 = 0;
    }
  
    //電圧計算　ADC　* （(リファレンス電圧(3.3V)/ ADCの分解能(256)) * 分圧比（２倍））
    double temp_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
    dataBatt = (float)(temp_mv / 1000);

    Serial.println("--------------------------------------------------");
    Serial.print("MIC  = ");
    Serial.println(dataMic,DEC);
    Serial.print("VR   = ");
    Serial.println(dataVR,DEC);
    Serial.print("VBAT = ");
    Serial.println(dataBatt,DEC);  
}
//-----------------------------------------------
// VRとMICの値にあわせてLEDを点灯、消灯させる
//-----------------------------------------------
void viewSencerData()
{    
    if (dataVR < 400)
    {
      digitalWrite(LED4,LOW);
      digitalWrite(LED5,LOW);
      digitalWrite(LED6,LOW);    
    }
    else if (dataVR >= 400 && dataVR < 800)
    {
       digitalWrite(LED4,LOW);
       digitalWrite(LED5,LOW); 
       digitalWrite(LED6,HIGH); 
    }
    else if (dataVR >= 800 && dataVR < 900)
    {
       digitalWrite(LED4,LOW);
       digitalWrite(LED5,HIGH); 
       digitalWrite(LED6,HIGH);     
    }
    else
    {
       digitalWrite(LED4,HIGH);
       digitalWrite(LED5,HIGH); 
       digitalWrite(LED6,HIGH);     
    }
  
    if (dataMic < 300)
    {
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);   
      digitalWrite(LED3,LOW);     
    }
    else if (dataMic >= 300 && dataMic < 350)
    {
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);   
      digitalWrite(LED3,HIGH);     
    }
    else if (dataMic >= 350 && dataMic < 450)
    {
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,HIGH);   
      digitalWrite(LED3,HIGH);     
    }
    else
    {
      digitalWrite(LED1,HIGH);
      digitalWrite(LED2,HIGH);   
      digitalWrite(LED3,HIGH);           
    }
}

//-----------------------------------------------
// setup
//-----------------------------------------------
void setup() {

 //WDT disable
  wdt_disable();

  //内部の各モジュールの電源OFF
  power_all_disable();
  power_timer0_enable();
  power_timer2_enable();
  power_twi_enable();
  power_usart0_enable();
  power_adc_enable();

  delay(10);
  
  Serial.begin(115200);
  Wire.begin();
  Serial.println("start!!");
  
  //起動テスト(LED)
  digitalWrite(LED1,HIGH);
  delay(1000);
  digitalWrite(LED2,HIGH);
  delay(1000);
  digitalWrite(LED3,HIGH);
  delay(1000);
  digitalWrite(LED4,HIGH);
  delay(1000);
  digitalWrite(LED5,HIGH);
  delay(1000);
  digitalWrite(LED6,HIGH);
  delay(1000);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);
  digitalWrite(LED4,LOW);
  digitalWrite(LED5,LOW);
  digitalWrite(LED6,LOW);
  //
  delay(2000);
  setupExtInt();
  setupTC2Int();
  MsTimer2::start();

}
//-----------------------------------------------
// Main
//-----------------------------------------------
void loop() {
  if (bInterval == true)
  {
    bInterval = false;
    getSencerData();
    viewSencerData();
  }  
}
