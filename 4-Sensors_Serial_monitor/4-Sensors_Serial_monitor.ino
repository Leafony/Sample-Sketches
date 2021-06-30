//=====================================================================
//  Leafony Platform sample sketch
//     Application  : 4-Sensors with Serial monitor
//     Processor    : ATmega328P (3.3V /8MHz)
//
//     Leaf configuration
//       (1) AI01 4-Sensors
//       (2) AP01 AVR MCU
//       (3) AZ01 USB
//
//		(c)2021 LEAFONY SYSTEMS Co., Ltd
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01 First release
//      
//=====================================================================
// use libraries
//  Adafruit Unified Sensor Driver
//    https://github.com/adafruit/Adafruit_Sensor
//  Adafruit Bus IO Library
//    https://github.com/adafruit/Adafruit_BusIO
//  Adafruit LIS3DH
//    https://github.com/adafruit/Adafruit_LIS3DH
//  SmartEverything ST HTS221 Humidity Sensor
//    https://github.com/ameltech/sme-hts221-library
//  ClosedCube Arduino Library for ClosedCube OPT3001
//    https://github.com/closedcube/ClosedCube_OPT3001_Arduino
//=====================================================================

//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <Wire.h>                           // I2C

#include <Adafruit_LIS3DH.h>                // 3-axis accelerometer
#include <HTS221.h>                         // humidity and temperature sensor
#include <ClosedCube_OPT3001.h>             // Ambient Light Sensor

//===============================================
// シリアルモニタへの出力
//      #define SERIAL_MONITOR = 出力あり
//    //#define SERIAL_MONITOR = 出力なし（コメントアウトする）
//===============================================
#define SERIAL_MONITOR

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
// プログラム内で使用する定数定義
//-----------------------------------------------
//------------------------------
// I2Cアドレス
//------------------------------
#define LIS2DH_ADDRESS          0x19        // Accelerometer (SD0/SA0 pin = VCC)
#define OPT3001_ADDRESS         0x45        // Ambient Light Sensor (ADDR pin = VCC)

//---------------------------------------------------------------------
// object
//---------------------------------------------------------------------
//-----------------------------------------------
// Sensor
//-----------------------------------------------
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;

//---------------------------------------------------------------------
// プログラムで使用する変数定義
//---------------------------------------------------------------------
//------------------------------
// LIS2DH : accelerometer
//------------------------------
float dataX_g, dataY_g, dataZ_g;
float dataTilt;

//------------------------------
// HTS221 : Temperature/Humidity
//------------------------------
float dataTemp;
float dataHumid;

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
// OPT3001 : Ambient Light Sensor
//------------------------------
float dataLight;

//====================================================================
// setup
//====================================================================
void setup() {
  Wire.begin();               	// I2C 100kHz
#ifdef SERIAL_MONITOR
  	Serial.begin(115200);       // UART 115200bps
    Serial.println(F("========================================="));
    Serial.println(F("setup start"));
#endif
  setupPort();
  delay(10);

  setupSensor();
#ifdef SERIAL_MONITOR
    Serial.println(F("setup end"));
#endif
}

//-----------------------------------------------
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//-----------------------------------------------
void setupPort(){
}

//---------------------------------------------------------------------
// 各デバイスの初期設定
//---------------------------------------------------------------------
//------------------------------
// sensor
//------------------------------
void setupSensor(){
  //-------------------------------------
  // LIS2DH (accelerometer)
  //-------------------------------------
  accel.begin(LIS2DH_ADDRESS);                      // I2C address

  accel.setClick(0, 0);                             // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);                 // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_10_HZ);         // Data rate = 10Hz
  
  //-------------------------------------
  // HTS221 (humidity and temperature sensor)
  //-------------------------------------
  smeHumidity.begin(); 

  //-------------------------------------
  // OPT3001 (Ambient Light Sensor)
  //-------------------------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  light.begin(OPT3001_ADDRESS);                   // I2C address

  newConfig.RangeNumber = B1100;                  // automatic full scale
  newConfig.ConvertionTime = B1;                  // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;      // continous conversion
  newConfig.Latch = B0;                           // hysteresis-style
  
  errorConfig = light.writeConfig(newConfig);
  
  if(errorConfig != NO_ERROR){
    errorConfig = light.writeConfig(newConfig);   // retry
  }
}

//====================================================================
// Main loop
//====================================================================
void loop(){
    //-------------------------
    // LIS2DH
    // 3軸センサのデータ取得
    //-------------------------
    accel.read();
    dataX_g = accel.x_g;    //X軸
    dataY_g = accel.y_g;    //Y軸
    dataZ_g = accel.z_g;    //Z軸

    if(dataZ_g >= 1.0){
      dataZ_g = 1.00;
    } else if (dataZ_g <= -1.0){
      dataZ_g = -1.00;
    }

    dataTilt = acos(dataZ_g)/PI*180;
    
    //-------------------------
    // HTS221
    // 温湿度センサーデータ取得
    //-------------------------
    dataTemp = (float)smeHumidity.readTemperature();        //温度
    dataHumid = (float)smeHumidity.readHumidity();          //湿度

    //-------------------------
    // 温度と湿度の2点補正
    //-------------------------
    dataTemp=TM0+(TM1-TM0)*(dataTemp-TL0)/(TL1-TL0);      // 温度補正
    dataHumid=HM0+(HM1-HM0)*(dataHumid-HL0)/(HL1-HL0);    // 湿度補正

    //-------------------------
    // OPT3001
    // 照度センサーデータ取得
    //-------------------------
    OPT3001 result = light.readResult();

    if(result.error == NO_ERROR){
      dataLight = result.lux;
    }
    //-------------------------
    // シリアルモニタ表示
    //-------------------------
#ifdef SERIAL_MONITOR
    Serial.println("--- sensor data ---");    
    Serial.println("  Tmp[degC]     = " + String(dataTemp));
    Serial.println("  Hum[%]        = " + String(dataHumid));
    Serial.println("  Lum[lx]       = " + String(dataLight));
    Serial.println("  Ang[arc deg]  = " + String(dataTilt));
#endif

    delay(3000);
}


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
