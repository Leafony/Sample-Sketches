//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : 4-Sensors
//     Processor    : ATmega328P (3.3V /8MHz)
//     Application  : 4-Sensors with Serial monitor
//
//     Leaf configuration
//       (1) AI01 4-Sensors
//       (2) AP01 AVR MCU
//       (3) AZ01 USB
//
//		(c) 2019  Trillion-Node Study Group
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2019/08/20 First release
//=====================================================================
//use libraries
//Adafruit LIS3DH
//https://github.com/adafruit/Adafruit_LIS3DH
//※  Adafruit_LIS3DH.h
//    uint8_t readRegister8(uint8_t reg);
//    void writeRegister8(uint8_t reg, uint8_t value);
//    をpublic:に移動する
//Adafruit Unified Sensor Driver
//https://github.com/adafruit/Adafruit_Sensor
//SmartEverything ST HTS221 Humidity Sensor
//https://github.com/ameltech/sme-hts221-library
//ClosedCube Arduino Library for ClosedCube OPT3001
//https://github.com/closedcube/ClosedCube_OPT3001_Arduino
//=====================================================================

//=====================================================================
// difinition
//=====================================================================
#include <Wire.h>
#include <Adafruit_LIS3DH.h>    
#include <Adafruit_Sensor.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>
#include <SoftwareSerial.h>
//=====================================================================

//=====================================================================
// IOピンの名前定義
// 接続するリーフに合わせて定義する
//=====================================================================
#define SDA       18
#define SCL       19

//=====================================================================
// プログラム内で使用する定数定義
// 
//=====================================================================
//-----------------------------------------------
//３軸センサ、輝度センサ I2Cアドレス
//-----------------------------------------------
#define LIS2DH_ADDRESS 0x19       // SD0/SA0 pin = VCC 
#define OPT3001_ADDRESS 0x45      // ADDR pin = VCC

//-----------------------------------------------
// LIS2DH
//-----------------------------------------------
#define DIVIDER_2G 16383          // full scale 2G  (=0xFFFF/4)
#define DIVIDER_4G 8191           // full scale 4G  (=0xFFFF/4/2)
#define DIVIDER_8G 4096           // full scale 8G  (=0xFFFF/4/4)
#define DIVIDER_16G 1365          // full scale 16G (=0xFFFF/4/12)

//=====================================================================
// object
//=====================================================================
//-----------------------------------------------
// Sensor
//-----------------------------------------------
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;      

//=====================================================================
// プログラムで使用する変数定義
// 
//=====================================================================
//=====================================================================
// RAM data
//=====================================================================
//---------------------------
// LIS2DH : accelerometer
//---------------------------
int16_t dataX, dataY, dataZ;
float dataX_g, dataY_g, dataZ_g;
float dataTilt, avrTilt;

//---------------------------
// HTS221 : Temperature/Humidity
//---------------------------
float dataTemp, avrTemp;
float dataHumid, avrHumid;
float calcTemp = 0;
float calcHumid = 0;

//---------------------------
// 2点補正用データ
//---------------------------
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

//---------------------------
// OPT3001 : Light
//---------------------------
float dataLight, avrLight;
float calcLight = 0;

//====================================================================
// setup
//====================================================================
void setup() {
  Serial.begin(115200);       // UART 115200bps
  Wire.begin();               // I2C 100KHz

  delay(10);

  setupSensor();
}

//====================================================================
// Main loop
//====================================================================
void loop() {
    //-------------------------
    // LIS2DH
    // 3軸センサーのデータ取得
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
    dataTemp = (float)smeHumidity.readTemperature();  //温度
    dataHumid = (float)smeHumidity.readHumidity();    //湿度

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
    Serial.println("--- sensor data ---");    
    Serial.println("  Tmp[degC]     = " + String(dataTemp));
    Serial.println("  Hum[%]        = " + String(dataHumid));
    Serial.println("  Lum[lx]       = " + String(dataLight));
    Serial.println("  Ang[arc deg]  = " + String(dataTilt));

    delay(3000);

}

//=====================================================================
// 各デバイスの初期設定
// 
//=====================================================================
//-----------------------------------------------
// sensor
//-----------------------------------------------
void setupSensor(){

  //-------------------------------------
  // LIS2DH (accelerometer)
  //-------------------------------------
  //-------------------
  // I2C address
  //------------------
  accel.begin(LIS2DH_ADDRESS);

  //-------------------
  // register
  //-------------------
  accel.writeRegister8(LIS3DH_REG_CTRL1, 0x07);    // X,Y,Z axis = enable
  accel.setDataRate(LIS3DH_DATARATE_1_HZ);         // Data rate = 1Hz
  
  accel.writeRegister8(LIS3DH_REG_CTRL2, 0x00);
  accel.writeRegister8(LIS3DH_REG_CTRL3, 0x00);    // INT Disable
  accel.writeRegister8(LIS3DH_REG_CTRL4, 0x80);    // BUD = enable, Scale = +/-2g

  //-------------------------------------
  // HTS221 (temperature /humidity)
  //-------------------------------------
  smeHumidity.begin(); 

  //-------------------------------------
  // OPT3001 (light)
  //-------------------------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  //-------------------
  // I2C address
  //-------------------
  light.begin(OPT3001_ADDRESS);

  //-------------------
  // config register
  //-------------------
  newConfig.RangeNumber = B1100;               // automatic full scale
  newConfig.ConvertionTime = B1;               // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;   // continous conversion
  newConfig.Latch = B0;                        // hysteresis-style
  
  errorConfig = light.writeConfig(newConfig);
  
  if(errorConfig != NO_ERROR){
    errorConfig = light.writeConfig(newConfig);   //retry
  }
}


//---------------------------------------
// trim
// 文字列配列からSPを削除する
//---------------------------------------
void trim(char * data)
{
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
