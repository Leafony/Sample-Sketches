//=====================================================================
// Leafony Platform sample sketch
//     Platform     : PIR&SP
//     Processor    : ATmega328P (3.3V /8MHz)
//     Application  : PIR with SP Beep
//
//     Leaf configuration
//       (1) AI02 SP&PIR
//       (2) AP01 AVR MCU
//       (3) AZ01 USB
//
//		(c) 2020  Trillion-Node Study Group
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2019/08/01  First release
//=====================================================================
//=====================================================================
// difinition
//=====================================================================
#include <stdio.h>
#include <Arduino.h>
#include <Wire.h>

#define I2C_PIR_ADDR   0x65
#define I2C_SEND_BUF_LENGTH 10
unsigned char i2c_sendBuf[I2C_SEND_BUF_LENGTH];

#define I2C_RECEIVE_BUF_LENGTH 10
unsigned char i2c_receiveBuf[I2C_RECEIVE_BUF_LENGTH];
unsigned char i2c_receiveLenght;

byte readReg;
double irData;
double tempData;
char buf[120];

volatile int state = 0;

//=====================================================================
// setup
//=====================================================================
void setup() {
  //pinMode(2, INPUT);
  attachInterrupt(0,catchHuman , FALLING );           //人接近検知割り込み

  Wire.begin();
  Serial.begin( 115200 );
  delay(100);

  //人感センサ設定
  i2c_write_byte(I2C_PIR_ADDR, 0x20, 0xFF); //CNTL1  Resrt 
  i2c_write_byte(I2C_PIR_ADDR, 0x2A, 0xF2); //CNTL11 人感アルゴリズム有効/割り込み出力有効
  i2c_write_byte(I2C_PIR_ADDR, 0x25, 0x0F); //CNTL6  センサゲイン205%(最大)
  i2c_write_byte(I2C_PIR_ADDR, 0x2B, 0xFF); //CNTL12 Mode=1 start Meas(連続測定モード)
   delay(1000);
}
//=====================================================================
// Main loop
//=====================================================================
void loop() {
  clearI2CReadbuf();
  i2c_read(I2C_PIR_ADDR, 0x04, 6, i2c_receiveBuf);
  sprintf(buf, "REG = %02X , %02X , %02X , %02X , %02X , %02X", i2c_receiveBuf[0], i2c_receiveBuf[1], i2c_receiveBuf[2], i2c_receiveBuf[3], i2c_receiveBuf[4], i2c_receiveBuf[5]);
  Serial.println(buf);
  sprintf(buf, "Human detection = %d", (i2c_receiveBuf[0] & 0x10) >> 4  );
  Serial.println(buf);

  //IRセンサ測定データ
  irData = clacIR();
  Serial.print("IR   = ");
  Serial.print(irData,2);
  Serial.println(" pA");

  //センサ温度
  tempData = clacTemp();
  Serial.print("TSENS = ");
  Serial.print(tempData,2);
  Serial.println(" deg");
  Serial.println("===================================");
  if (state == 1){
    tone(5, 262, 300);
    state = 0;
  }
  delay(1000);
}

//=====================================================================
void catchHuman()
{
  state = 1;
  Serial.println("!! Interrupt !!");      //人の接近を検知
}

//=====================================================================
double clacTemp()
{
  double ret;
  unsigned short val = (unsigned short)((i2c_receiveBuf[4] << 8) |  i2c_receiveBuf[3]);
  if ( (val & 0x8000) == 0x8000)
  {
     val = ~val + 1;
     ret = (double)((val) *   0.0019837 ) * -1;
  }
  else
  {
    ret = (double)val * 0.0019837;
  }
  return ret + 25;
}

//=====================================================================
double clacIR()
{
  double ret;
  unsigned short val = (unsigned short)((i2c_receiveBuf[2] << 8) |  i2c_receiveBuf[1]);
  if ( (val & 0x8000) == 0x8000)
  {
    val = ~val + 1;
    ret = (double)(val *   0.4578 ) * -1;
  }
  else
  {
    ret = (double)(val *  0.4578 );
  }
  return ret;
}
/**********************************************
* I2C スレーブデバイスに1バイト書き込む
**********************************************/
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}
/**********************************************
* I2C スレーブデバイスから1バイト読み込む
**********************************************/
unsigned char i2c_read_byte(int device_address, int reg_address){

  int read_data = 0;

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, 1);
  read_data = Wire.read();

  return read_data;
}
/**********************************************
* I2C スレーブデバイスに複数バイト書き込む
**********************************************/
void i2c_write(int device_address, int reg_address, int lengrh, unsigned char* write_byte){

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  for (int i = 0; i < lengrh; i++){
    Wire.write(write_byte[i]);
  }
  Wire.endTransmission();
}
/**********************************************
* I2C スレーブデバイスから複数バイト読み込む
**********************************************/
void i2c_read(int device_address, int reg_address, int lengrh, unsigned char* read_byte){

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, lengrh);
  for (int i = 0; i < lengrh; i++){
    read_byte[i] = Wire.read();
  }
}
/**********************************************
* I2C 受信バッファクリア
**********************************************/
void clearI2CReadbuf(){
  memset(&i2c_receiveBuf[0], 0x00, sizeof(i2c_receiveBuf));
}
