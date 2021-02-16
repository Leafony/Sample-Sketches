//=====================================================================
//  Leafony Platform sample sketch
//     Application  : LCD SW test
//     Processor    : STM32L452RE (Nucleo-64/Nucleo L452RE)
//
//     Leaf configuration
//       (1) AI04 LCD
//       (2) AP03 STM32 MCU
//       (3) AZ01 USB
//
//		(c) 2020  Trillion-Node Study Group
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2021/01/26  First release
//=====================================================================
// use libraries
//  ST7032 - Arduino LiquidCrystal compatible library
//    https://github.com/tomozh/arduino_ST7032
//=====================================================================

//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <Wire.h>                   // I2C
#include <ST7032.h>                 // LCD

//-----------------------------------------------
// IOピンの名前定義
// 接続するリーフに合わせて定義する
//-----------------------------------------------
#define SW1         PC7             // D2  (INT0)

//-----------------------------------------------
// プログラム内で使用する定数定義
//-----------------------------------------------
#define I2C_EXPANDER_ADDR   0x1A

//---------------------------------------------------------------------
// object
//---------------------------------------------------------------------
//------------------------------
// LCD
//------------------------------
ST7032 lcd;

//====================================================================
// setup
//====================================================================
void setup(){
  pinMode(SW1, INPUT);                                //LCD SW1
  Wire.begin();

  // IO　Expander Initialize
  i2c_write_byte(I2C_EXPANDER_ADDR, 0x03, 0xFE);
  i2c_write_byte(I2C_EXPANDER_ADDR, 0x01, 0x01);      // LCD Power on
  
  //LCD Initialize
  lcd.begin(8, 2);                                    // 8文字, 2行
  lcd.setContrast(30);
  lcd.clear();

  lcd.print(" Hello!");
  lcd.setCursor(0, 1);
  delay(3000);

  i2c_write_byte(I2C_EXPANDER_ADDR, 0x01, 0x00);      // LCD Power off
  delay(3000);

  i2c_write_byte(I2C_EXPANDER_ADDR, 0x01, 0x01);      // LCD Power on

  // LCD Initialize
  lcd.begin(8, 2);                                    // 8文字, 2行
  lcd.setContrast(30);
  lcd.clear();

  lcd.print("12345678");
  lcd.setCursor(0, 1);
  lcd.print("87654321");
  delay(3000);

  lcd.clear();

  lcd.blink();
  int i;
  for (i=0 ; i<8 ;i++){
     lcd.setCursor(i, 0);
      delay(1000);
  }
  for (i=0 ; i<8 ;i++){
     lcd.setCursor(i, 1);
      delay(1000);
  }
  lcd.noBlink();
}

//====================================================================
// Main loop
//====================================================================
void loop(){
  char val;

  // SW 1
  val = digitalRead(SW1);
  lcd.setCursor(0, 0);
  if (val == 1) {
     lcd.print("SW1 is H");
  }else{
     lcd.print("SW1 is L");
  }
	
  // SW 2
  val = i2c_read_byte(I2C_EXPANDER_ADDR, 0x00); 
  lcd.setCursor(0, 1);
  if ((val & 0x02) == 0x02)  {     
     lcd.print("SW2 is H");
  }
  else {
    lcd.print("SW2 is L");
  }
  delay(1000);
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
