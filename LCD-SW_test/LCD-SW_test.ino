//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : LCD
//     Processor    : ATmega328P (3.3V /8MHz)
//     Application  : LCD SW test
//
//     Leaf configuration
//       (1) AI04 LCD
//       (2) AP01 AVR MCU
//       (3) AZ01 USB
//
//		(c) 2020  Trillion-Node Study Group
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2019/08/01  First release
//=====================================================================
//use libraries
//=====================================================================

//=====================================================================
// difinition
//=====================================================================
#include <Wire.h>
#include <ST7032.h>
//=====================================================================
// プログラム内で使用する定数定義
// 
//=====================================================================
#define I2C_EXPANDER_ADDR   0x1A

// LCD
ST7032 lcd;

char buf[10];

//====================================================================
// setup
//====================================================================
void setup() {

  pinMode(2, INPUT);                    //LCD SW1
  Wire.begin();

  // IO　Expander Initialize
  i2c_write_byte(I2C_EXPANDER_ADDR, 0x03, 0xFE);
  i2c_write_byte(I2C_EXPANDER_ADDR, 0x01, 0x01);

  //LCD Initialize
  lcd.begin(8, 2);
  lcd.setContrast(30);
  lcd.clear();

  lcd.print(" Hello!");
  lcd.setCursor(0, 1);
  delay(3000);

  // LCD Power off
  i2c_write_byte(I2C_EXPANDER_ADDR, 0x01, 0x00);
  delay(3000);

  // LCD Power on
  i2c_write_byte(I2C_EXPANDER_ADDR, 0x01, 0x01);

  //LCD Initialize
  lcd.begin(8, 2);
  lcd.setContrast(30);
  lcd.clear();

  lcd.print("12345678");
  lcd.setCursor(0, 1);
  lcd.print("87654321");
  delay(3000);

  lcd.clear();

  lcd.blink();
  int i;
  for (i=0 ; i<8 ;i++)
  {
     lcd.setCursor(i, 0);
      delay(1000);
  }
  for (i=0 ; i<8 ;i++)
  {
     lcd.setCursor(i, 1);
      delay(1000);
  }
  lcd.noBlink();

}
//====================================================================
// Main loop
//====================================================================
void loop() {
  char val;

  // SW 1
  val = digitalRead(2);
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