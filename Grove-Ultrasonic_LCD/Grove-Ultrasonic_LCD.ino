//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : Grove + Ultrasonic
//     Processor    : ATmega328P (3.3V /8MHz)
//     Application  : Ultrasonic Ranger demo
//
//     Leaf configuration
//       (1) AI04 LCD
//       (2) AP01 AVR MCU
//       (3) AX01 Grove&5V + Grove - Ultrasonic Ranger (UART pinに接続)
//           ※  Ultrasonic RangerはGrove&5VのUARTに接続する
//       (4) AZ01 USB
//
//		(c) 2021 LEAFONY SYSTEMS Co., Ltd
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01 First release
//=====================================================================
// Grove - Demonstration using Ultrasonic Ranger
// Displays the distance from the object obtained from the ultrasonic sensor on the LCD.
//=====================================================================
//use libraries
//ST7032 - Arduino LiquidCrystal compatible library
//https://github.com/tomozh/arduino_ST7032
//Grove_Ultrasonic_Ranger
//https://github.com/Seeed-Studio/Grove_Ultrasonic_Ranger/
//=====================================================================

#include <Wire.h>
#include <ST7032.h>

// LCD
ST7032 lcd;
char strMessage[8];

#include "Ultrasonic.h"
Ultrasonic ultrasonic(A1);


void setup() {
  Serial.begin(115200);

  //LCD Initialize
  lcd.begin(8, 2);
  lcd.setContrast(30);
  lcd.clear();
  lcd.print(" Hello!");
  lcd.setCursor(0, 1);
  delay(1000);
  lcd.begin(8, 2);
  lcd.setContrast(30);
  lcd.clear();
  lcd.print("12345678");
  lcd.setCursor(0, 1);
  lcd.print("87654321");
  delay(500);
  lcd.clear();
  lcd.blink();
  int i;
  for (i=0 ; i<8 ;i++)
  {
     lcd.setCursor(i, 0);
      delay(100);
  }
  for (i=0 ; i<8 ;i++)
  {
     lcd.setCursor(i, 1);
      delay(100);
  }
  lcd.noBlink();

}



void loop() {
  long RangeInCentimeters;

  lcd.clear();
  lcd.print("Distance");
  RangeInCentimeters = ultrasonic.MeasureInCentimeters();
  sprintf(strMessage,"%5d cm",RangeInCentimeters);
  lcd.setCursor(0, 1);
  lcd.print(strMessage);

  delay(250);
}
