//=====================================================================
//  Leafony Platform sample sketch
//     Application  : UART test
//     Processor    : STM32L452RE (Nucleo-64/Nucleo L452RE)
//     Arduino IDE  : 1.8.13
//     STM32 Core   : Ver1.9.0
//
//     Leaf configuration
//       (1) AP02 ESP32 MCU (AT command)
//       (2) AP03 STM32 MCU
//       (3) AZ01 USB
//
//    (c) 2022 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2022/08/10 First release
//=====================================================================

HardwareSerial Serial_ESP(USART1);            // UART1 RX:D9(PA10) TX:D8(PA9)

//=====================================================================
// setup
//=====================================================================
void setup() {
  Serial.begin(115200);                       // UART 115200bps
    uint8_t tm=0;
    while (!Serial && tm <150){               // Serial起動待ち タイムアウト1.5s
      tm++;
      delay(10);
    }
  Serial_ESP.begin(115200);                   // UART 115200bps
    tm=0;
    while (!Serial_ESP && tm <150){           // Serial起動待ち タイムアウト1.5s
      tm++;
      delay(10);
    }
    Serial.println(F("setup start"));

//    Serial_ESP.println(F("setup start1"));
}

//====================================================================
// Loop
//=====================================================================
void loop() {
  char Read_Buff = 0;                           // 受信データ用

  if (Serial.available() > 0) {
    Read_Buff = Serial.read();                  // 受信データを読み込む
    Serial_ESP.print(Read_Buff);
  } else if (Serial_ESP.available() > 0){
    Read_Buff = Serial_ESP.read();              // 受信データを読み込む
    Serial.print(Read_Buff);
  }
}
