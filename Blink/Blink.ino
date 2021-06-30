//=====================================================================
//  Blink
//
//    (c) 2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01  First release
//=====================================================================

void setup() {
  // LEDピンを出力ピンに設定
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // LEDを点灯
  delay(1000);                       // 1秒待つ
  digitalWrite(LED_BUILTIN, LOW);    // LEDを消灯
  delay(1000);                       // 1秒待つ
}
