//=====================================================================
//  Blink
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/05/05  First release
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
