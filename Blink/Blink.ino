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
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
