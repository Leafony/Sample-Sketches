//=====================================================================
//  Button
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/07/02  First release
//=====================================================================

int pushButton = 0;

void setup() {
  Serial.begin(115200);
  pinMode(pushButton, INPUT);
}

void loop() {
  int buttonState = digitalRead(pushButton);
  Serial.println(buttonState);
  delay(1);
}
