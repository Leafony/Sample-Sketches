# AV05 Solar 3.2V 検査手順
## はじめに
本内容は、AV05 Solar 3.2V の検査検査手順を記す。
## 用意するもの
### Solar 3.2V AS
* AX02 29 pin
* AP03 STM32 MCU  
* AV05 Solar 3.2V
* AZ63 Nut Plate
* M2X10mm ネジ
### Solar-charger-debugger AS
* AZ01 USB
* Solar-charger-debugger-Leaf
* AZ63 Nut Plate
* M2X8mm
### その他
* リン酸鉄リチウムイオンバッテリ
* ACアダプタ DC12V
* スイッチ
* USB/UART変換ケーブル
* 6p-SHコネクタ・ケーブル
* PC
## ソースコード
* [Leaf_Solar_Test.ino](https://github.com/Leafony/Sample-Sketches/blob/master/Leaf_Solar_Test/Leaf_Solar_Test.ino)
## 組立て

## 検査方法
1.Arduino IDEでシリアルモニタを立ち上げ、テキストボックスに`m`を入力するとシリアルモニタに以下の内容が表示される。
 --------------------------------------------------
 m = This Menu
 1 = KTD: LED start
 2 = KTD: LED stop
 3 = TCA: Initialize
 4 = TCA: Read register
 5 = TCA: ONVCC_High
 6 = TCA: ONVCC_Low
 7 = ADC: Initialize
 8 = ADC: AD convert
 -------------------------------------------------
2.

