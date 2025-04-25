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
* テスター
## ソースコード
* [Leaf_Solar_Test.ino](https://github.com/Leafony/Sample-Sketches/blob/master/Leaf_Solar_Test/Leaf_Solar_Test.ino)
## 組立て

## 検査方法
1.バッテリーをSolar 3.2Vに繋げると、スイッチのLEDが点滅し、Solar-charger-debuggerのLEDが点灯することを確認する。
2.次に、Arduino IDEでシリアルモニタを立ち上げ、テキストボックスに、コマンド`m`を入力すると以下のメニュー画面が表示される。
 ```
 m = This Menu
 1 = KTD: LED start
 2 = KTD: LED stop
 3 = TCA: Initialize
 4 = TCA: Read register
 5 = TCA: ONVCC_High
 6 = TCA: ONVCC_Low
 7 = ADC: Initialize
 8 = ADC: AD convert
 ```
3.コマンド`1`と`2`を入力し、スイッチのLEDの点滅と消灯を確認する。

4.[29 pinリーフ](https://docs.leafony.com/docs/leaf/extension/ax02/#%E3%83%94%E3%83%B3%E9%85%8D%E7%BD%AE)にテスターを当てて、3.3V±0.3Vを確認する。(3V3-GND)
　
5.次に、コマンド`5`を入力し、5V電源がON（負荷が無いため実測5.2～5.3V）であることを確認する。(VBUS-GND)

6.充電のON/OFFをIOExpanderリードし、確認する
| 充電 | | ACアダプター | バッテリー | 表示|
| :--- | | :--- | :--- | :---: |
|ON | | 無し | 有り |"Addr_0x00 = f0" 表示  |
|OFF| | 有り | 有り | "Addr_0x00 = f1" 表示 |

