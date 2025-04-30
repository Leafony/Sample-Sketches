# AV06 1.8V～5.5V 検査手順
## はじめに
本内容は、AV06 1.8V～5.5V の検査検査手順を記す。
## 用意するもの
### AV06 1.8V～5.5V AS
* [2.54x6P_through_hole](https://github.com/Leafony/HW-Design-Files/tree/master/2.54x6P_through_hole)
* AP03 STM32 MCU  
* AV06 1.8V～5.5V
* AZ63 Nut Plate
* M2X10mm ネジ  
### Solar-charger-debugger AS
* [Solar-charger-debugger-Leaf](https://github.com/Leafony/HW-Design-Files/tree/master/Solar-charger-debugger-Leaf) [^1]
* AZ01 USB
* AZ63 Nut Plate
* M2X8mm  
[^1]:Solar-charger-debugger-Leafは、高さ約3mmなので必ず上段にすること
### その他
* XXXX
* バッテリー
* USBケーブル
* 6p-SHコネクタ・ケーブル
* PC
* テスター
## ソースコード
* [Leaf_SolarTestBoard_Test.ino](https://github.com/Leafony/Sample-Sketches/blob/master/Leaf_SolarTestBoard_Test/Leaf_SolarTestBoard_Test.ino)
## 組立て
<img src="./docs/Solar_3.2V_Test.jpg" width="400" />


## 検査方法
1.Solar-charger-debugger-LeafをRunモードにする

<img src="./docs/Solar-charger-debugger-Leaf_3d.png" width="400" />

2.AV06 1.8V～5.5VのスイッチをONにすると、Solar-charger-debuggerのLEDが点灯することを確認する

3.次に、Arduino IDEでシリアルモニタを立ち上げ、テキストボックスに、コマンド`m`を入力すると以下のメニュー画面が表示される
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
4.Solar-charger-debugger-Leafにテスターを当て、3.3V±0.3Vを確認する

5.コマンド`8`を入力し、A/D変換値リードさせて、ACアダプター無し、バッテリーのみで、バッテリーの電圧を確認する
