# Wi-Fiモジュールのファームウェアのバージョン確認
## はじめに
Basic Kit2とWi-Fi Maryを使って、Wi-Fiモジュール(ATWINC1500)のファームウェアのバージョン確認をします。

## 用意するもの
* Basic Kit2
* AC06 Wi-Fi Mary
* AX04 Spacer
* AZ63 Nut Plate

## Leafonyの組み立て
USB、STM32MCU、AX04 Spacer、Wi-Fi Maryの順番で組み立てます。</br>
**Wi-Fi Maryに実装されているATWINC 1500のモジュールが、上段のリーフと干渉するのでスペーサリーフを入れて組み立ててください。**</br>

## ソースコードの書き込み
ソースコードを書き込みます。</br>

## 実行結果
正しく動作していれば、シリアルモニターに以下の表示がされます。ファームウェアのバージョンが、19.6.1であることを確認してください。</br>

```c++
WiFi101 firmware check.

WiFi101 shield: DETECTED
Firmware version installed: 19.6.1
Latest firmware version available : 19.6.1

Check result: PASSED
```
