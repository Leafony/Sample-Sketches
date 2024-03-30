# Google スプレッドシートでセンサデータの可視化
## はじめに
Basic Kit2とWi-Fi Maryを使って、環境センサが出来上がります。この環境センサをインターネットに繋げば、Googleスプレッドシートでのセンサデータの可視化が簡単に出来ます。

## 用意するもの
* Basic Kit2
* AC06 Wi-Fi Mary
* AV04 2V～4.5V
* AX04 Spacer
* AZ67 AAA battery holder
* AZ63 Nut Plate
* 単4ニッケル⽔素電池x3本  
* Wi-Fiルータ

## Google Apps Scriptの設定
スプレッドシートを開き`拡張機能`→`Apps Script`をクリックし、以下のソースコードを貼り付けます。</br>
`spreadsheetId`と`sheetName`は、以下を参照し入力します。</br>
* `spreadsheetId`: [こちら](https://developers.google.com/sheets/api/guides/concepts?hl=ja)を参照し入力します。</br>
* `sheetName` :スプレッドシートのシート名を入力します。デフォルトは、`シート1`になっています。</br>
```c++
function doGet(e) {

    let id = '1_F0sVNRqXUmifNbBHzWQUEvTeUmPkJeOTxO5MKtY83k'; //Insert spreadsheetId
    let sheetName = 'シート1'; //Insert sheetName
    var result;

    // e.parameter has received GET parameters, i.e. temperature, humidity, Illumination, Battery
    if (e.parameter == undefined) {
        result = 'Parameter undefined';
    } else {
        var sheet = SpreadsheetApp.openById(id).getSheetByName(sheetName);
        var newRow = sheet.getLastRow() + 1;  // get row number to be inserted
        var rowData = [];

        // get current time
        
        rowData[0] = e.parameter.UniqueID;
        rowData[1] = new Date();
        rowData[2] = e.parameter.temperature;
        rowData[3] = e.parameter.humidity;
        rowData[4] = e.parameter.illumination;
        rowData[5] = e.parameter.Battery;         

        // 1 x rowData.length cells from (newRow, 1) cell are specified
        var newRange = sheet.getRange(newRow, 1, 1, rowData.length);

        // insert data to the target cells
        newRange.setValues([rowData]);
        result =  'Ok';
    }

    return ContentService.createTextOutput(result);
}

```
詳しくは、以下を参照してください。</br>
`https://docs.leafony.com/docs/examples/advanced/2_p/esp32/esp32_googlesheets_1/#google-apps-script%E3%81%AE%E8%A8%AD%E5%AE%9A`

## Google Apps Scriptのデプロイ
下記に従って、Google Apps Scriptのデプロイをします。
https://docs.leafony.com/docs/examples/advanced/2_p/esp32/esp32_googlesheets_1/#google-apps-script%E3%81%AE%E3%83%87%E3%83%97%E3%83%AD%E3%82%A4

## Leafonyの組み立て
4-Sensors、USB、STM32MCU、AX04 Spacer、Wi-Fi Mary、2V～4.5Vリーフの順番で組み立てます。</br>
**Wi-Fi Maryに実装されているATWINC 1500のモジュールが、上段のリーフと干渉するのでスペーサリーフを入れて組み立ててください。**

## ソースコードの書き込み
`arduino_secrets.h` に、Wi-FiルータのSSIDとパスワード、Google Apps ScriptのデプロイIDを入力して、書き込みます。

## 実行結果
Google スプレッドシートにセンサデータが書き込まれたことが確認できます。
