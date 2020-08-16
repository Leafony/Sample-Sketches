//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : ESP32
//     Processor    : ESP32-WROOM-32
//     Application  : Sending Sensor Data to Google Sheets
//
//     Leaf configuration
//       (1) AP02 ESP MCU
//       (2) AI01 4-Sensors
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/8/17  First release
//=====================================================================

// Paste following codes to Google Scripts Editor


// doGet is executed when accessed with GET
function doGet(e) {

    let id = 'google_sheets_id'; 
    let sheetName = 'sheet_name'; 
    var result;

    // e.parameter has received GET parameters, i.e. temperature, humidity, brightness
    if (e.parameter == undefined) {
        result = 'Parameter undefined';
    } else {
        var sheet = SpreadsheetApp.openById(id).getSheetByName(sheetName);
        var newRow = sheet.getLastRow() + 1;  // get row number to be inserted
        var rowData = [];

        // get current time
        rowData[0] = new Date();
        rowData[1] = e.parameter.temperature;
        rowData[2] = e.parameter.humidity;
        rowData[3] = e.parameter.brightness;

        // 1 x rowData.length cells from (newRow, 1) cell are specified
        var newRange = sheet.getRange(newRow, 1, 1, rowData.length);

        // insert data to the target cells
        newRange.setValues([rowData]);
        result =  'Ok';
    }

    return ContentService.createTextOutput(result);
}
