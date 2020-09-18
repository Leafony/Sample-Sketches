function doGet(e) {

    let id = 'google_sheets_id'; 
    let sheetName = 'sheet_name'; 
    var result;

    // e.parameter has received GET parameters, i.e. temperature, humidity, illumination
    if (e.parameter == undefined) {
        result = 'Parameter undefined';
    } else {
        var sheet = SpreadsheetApp.openById(id).getSheetByName(sheetName);
        var newRow = sheet.getLastRow() + 1;  // get row number to be inserted
        var rowData = [];

        // get current time
        rowData[0] = new Date();
        rowData[1] = e.parameter.UniqueID;
        rowData[2] = e.parameter.temperature;
        rowData[3] = e.parameter.humidity;
        rowData[4] = e.parameter.illumination;

        // 1 x rowData.length cells from (newRow, 1) cell are specified
        var newRange = sheet.getRange(newRow, 1, 1, rowData.length);

        // insert data to the target cells
        newRange.setValues([rowData]);
        result =  'Ok';
    }

    return ContentService.createTextOutput(result);
}
