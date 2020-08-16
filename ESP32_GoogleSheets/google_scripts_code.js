function doGet(e) {

 let id = 'google_sheets_id';
 let sheetName = 'sheet_name';  
 var result;
 
 if (e.parameter == undefined) {
   result = 'Parameter undefined';
 } else {
   var sheet = SpreadsheetApp.openById(id).getSheetByName(sheetName);
   var newRow = sheet.getLastRow() + 1;  // 次の行に入力する
   var rowData = [];
   rowData[0] = new Date();   //タイムスタンプ

   for (var param in e.parameter) {
     var value = e.parameter[param];
     rowData[parseInt(param)] = value;
   }

   var newRange = sheet.getRange(newRow, 1, 1, rowData.length);
   newRange.setValues([rowData]);
   result =  'Ok'
 }
 return ContentService.createTextOutput(result);
}