/*
 * app.js: simple BLE connect application
 *
 * This application uses Web Bluetooth API.
 * Supporting OS and browsers are listed in the link below.
 * https://github.com/WebBluetoothCG/web-bluetooth/blob/master/implementation-status.md
 */

const textDeviceName = document.getElementById('textDeviceName');
const textUniqueName = document.getElementById('textUniqueName');
const textDateTime = document.getElementById('textDateTime');
const textWeight = document.getElementById('textWeight');
const textBatt = document.getElementById('textBatt');

const buttonConnect = document.getElementById('ble-connect-button');
const buttonDisconnect = document.getElementById('ble-disconnect-button');
const buttonLedPls = document.getElementById('button-led-pls');
const buttonLedMns = document.getElementById('button-led-mns');
const buttonDownload = document.getElementById("button-download");

const switchSleepMode = document.getElementById('sleepmode-switch');

let leafony;

// array of received data
let savedData = [];
// length of savedData
const CSV_BUFF_LEN = 1024;


window.onload = function () {

	clearTable();

};


buttonConnect.addEventListener( 'click', function () {

	leafony = new Leafony();
	leafony.onStateChange( function ( state ) {
		updateTable( state );
	} );

	if ( switchSleepMode.checked ) {
		leafony.enableSleep();
	} else {
		leafony.disableSleep();
	}

	leafony.connect();

	buttonConnect.style.display = 'none';
	buttonDisconnect.style.display = '';


} );


buttonDisconnect.addEventListener( 'click', function () {

	leafony.disconnect();
	leafony = null;

	clearTable();
	buttonConnect.style.display = '';
	buttonDisconnect.style.display = 'none';

} );


function clearTable () {

	textDeviceName.innerHTML = '';
	textUniqueName.innerHTML = '';
	textDateTime.innerHTML = '';
	textWeight.innerHTML = '';
	textBatt.innerHTML = '';

}


function updateTable ( state ) {
	let date = new Date();
	let year     = String( date.getFullYear() );
	let month    = ( '00' + ( date.getMonth() + 1 ) ).slice( -2 );
	let day      = ( '00' + date.getDate() ).slice( -2 );
	let hours    = ( '00' + date.getHours() ).slice( -2 );
	let minutes  = ( '00' + date.getMinutes() ).slice( -2 );
	let seconds  = ( '00' + date.getSeconds() ).slice( -2 );
	let datetime = year + '/' + month + '/' + day + ' ' +
				   hours + ':' + minutes + ':' + seconds;

	textDeviceName.innerText = state.devn;
	textUniqueName.innerText = state.unin;
	textDateTime.innerText = datetime;
	textWeight.innerText = state.weight;
	textBatt.innerText = state.batt;

	// Create array of reveived data and sensors data
	let darray = new Array(
		datetime,
		state.devn,
		state.unin,
		state.weight,
		state.batt);

	// stack reveived data up to CSV_BUFF_LEN
	if (savedData.length >= CSV_BUFF_LEN) {
		savedData.shift();
	}
	savedData.push( darray );
}




buttonDownload.addEventListener( 'click', function () {

	let bom_utf_8 = new Uint8Array( [ 0xEF, 0xBB, 0xBF ] );
	let csvText = "";

	csvText += "Datetime,Device Name,Unique Name,Weight,BattVolt,Humid,Light,Tilt,Dice\n";
	// Write all received data in savedData
	for ( var i = 0; i < savedData.length; i++ ) {
		for ( var j = 0; j < savedData[i].length; j++ ) {
			csvText += savedData[i][j];
			if ( j == savedData[i].length - 1 ) csvText += "\n";
			else csvText += ",";
		}
	}

	let blob = new Blob( [ bom_utf_8, csvText ], { "type": "text/csv" } );

	let url = window.URL.createObjectURL( blob );

	let downloader = document.getElementById( "downloader" );
	downloader.download = "data.csv";
	downloader.href = url;
	$( "#downloader" )[0].click();

	delete csvText;
	delete blob;
});