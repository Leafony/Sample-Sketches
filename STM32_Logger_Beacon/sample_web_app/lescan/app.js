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
const textTemp = document.getElementById('textTemp');
const textHumid = document.getElementById('textHumid');
const textIllum = document.getElementById('textIllum');
const textBatt = document.getElementById('textBatt');

const buttonConnect = document.getElementById('ble-connect-button');
const buttonDisconnect = document.getElementById('ble-disconnect-button');
const buttonLescan = document.getElementById('ble-lescan-button');
const buttonLestop = document.getElementById('ble-lestop-button');

let leafony;
let chart;

// array of received data
let array_temp = ['Temp'];
let array_humid = ['Humid'];
let array_illum = ['Illum'];
let array_batt = ['Batt'];


window.onload = function () {

	clearTable();
	initChart();

	leafony = new Leafony();
	leafony.lescan();

};


buttonConnect.addEventListener( 'click', function () {

	// initialize display
	clearTable();
	initChart();

	// connect to leafony
	leafony.onStateChange( function ( state ) {
		updateTable( state );
	} );
	leafony.onAdvertisementReceived( function ( state ) {
		let textDecoder = new TextDecoder('ascii');
      	let asciiString = textDecoder.decode(state);
		textTemp.innerHTML = asciiString;
		console.log("onAdvertisementReceived: " + asciiString);
	} );
	leafony.disableSleep();
	leafony.connect();

	buttonConnect.style.display = 'none';
	buttonDisconnect.style.display = '';

} );


buttonDisconnect.addEventListener( 'click', function () {

	leafony.disconnect();
	leafony = null;

	buttonConnect.style.display = '';
	buttonDisconnect.style.display = 'none';

} );


// buttonLescan.addEventListener( 'click', function () {
// 	console.log('buttonLescan: click');
// 	leafony.lescan();
// } );


// buttonLestop.addEventListener( 'click', function () {
// 	console.log('buttonLestop: click');
// 	leafony.lestop();
// } );


function clearTable () {

	textDeviceName.innerHTML = '';
	textUniqueName.innerHTML = '';
	textDateTime.innerHTML = '';
	textTemp.innerHTML = '';
	textHumid.innerHTML = '';
	textIllum.innerHTML = '';
	textBatt.innerHTML = '';

}

function initChart () {
	array_temp = ['Temp'];
	array_humid = ['Humid'];
	array_illum = ['Illum'];
	array_batt = ['Batt'];

	chart = c3.generate({
	    bindto: '#chart',
	    data: {
	      columns: [
			array_temp,
			array_humid,
			array_illum,
			array_batt
	      ]
	    }
	});
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
	textTemp.innerText = state.temp;
	textHumid.innerText = state.humd;
	textIllum.innerText = state.illm;
	textBatt.innerText = state.batt;

	array_temp.push( state.temp );
	array_humid.push( state.humd );
	array_illum.push( state.illm );
	array_batt.push( state.batt );

	chart.load({
		columns: [
			array_temp,
			array_humid,
			array_illum,
			array_batt
		]
	});
}