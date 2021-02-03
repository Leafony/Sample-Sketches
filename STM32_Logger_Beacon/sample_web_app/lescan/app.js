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
const textTempLe = document.getElementById('textTempLe');
const textBattLe = document.getElementById('textBattLe');
const textTimeLe = document.getElementById('textTimeLe');

const alertBox = document.getElementById('alertBox');
const alertMessage = document.getElementById('alertMessage');

const buttonConnect = document.getElementById('ble-connect-button');
const buttonDisconnect = document.getElementById('ble-disconnect-button');
const buttonLescan = document.getElementById('ble-lescan-button');
const buttonLestop = document.getElementById('ble-lestop-button');

const alertController = document.getElementById('alert-controller');
const buttonCheckWake  = document.getElementById('check-wake-button');
const buttonCheckSleep = document.getElementById('check-sleep-button');
const buttonCheckSens  = document.getElementById('check-sens-button');
const buttonCheckSave  = document.getElementById('check-save-button');
const buttonCheckAll   = document.getElementById('check-all-button');
const buttonSubmitWake  = document.getElementById('submit-wake-button');
const buttonSubmitSleep = document.getElementById('submit-sleep-button');
const buttonSubmitSens  = document.getElementById('submit-sens-button');
const buttonSubmitSave  = document.getElementById('submit-save-button');
const buttonSubmitAll   = document.getElementById('submit-all-button');
const inputWakeText = document.getElementById('wake-text-input');
const inputSleepText = document.getElementById('sleep-text-input');
const inputSensText = document.getElementById('sens-text-input');
const inputSaveText = document.getElementById('save-text-input');


const buttonDownload = document.getElementById("button-download");


let leafony;
let chart_temp, chart_ilum, chart_batt;
let array_temp, array_humd, array_ilum, array_batt;

let recv_state; // string


/**
 * 
 */
window.onload = function () {

	initChart();

	leafony = new Leafony();

	if (!leafony.getBleAvailability()) {
		alertBox.style.display = '';
		alertMessage.textContent = 'WebBluetooth API is not available on this device.'
	}

	leafony.onConnected( function () {
		buttonConnect.style.display = 'none';
		buttonDisconnect.style.display = '';
	});

	// Beacon event
	leafony.onAdvertisementReceived( function ( state ) {
		onAdvertisementReceived( state );
	} );

	// Characteristics event
	leafony.onStateChange( function ( state ) {
		onStateChange( state );
	} );

	// Disconnected event
	leafony.onDisconnected( function ( state ) {
		onDisconnected( state );
	} );

};


/**
 * Connect button
 */
buttonConnect.addEventListener( 'click', function () {

	// initialize display
	clearTable();
	initChart();

	// connect to leafony
	leafony.disableSleep();
	leafony.connect();

	// Spinner connect button
	buttonConnect.disabled = true;
	buttonConnect.innerHTML = '<span class="spinner-border spinner-border-sm" role="status" aria-hidden="true"></span> Connecting...'
} );


/**
 * Disconnect button
 */
buttonDisconnect.addEventListener( 'click', function () {

	leafony.disconnect();

	buttonConnect.style.display = '';
	buttonDisconnect.style.display = 'none';
	buttonConnect.disabled = false;
	buttonConnect.innerHTML = 'Connect';

} );


/**
 * Scanning button
 */
buttonLescan.addEventListener( 'click', function () {
	leafony.lescan();

	buttonLescan.style.display = 'none';
	buttonLestop.style.display = '';
} );


/**
 * Stop scanning button
 */
buttonLestop.addEventListener( 'click', function () {
	leafony.lestop();

	buttonLescan.style.display = '';
	buttonLestop.style.display = 'none';
} );


/**
 * Wake time check button
 */
buttonCheckWake.addEventListener( 'click', function () {
	recv_state = "checkWake";
	sendCommand( 'getWake' );
} );


/**
 * Sleep time check button
 */
buttonCheckSleep.addEventListener( 'click', function () {
	recv_state = "checkSleep";
	sendCommand( 'getSleep' );
} );


/**
 * Sens frequency check button
 */
buttonCheckSens.addEventListener( 'click', function () {
	recv_state = "checkSens";
	sendCommand( 'getSensFreq' );
} );


/**
 * Save frequency check button
 */
buttonCheckSave.addEventListener( 'click', function () {
	recv_state = "checkSave";
	sendCommand( 'getSaveFreq' );
} );


/**
 * Wake time submit button
 */
buttonSubmitWake.addEventListener( 'click', function () {
	sendCommand( 'setWake ' + toString(inputWakeText.value));
} );


/**
 * Sleep time submit button
 */
buttonSubmitSleep.addEventListener( 'click', function () {
	sendCommand( 'setSleep ' + toString(inputSleepText.value));
} );


/**
 * Sens frequency submit button
 */
buttonSubmitSens.addEventListener( 'click', function () {
	sendCommand( 'setSensFreq ' + toString(inputSensText.value));
} );


/**
 * Save frequency submit button
 */
buttonSubmitSave.addEventListener( 'click', function () {
	sendCommand( 'setSaveFreq ' + toString(inputSaveText.value));
} );


/**
 * 
 */
function clearTable () {

	textDeviceName.innerHTML = '';
	textUniqueName.innerHTML = '';
	textDateTime.innerHTML = '';
	textTemp.innerHTML = '';
	textHumid.innerHTML = '';
	textIllum.innerHTML = '';
	textBatt.innerHTML = '';

}


/**
 * Initialize Charts
 */
function initChart () {
	array_temp = ['Temperature'];
	array_humd = ['Humidity'];
	array_ilum = ['Illuminance'];
	array_batt = ['Battery'];

	chart_temp = c3.generate({
	    bindto: '#chart_temp',
	    data: {
	      columns: [
			array_temp,
			array_humd,
		  ],
		  axes: {
			  Humidity: 'y2'
		  },
		},
		axis: {
			y: {
				max: 40,
				min: -10,
				label: {
					text: 'Temperature (℃)',
					position: 'outer-middle',
				}
			},
			y2: {
				show: true,
				max: 100,
				min: 0,
				label: {
					text: 'Humidity (%)',
					position: 'outer-middle',
				}
			}
		}
	});

	chart_ilum = c3.generate({
		bindto: '#chart_ilum',
		data: {
			columns: [
				array_ilum,
			],
			colors: {
				Illuminance: '#ff9896'
			}
		},
		axis: {
			y: {
				max: 10000,
				min: 0,
				label: {
					text: 'Illuminance (lux)',
					position: 'outer-middle',
				}
			}
		},

	});

	chart_batt = c3.generate({
		bindto: '#chart_batt',
		data: {
			columns: [
				array_batt,
			],
			colors: {
				Battery: '#2ca02c'
			}
		},
		axis: {
			y: {
				max: 4,
				min: 0,
				label: {
					text: 'Battery Voltage (V)',
					position: 'outer-middle',
				}
			}
		}
	});
}


/**
 * Update charts
 * @param {*} state: received ble state from onStateChange handler
 */
function updateChart( state ) {
	// Reveiced datetime
	let date = new Date();
	let year = String(date.getFullYear());
	let month = ('00' + (date.getMonth() + 1)).slice(-2);
	let day = ('00' + date.getDate()).slice(-2);
	let hours = ('00' + date.getHours()).slice(-2);
	let minutes = ('00' + date.getMinutes()).slice(-2);
	let seconds = ('00' + date.getSeconds()).slice(-2);
	let datetime = year + '/' + month + '/' + day + ' ' +
		hours + ':' + minutes + ':' + seconds;

	// Decode received data
	let data = new Uint8Array(state.data.buffer);
	// console.log(new TextDecoder("utf-8").decode(data));
	console.log(data);
	let temp = (data[0] * 256.0 + data[1]) / 256.0;
	let humd = (data[2] * 256.0 + data[3]) / 256.0;
	let illm = data[4] * 256.0 + data[5];
	let batt = (data[6] * 256.0 + data[7]) / 256.0;

	let unixtime = new Uint32Array(state.data.buffer)[2];
	let time = new Date(unixtime * 1000);
	// console.log(time, unixtime)

	textDeviceName.innerText = state.devn;
	textUniqueName.innerText = state.unin;
	textDateTime.innerText = datetime;
	textTemp.innerText = temp;
	textHumid.innerText = humd;
	textIllum.innerText = illm;
	textBatt.innerText = batt;

	// Append sensors values to array
	array_temp.push(temp);
	array_humd.push(humd);
	array_ilum.push(illm);
	array_batt.push(batt);

	// Update charts
	chart_temp.load({
		columns: [
			array_temp,
			array_humd,
		]
	});

	chart_ilum.load({
		columns: [
			array_ilum,
		]
	});

	chart_batt.load({
		columns: [
			array_batt,
		]
	});

}

/**
 * This function is called when bluetooth reveice data.
 * @param {*} state : received buffer
 */
function onStateChange(state) {
	console.log(recv_state);

	let data = new Uint8Array(state.data.buffer);
	let recv = new TextDecoder('utf-8').decode(data);

	if (recv_state == 'checkSleep'){
		inputSleepText.value = parseInt(recv);
		recv_state = 'main';
	}
	else if (recv_state == 'checkWake') {
		inputWakeText.value = parseInt(recv);
		recv_state = 'main';
	}
	else if (recv_state == 'checkSens') {
		inputSensText.value = parseInt(recv);
		recv_state = 'main';
	}
	else if (recv_state == 'checkSave') {
		inputSaveText.value = parseInt(recv);
		recv_state = 'main';
	}
	else if (recv_state == 'checkAll') {
		recv_state = 'main';
	}
	else{
		if (recv == 'finish') {
			console.log('Finish!');
		}
		else {
			updateChart(state);
		}
	}
}


/**
 * This function is called when Bluetooth receives advertising packet.
 * @param {*} state 
 */
function onAdvertisementReceived( state ) {

	let textDecoder = new TextDecoder('ascii');
	let asciiString = textDecoder.decode(state).split(',');
	textTempLe.innerText = asciiString[0] + '℃';
	textBattLe.innerText = asciiString[1] + 'V';
	textTimeLe.innerText = 'Last Update: ' + new Date().toTimeString();
	console.log("onAdvertisementReceived: " + asciiString);

}


/**
 * This function is called when Bluetooth is disconnected.
 * @param {*} state 
 */
function onDisconnected( state ) {
	buttonConnect.style.display = '';
	buttonDisconnect.style.display = 'none';
	buttonConnect.disabled = false;
	buttonConnect.innerHTML = 'Connect';
}


/**
 * Send command to Leafony.
 * If leafony is not connected, show alert popups.
 * @param {string} command
 */
function sendCommand( command ) {
	if ( leafony.isConnected() ) {
		leafony.sendCommand( command );
	} else {
		alertController.style.display = "";
	}
}

/**
 * Download CSV button
 */
buttonDownload.addEventListener( 'click', function () {

	let bom_utf_8 = new Uint8Array( [ 0xEF, 0xBB, 0xBF ] );
	let csvText = "";

	for (var i = 0; i < array_temp.length; i++ ) {
		csvText += String(i) + "," + array_temp[i] + "," +
				   array_humd[i] + "," + array_ilum[i] + "," + array_batt[i] + '\n';
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