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

const alertBox = document.getElementById('alertBox');
const alertMessage = document.getElementById('alertMessage');

const buttonDownload = document.getElementById("button-download");

let leafony;
let chart_temp, chart_ilum, chart_batt;

// array of received data
let array_temp, array_humd, array_ilum, array_batt;

window.onload = function () {

	clearTable();
	initChart();

	leafony = new Leafony();
	// Check BLE availability
	if (!leafony.getBleAvailability()) {
		alertBox.style.display = '';
		alertMessage.textContent = 'お使いのデバイスではWebBluetoothがご利用いただけません。'
	}
	leafony.lescan();

};


buttonConnect.addEventListener( 'click', function () {

	// initialize display
	clearTable();
	initChart();

	// connect to leafony
	leafony.onStateChange( function ( state ) {
		onStateChange( state );
	} );
	leafony.onAdvertisementReceived( function ( state ) {
		onAdvertisementReceived( state );
	} );
	leafony.disableSleep();
	leafony.connect();

	buttonConnect.style.display = 'none';
	buttonDisconnect.style.display = '';

} );


buttonDisconnect.addEventListener( 'click', function () {

	leafony.disconnect();
	leafony = new Leafony();

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


/**
 * Initialize Chart
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
 * This function is called when bluetooth reveice data.
 * @param {*} state : received buffer
 */
function onStateChange(state) {
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
	let temp = (data[0] * 256.0 + data[1]) / 256.0;
	let humd = (data[2] * 256.0 + data[3]) / 256.0;
	let illm = data[4] * 256.0 + data[5];
	let batt = (data[6] * 256.0 + data[7]) / 256.0;

	let unixtime = new Uint32Array(state.data.buffer)[2];
	let time = new Date(unixtime * 1000);
	console.log(time, unixtime)

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
 * This function is called when Bluetooth receive advertising packet.
 * @param {*} state 
 */
function onAdvertisementReceived(state) {

	let textDecoder = new TextDecoder('ascii');
	let asciiString = textDecoder.decode(state);
	textTemp.innerHTML = asciiString;
	console.log(state);
	console.log("onAdvertisementReceived: " + asciiString);

}


/**
 * Download CSV
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