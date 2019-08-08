/*
 * app.js: simple BLE connect application
 *
 * This application uses Web Bluetooth API.
 * Supporting OS and browsers are listed in the link below.
 * https://github.com/WebBluetoothCG/web-bluetooth/blob/master/implementation-status.md
 *
 * This application requires Trillion Node BLE Sensor Units.
 * This node sends sensor data in 1 packet in this format.
 *   temperature (4bytes), humidity (4bytes), illuminance (4bytes), tilt (4bytes)
 *
 * This application controlls by sending these commands.
 *   SND: start to send data
 *   STP: stop to send data
 *   PLS: LED blinking speed up
 *   MNS: LED blinking speed down
 */

// Bluetooth UUID
const GENERIC_ACCESS_UUID = "00001800-0000-1000-8000-00805f9b34fb";
const SERVICE_UUID = "442f1570-8a00-9a28-cbe1-e1d4212d53eb";
const CHARACTERISTIC_READ_UUID = "442f1571-8a00-9a28-cbe1-e1d4212d53eb";
const CHARACTERISTIC_WRITE_UUID = "442f1572-8a00-9a28-cbe1-e1d4212d53eb";
// Bluetooth device
var bluetoothDevice;
// connected service characteristic
var serviceCharacteristic;
// connected characteristic
var readCharacteristic;
var writeCharacteristic;
// Bluetooth device name
var deviceName;
var uniqueName;

// array of received data
var savedData = [];
// length of savedData
const CSV_BUFF_LEN = 1024;

// texts
const textDeviceName = document.getElementById('textDeviceName');
const textUniqueName = document.getElementById('textUniqueName');
const textDateTime = document.getElementById('textDateTime');
const textTemp = document.getElementById('textTemp');
const textHumid = document.getElementById('textHumid');
const textIllum = document.getElementById('textIllum');
const textTilt = document.getElementById('textTilt');
const textBatt = document.getElementById('textBatt');
const textDice = document.getElementById('textDice');
// Buttons
const buttonConnect = document.getElementById('ble-connect-button');
const buttonDisconnect = document.getElementById('ble-disconnect-button');
const buttonLedPls = document.getElementById('button-led-pls');
const buttonLedMns = document.getElementById('button-led-mns');
const buttonDownload = document.getElementById("button-download");
// ErrorMessage
const alertDisplay = document.getElementById('alert-display');
const alertButtonClose = document.getElementById('alert-button-close');
const alertMessage = document.getElementById('alert-message');

const strMonth = new Array("Jan.", "Feb.", "Mar.", "Apr.", "May", "June", "July", "Aug.", "Sept.", "Oct.", "Nov.", "Dec.");

// Check OS
const os = navigator.platform;
var isApple = os.match(/iPhone|iPad|iPod|Mac/);

/**
 * This Function is called when the window is loaded
 */
window.onload = function () {
	if (isApple) {
		console.log("GAP Characteristic is not supported on " + os.platform);
		document.getElementById("deviceNameTable").style.display = "none";
	}
	// clear all text in the table
	tableClear();
	// When the connect/disconnect button is clicked, these function is called.
	buttonConnect.addEventListener("click", connectBle);
	buttonDisconnect.addEventListener("click", onClickButtonDisconnect);
	// Hide error message
	hideErrorMessage();
};

/**
 * This function is called when the connect button clicked.
 */
function connectBle() {
	bluetoothDevice = null;

	console.log("Requesting Bluetooth Device...");
	// Open Bluetooth Search popup
	navigator.bluetooth.requestDevice({
		acceptAllDevices: true,
		// optionalServices: [SERVICE_UUID],
		optionalServices: [GENERIC_ACCESS_UUID, SERVICE_UUID],
	})
		.then(device => {
			bluetoothDevice = device;

			console.log("> Device Name: " + device.name);
			console.log("> ID: " + device.id);
			console.log("> Connected: " + device.gatt.connected);

			// Display device name on the table
			uniqueName = String(device.name);

			// When this bluetooth device is disconnected, onDisconnected function is called
			bluetoothDevice.addEventListener('gattserverdisconnected', onDisconnected);

			return device.gatt.connect();
		})
		.then(server => {
			console.log("Success to connect the Device. Getting service...");
			// get service matching UUID
			if (isApple) {
				// gerPrimaryServices function is not implemented in WebBLE
				// and connecting to GENERIC ACCESS is not supported on Apple devices
				console.log("getPrimeryService");
				return server.getPrimaryService(SERVICE_UUID);
			} else {
				console.log("getPrimeryServices");
				return server.getPrimaryServices();
			}
		})
		.then(services => {
			if (isApple) {
				connectCharacteristic(services);
				return;
			}

			console.log("Success to get services. Getting characterisctic...");
			let queue = Promise.resolve();
			services.forEach(service => {
				console.log("> Service " + service.uuid);
				switch(service.uuid) {
					// 'generic_access' service
					case GENERIC_ACCESS_UUID:
						queue = queue.then(_ => service.getCharacteristics().then(characteristics => {
							let queue = Promise.resolve();
							characteristics.forEach(characteristic => {
								console.log("> Characteristic " + characteristic.uuid);
								switch (characteristic.uuid){
									case BluetoothUUID.getCharacteristic('gap.device_name'):
						 				queue = queue.then(_ => readDeviceNameValue(characteristic));
								}
							});
							return queue;
						}));
					// write and read characteristics service
					case SERVICE_UUID:
						serviceCharacteristic = service;
						queue = queue.then(_ => service.getCharacteristics().then(characteristics => {
							let queue = Promise.resolve();
							characteristics.forEach(characteristic => {
								console.log("> Characteristic " + characteristic.uuid);
								switch (characteristic.uuid){
									// get READ_CHARACTERISTIC matching UUID
									case CHARACTERISTIC_READ_UUID:
										queue = queue.then(_ => getReadCharacteristic(characteristic));
									// get WRITE_CHARACTERISTIC matching UUID
						 			case CHARACTERISTIC_WRITE_UUID:
										queue = queue.then(_ => getWriteCharacteristic(characteristic));
								}
							});
							return queue;
						}));
				}
			});
			// hide connect button and display disconnect button
			buttonConnect.style.display = "none";
			buttonDisconnect.style.display = "";
			// hide error message
			hideErrorMessage();
			return queue;
		})
		.catch(error => {
			console.log("Error : " + error);
			// hide disconnect button and display connect button
			buttonConnect.style.display = "";
			buttonDisconnect.style.display = "none";
			// display error messages
			displayErrorMessage("<strong>エラー</strong> BLEとの接続に失敗しました。 <p>"
									 + String(error) + "</p>");
		});
}

function readDeviceNameValue(characteristic) {
	return characteristic.readValue().then(value => {
		deviceName = new TextDecoder().decode(value);
		console.log('> Device Name: ' + deviceName);
	});
}

function getReadCharacteristic(characteristic){
	console.log("Success to get read/notify characteristic");
	readCharacteristic = characteristic;
	return characteristic.startNotifications()
		.then(char => {
			// When Web Bluetooth API receives data, onValueChanged function is called.
			characteristic.addEventListener('characteristicvaluechanged', onValueChanged);
		});
}

function getWriteCharacteristic(characteristic){
	console.log("Success to get write characteristic");
	writeCharacteristic = characteristic;
	// send command to BLE device then it starts serving
	setTimeout(sendBleCommand, 1000, "SND");
}

/**
 * 
 * @param {*} service 
 */
function connectCharacteristic(service) {
	// get READ_CHARACTERISTIC matching UUID
	service.getCharacteristic(CHARACTERISTIC_READ_UUID)
		.then(characteristic => {
			console.log("Success to get read/notify characteristic");
			readCharacteristic = characteristic;
			return characteristic.startNotifications()
				.then(char => {
					// When Web Bluetooth API receives data, onValueChanged function is called.
					characteristic.addEventListener('characteristicvaluechanged', onValueChanged);
				});
		});
	// get WRITE_CHARACTERISTIC matching UUID
	service.getCharacteristic(CHARACTERISTIC_WRITE_UUID)
		.then(characteristic => {
			console.log("Success to get write characteristic");
			writeCharacteristic = characteristic;

			// send command to BLE device then it starts serving
			setTimeout(sendBleCommand, 1000, "SND");
		});
	// hide connect button and display disconnect button
	buttonConnect.style.display = "none";
	buttonDisconnect.style.display = "";
	// hide error message
	hideErrorMessage();
}

/**
 * 
 */
function onClickButtonDisconnect() {
	// send a command then BLE device stop serving
	sendBleCommand("STP");
	setTimeout(disconnectBle, 500);
}

/**
 * 
 */
function disconnectBle() {
	if (!bluetoothDevice) {
		return;
	}
	console.log("Disconnecting from Bluetooth Device...");
	if (bluetoothDevice.gatt.connected) {
		// disconnect
		bluetoothDevice.gatt.disconnect();
	} else {
		console.log("> Bluetooth Device is already disconnected.");
	}

}

/**
 * This function is called when the Bluetooth Device Disconected
 */
function onDisconnected() {
	console.log('> Bluetooth Device disconnected.');
	tableClear();
	// hide disconnect button and display connect button
	buttonConnect.style.display = "";
	buttonDisconnect.style.display = "none";
}

/**
 * 
 * @param {*} cmd
 */
function sendBleCommand(cmd) {
	if (writeCharacteristic) {
		var ArrayBuffer = new TextEncoder().encode(cmd);
		writeCharacteristic.writeValue(ArrayBuffer);
	}
}

/**
 * When new data received, this function is called.
 * @param {*} event
 */
function onValueChanged(event) {
	// decode received data to float array
	var receivedData = event.target.value;
	var decoder = new TextDecoder('utf-8')
	var strData = decoder.decode(receivedData)
	var arrayOfStrings = strData.split(',')
	var array = []
	arrayOfStrings.forEach(element => {
		array.push(parseFloat(element));
	});

	tableUpdate(array);
}

/**
 * Display received data on the device info table
 * @param {Array.<Float>} data received data
 */
function tableUpdate(data) {
	// get received time
	var date = new Date();
	var year = date.getFullYear();
	var month = date.getMonth();
	var day = date.getDate();
	var hours = date.getHours();
	var minutes = date.getMinutes();
	var seconds = date.getSeconds();

	// Datetime display mode
	var strYear = String(year);
	var strDate = ('00' + day).slice(-2);
	var strHours = ('00' + hours).slice(-2);
	var strMinutes = ('00' + minutes).slice(-2);
	var strSeconds = ('00' + seconds).slice(-2);
//	var strDay = strMonth[month] + " " + strDate + " " + strYear;
	// var strDay = strMonth[month] + " " + strDate + " " + strYear;
	var strDay = strYear + "/" + ('00' + (month + 1)).slice(-2) + "/" + strDate;
	var strTime = strHours + ":" + strMinutes + ":" + strSeconds;

	// Received sensors data
	var strTemp = String(data[0]);
	var strHumid = String(data[1]);
	var strIllum = String(data[2]);
	var strTilt = String(data[3]);
	var strBatt = String(data[4]);
	var strDice = String(data[5]);

	if (isApple) {
		deviceName = uniqueName;
	}

	// Update table field
	textDeviceName.innerHTML = deviceName;
	textUniqueName.innerHTML = uniqueName;
//	textDateTime.innerHTML = strDay + " at " + strTime;
	textDateTime.innerHTML = strDay + " " + strTime;
	textTemp.innerHTML = strTemp;
	textHumid.innerHTML = strHumid;
	textIllum.innerHTML = strIllum;
	textTilt.innerHTML = strTilt;
	textBatt.innerHTML = strBatt;
	textDice.innerHTML = strDice;

	// Change datatime display mode
//	strDay = strYear + "/" + ('00' + (month + 1)).slice(-2) + "/" + strDate;
	// Create array of reveived data and sensors data
	var darray = new Array(strDay, strTime, deviceName, uniqueName, strTemp, strHumid, strIllum, strTilt, strBatt, strDice);

	// Stack reveived data up to CSV_BUFF_LEN
	if (savedData.length >= CSV_BUFF_LEN) {
		savedData.shift();
	}
	savedData.push(darray)
}

/**
 * Clear all field in the device info table
 */
function tableClear() {
	textDeviceName.innerHTML = "";
	textUniqueName.innerHTML = "";
	textDateTime.innerHTML = "";
	textTemp.innerHTML = "";
	textHumid.innerHTML = "";
	textIllum.innerHTML = "";
	textTilt.innerHTML = "";
	textBatt.innerHTML = "";
	textDice.innerHTML = "";
}

/**
 * This function displaies error messages
 * @param {*} text
 */
function displayErrorMessage(text) {
	alertDisplay.style.display = "";
	alertMessage.innerHTML = text;
}

/**
 * This function hides error messages
 */
function hideErrorMessage() {
	alertDisplay.style.display = "none";
	alertMessage.innerHTML = "";
}

// Close button in alert display is clicked.
alertButtonClose.addEventListener("click", function () {
	alertDisplay.style.display = "none";
});

// LED+ button clicked
buttonLedPls.addEventListener("click", function () {
	console.log("LED Plus Button Clicked");
	// Send "PLS" comman
	sendBleCommand("PLS");
});

// LED- button clicked
buttonLedMns.addEventListener("click", function () {
	console.log("LED Minus Button Clicked");
	// Send "MNS" comman
	sendBleCommand("MNS");
});

// Download CSV button clicked
buttonDownload.addEventListener("click", function () {
	// UTF-8 BOM
	var bom = new Uint8Array([0xEF, 0xBB, 0xBF]);
	// CSV data
	var data_csv = "";

	// 1st row
	data_csv += "Date,Time,Device Name,Unique Name,Temp,Humid,Light,Tilt,BattVolt,Dice\n";
	// Write all received data in savedData
	for (var i = 0; i < savedData.length; i++) {
		for (var j = 0; j < savedData[i].length; j++) {
			data_csv += savedData[i][j];
			if (j == savedData[i].length - 1) data_csv += "\n";
			else data_csv += ",";
		}
	}

	// Convert data_csv to csv file
	var blob = new Blob([bom, data_csv], { "type": "text/csv" });
	// Create blob link
	var url = window.URL.createObjectURL(blob);
	// Set URL and filename on downloader and redirect to it
	var downloader = document.getElementById("downloader");
	downloader.download = "data.csv";
	downloader.href = url;
	$("#downloader")[0].click();

	delete data_csv;
});

