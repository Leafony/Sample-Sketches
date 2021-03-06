/*
 * app.js: simple BLE connect application
 *
 * This application uses Web Bluetooth API.
 * Supporting OS and browsers are listed in the link below.
 * https://github.com/WebBluetoothCG/web-bluetooth/blob/master/implementation-status.md
 */

const textDevNameLe = document.getElementById('textDevNameLe');
const textTempLe = document.getElementById('textTempLe');
const textHumidLe = document.getElementById('textHumidLe');
const textBattLe = document.getElementById('textBattLe');
const textTimeLe = document.getElementById('textTimeLe');
const textGetDataStatus = document.getElementById('text-get-data-status');
const textClearEEPROMStatus = document.getElementById('text-clear-eeprom-status');

const alertBox = document.getElementById('alertBox');
const alertMessage = document.getElementById('alertMessage');

const buttonConnect = document.querySelectorAll('#ble-connect-button');
const buttonDisconnect = document.querySelectorAll('#ble-disconnect-button');
const buttonGetData = document.getElementById('ble-get-button');
const buttonLescan = document.getElementById('ble-lescan-button');
const buttonLestop = document.getElementById('ble-lestop-button');

const alertController = document.getElementById('alert-controller');
const buttonCheckVersion = document.getElementById('check-version-button');
const buttonCheckWake = document.getElementById('check-wake-button');
const buttonCheckSleep = document.getElementById('check-sleep-button');
const buttonCheckSens = document.getElementById('check-sens-button');
const buttonCheckSave = document.getElementById('check-save-button');
const buttonCheckAll = document.getElementById('check-all-button');
const buttonClearEEPROM = document.getElementById('clear-eeprom-button');
const buttonSubmitWake = document.getElementById('submit-wake-button');
const buttonSubmitSleep = document.getElementById('submit-sleep-button');
const buttonSubmitSens = document.getElementById('submit-sens-button');
const buttonSubmitSave = document.getElementById('submit-save-button');
const buttonSubmitAll = document.getElementById('submit-all-button');
const inputVersionText = document.getElementById('version-text-input');
const inputWakeText = document.getElementById('wake-text-input');
const inputSleepText = document.getElementById('sleep-text-input');
const inputSensText = document.getElementById('sens-text-input');
const inputSaveText = document.getElementById('save-text-input');

const buttonSetTime = document.getElementById('set-time-button');
const buttonDownload = document.getElementById('button-download');

let leafony;
let chartTemp, chartIlum, chartBatt;
let arrayTemp, arrayHumd, arrayIlum, arrayBatt;
let arrayTime;
let dataCount;

let stateRecv; // string

// Check OS
const platform = navigator.platform;
const userAgent = navigator.userAgent;
const isMac = /Mac/.test(platform);
const isAndroid = /Android/.test(userAgent);
const isScanningSupported = isAndroid;

/**
 * 
 */
window.onload = function () {
  if (!isScanningSupported) {
    document.getElementById('feature-beacon').style.display = 'none';
  }

  initChart();

  leafony = new Leafony();

  if (!leafony.getBleAvailability()) {
    alertBox.style.display = '';
    alertMessage.textContent = 'WebBluetooth API is not available on this device.'
  }

  leafony.onConnected(async function (uniqueName) {
    setTimeout(function () {
      sendCommand('setTime ' + timeStamp());

      for(let i=0; i<buttonConnect.length; i++) {
        buttonConnect[i].style.display = 'none';
        buttonDisconnect[i].style.display = '';
      }

      textUniqueName.innerText = uniqueName;

      buttonGetData.disabled = false;
      buttonCheckVersion.disabled = false;
      buttonCheckWake.disabled = false;
      buttonCheckSleep.disabled = false;
      buttonCheckSens.disabled = false;
      buttonCheckSave.disabled = false;
      buttonSubmitWake.disabled = false;
      buttonSubmitSleep.disabled = false
      buttonSubmitSens.disabled = false;
      buttonSubmitSave.disabled = false;
      buttonClearEEPROM.disabled = false;
    }, 1500);
  });

  // Beacon event
  leafony.onAdvertisementReceived((devname, state) => {
    onAdvertisementReceived(devname, state);
  });

  // Characteristics event
  leafony.onStateChange((state) => {
    onStateChange(state);
  });

  // Disconnected event
  leafony.onDisconnected((state) => {
    onDisconnected(state);
  });

  buttonGetData.disabled = true;
  buttonCheckVersion.disabled = true;
  buttonCheckWake.disabled = true;
  buttonCheckSleep.disabled = true;
  buttonCheckSens.disabled = true;
  buttonCheckSave.disabled = true;
  buttonSubmitWake.disabled = true;
  buttonSubmitSleep.disabled = true;
  buttonSubmitSens.disabled = true;
  buttonSubmitSave.disabled = true;
  buttonClearEEPROM.disabled = true;
};

/**
 * Download CSV button
 */
buttonDownload.addEventListener('click', () => {
  let bom_utf_8 = new Uint8Array([0xEF, 0xBB, 0xBF]);
  let csvText = `Device Name,${textUniqueName.innerText}\n`;

  for (var i = 0; i < arrayTemp.length; i++) {
    csvText += arrayTime[i] + "," + arrayTemp[i] + "," +
      arrayHumd[i] + "," + arrayIlum[i] + "," + arrayBatt[i] + '\n';
  }

  let blob = new Blob([bom_utf_8, csvText], { "type": "text/csv" });

  let url = window.URL.createObjectURL(blob);

  let downloader = document.getElementById("downloader");
  downloader.download = "data.csv";
  downloader.href = url;
  $("#downloader")[0].click();

  delete csvText;
  delete blob;
});

/**
 * Connect button
 */
for (let i=0; i<buttonConnect.length; i++) {
  buttonConnect[i].addEventListener('click', async () => {
    // Spinner connect button
    for (let j=0; j<buttonConnect.length; j++) {
      buttonConnect[j].disabled = true;
      buttonConnect[j].innerHTML = '<span class="spinner-border" role="status" aria-hidden="true"></span> Connecting...';
    }

    // initialize display
    initChart();

    // connect to leafony
    leafony.disableSleep();

    let isConnected = await leafony.connect();

    if (!isConnected) {
      for (let j=0; j<buttonConnect.length; j++) {
        buttonConnect[j].disabled = false;
        buttonConnect[j].innerHTML = 'Connect';
      }
    }

    return isConnected;
  });
}

/**
 * Disconnect button
 */
for (let i=0; i<buttonConnect.length; i++) {
  buttonDisconnect[i].addEventListener('click', () => {
    leafony.disconnect();

    for (let j=0; j<buttonConnect.length; j++) {
      buttonConnect[j].style.display = '';
      buttonDisconnect[j].style.display = 'none';
      buttonConnect[j].disabled = false;
      buttonConnect[j].innerHTML = 'Connect';
    }
  });
}

/**
 * Get Data button
 */
buttonGetData.addEventListener('click', function () {
  buttonGetData.innerHTML = '<span class="spinner-border" role="status" aria-hidden="true"></span> Receiving data...';
  buttonGetData.disabled = true;
  dataCount = 0;
  textGetDataStatus.innerText = 'Waiting for response...'
  stateRecv = 'getData';
  sendCommand('getData');
});


/**
 * Scanning button
 */
buttonLescan.addEventListener('click', function () {
  leafony.lescan();

  buttonLescan.style.display = 'none';
  buttonLestop.style.display = '';
});


/**
 * Stop scanning button
 */
buttonLestop.addEventListener('click', function () {
  leafony.lestop();

  buttonLescan.style.display = '';
  buttonLestop.style.display = 'none';
});


/**
 * Wake time check button
 */
buttonCheckVersion.addEventListener('click', function () {
  stateRecv = "checkVersion";
  sendCommand('version');
});


/**
 * Wake time check button
 */
buttonCheckWake.addEventListener('click', function () {
  stateRecv = "checkWake";
  sendCommand('getWake');
});


/**
 * Sleep time check button
 */
buttonCheckSleep.addEventListener('click', function () {
  stateRecv = "checkSleep";
  sendCommand('getSleep');
});


/**
 * Sens frequency check button
 */
buttonCheckSens.addEventListener('click', function () {
  stateRecv = "checkSens";
  sendCommand('getSensFreq');
});


/**
 * Save frequency check button
 */
buttonCheckSave.addEventListener('click', function () {
  stateRecv = "checkSave";
  sendCommand('getSaveFreq');
});


/**
 * Wake time submit button
 */
buttonSubmitWake.addEventListener('click', function () {
  if (!inputWakeText.value) {
    return;
  }
  sendCommand('setWake ' + inputWakeText.value);
});


/**
 * Sleep time submit button
 */
buttonSubmitSleep.addEventListener('click', function () {
  if (!inputSleepText.value) {
    return;
  }
  sendCommand('setSleep ' + inputSleepText.value);
});


/**
 * Sens frequency submit button
 */
buttonSubmitSens.addEventListener('click', function () {
  if (!inputSensText.value) {
    return;
  }
  sendCommand('setSensFreq ' + inputSensText.value);
});


/**
 * Save frequency submit button
 */
buttonSubmitSave.addEventListener('click', function () {
  if (!inputSaveText.value) {
    return;
  }
  sendCommand('setSaveFreq ' + inputSaveText.value);
});

/**
 * Wake time check button
 */
buttonClearEEPROM.addEventListener('click', function () {
  textClearEEPROMStatus.innerText = 'Waiting for response...'
  stateRecv = "clearEEPROM";
  sendCommand('clearEEPROM');
});

/**
 * Initialize Charts
 */
function initChart() {
  arrayTime = ['times', new Date(0)];
  arrayTemp = ['Temperature', 0];
  arrayHumd = ['Humidity', 0];
  arrayIlum = ['Illuminance', 0];
  arrayBatt = ['Battery', 0];

  chartTemp = c3.generate({
    bindto: '#chart-temp',
    data: {
      x: 'times',
      xFormat: '%Y/%m/%d %H:%M:%S',
      columns: [
        arrayTime,
        arrayTemp,
        arrayHumd,
      ],
      axes: {
        Humidity: 'y2'
      },
    },
    axis: {
      x: {
        type: 'timeseries',
        localtime: true,
        tick: {
          format: '%Y/%m/%d %H:%M:%S',
          rotate: -50,
          multiline: false,
          count: 10,
          culling: {
            max: 10,
          },
          height: 130,
          outer: false,
        },
        // min: new Date(2021, 03, 04, 10, 00, 0, 0),
        // max: new Date(2021, 03, 06, 10, 00, 0, 0),
      },
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
    },
  });

  chartIlum = c3.generate({
    bindto: '#chart-ilum',
    data: {
      x: 'times',
      xFormat: '%Y/%m/%d %H:%M:%S',
      columns: [
        arrayTime,
        arrayIlum,
      ],
      colors: {
        Illuminance: '#ff9896'
      }
    },
    axis: {
      x: {
        type: 'timeseries',
        localtime: true,
        tick: {
          format: '%Y/%m/%d %H:%M:%S',
          rotate: -50,
          multiline: false,
          count: 10,
          culling: {
            max: 10,
          },
          height: 130,
          outer: false,
        },
      },
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

  chartBatt = c3.generate({
    bindto: '#chart-batt',
    data: {
      x: 'times',
      xFormat: '%Y/%m/%d %H:%M:%S',
      columns: [
        arrayTime,
        arrayBatt,
      ],
      colors: {
        Battery: '#2ca02c'
      }
    },
    axis: {
      x: {
        type: 'timeseries',
        localtime: true,
        tick: {
          format: '%Y/%m/%d %H:%M:%S',
          rotate: -50,
          multiline: false,
          count: 10,
          culling: {
            max: 10,
          },
          height: 130,
          outer: false,
        },
      },
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
function decodeData(state) {
  // Decode received data
  let data = new Uint8Array(state.data.buffer);
  let temp = (data[0] * 256.0 + data[1]) / 256.0;
  let humd = (data[2] * 256.0 + data[3]) / 256.0;
  let illm = data[4] * 256.0 + data[5];
  let batt = (data[6] * 256.0 + data[7]) / 256.0;

  let unixtime = new Uint32Array(state.data.buffer)[2];
  let time = new Date(unixtime * 1000);
  var str_time = time.getFullYear()
    + '/' + ('0' + (time.getMonth() + 1)).slice(-2)
    + '/' + ('0' + time.getDate()).slice(-2)
    + ' ' + ('0' + time.getHours()).slice(-2)
    + ':' + ('0' + time.getMinutes()).slice(-2)
    + ':' + ('0' + time.getSeconds()).slice(-2);
  console.log(str_time, unixtime, temp, humd, illm, batt);

  // Append sensors values to array
  let min_date = new Date(2021, 1, 1);
  let max_date = new Date();
  if (time.getTime() > min_date.getTime() && time.getTime() <= max_date.getTime()) {
    arrayTime.push(str_time);
    arrayTemp.push(temp);
    arrayHumd.push(humd);
    arrayIlum.push(illm);
    arrayBatt.push(batt);
  }
}

const drawChart = () => {
  // グラフに最初から登録しているunixtime=0のデータを消す
  arrayTime.splice(1, 1);
  arrayTemp.splice(1, 1);
  arrayHumd.splice(1, 1);
  arrayIlum.splice(1, 1);
  arrayBatt.splice(1, 1);

  // Update charts
  chartTemp.load({
    columns: [
      arrayTime,
      arrayTemp,
      arrayHumd,
    ]
  });

  chartIlum.load({
    columns: [
      arrayTime,
      arrayIlum,
    ]
  });

  chartBatt.load({
    columns: [
      arrayTime,
      arrayBatt,
    ]
  });
};

/**
 * This function is called when bluetooth reveice data.
 * @param {*} state : received buffer
 */
function onStateChange(state) {
  let data = new Uint8Array(state.data.buffer);
  let recv = new TextDecoder('utf-8').decode(data);

  if (stateRecv === 'checkVersion') {
    inputVersionText.value = recv;
    stateRecv = 'main';
  }
  else if (stateRecv === 'checkSleep') {
    inputSleepText.value = parseInt(recv);
    stateRecv = 'main';
  }
  else if (stateRecv === 'checkWake') {
    inputWakeText.value = parseInt(recv);
    stateRecv = 'main';
  }
  else if (stateRecv === 'checkSens') {
    inputSensText.value = parseInt(recv);
    stateRecv = 'main';
  }
  else if (stateRecv === 'checkSave') {
    inputSaveText.value = parseInt(recv);
    stateRecv = 'main';
  }
  else if (stateRecv === 'checkAll') {
    stateRecv = 'main';
  }
  else if (stateRecv === 'clearEEPROM') {
    if (recv === 'finish') {
      console.log('clearEEPROM Finished!')
      textClearEEPROMStatus.innerText = 'EEPROM clear successfully finished!';
      leafony.disconnect(); // Automatically disconnect from Leafony.
      stateRecv = 'main';
    } else {
      textClearEEPROMStatus.innerText = `${parseInt(recv)}bytes/2040bytes cleared.`;
    }
  }
  else if (stateRecv === 'getData'){
    if (recv === 'finish') {
      console.log('getData Finished!');
      buttonGetData.innerHTML = 'Get Data';
      buttonGetData.disabled = false;
      drawChart();
      leafony.disconnect(); // Automatically disconnect from Leafony.
      stateRecv = 'main';
    }
    else {
      dataCount += 1;
      textGetDataStatus.innerText = `${dataCount}/170 samples are received.`;
      decodeData(state);
    }
  }
  else if (stateRecv === 'main') {
    console.error('Main state does not have any process.')
  }
  else {
    console.error('Invalid state.')
  }
}

/**
 * This function is called when Bluetooth receives advertising packet.
 * @param {*} state 
 */
function onAdvertisementReceived(devname, state) {
  let textDecoder = new TextDecoder('ascii');
  let asciiString = textDecoder.decode(state).split(',');
  textDevNameLe.innerText = devname;
  textTempLe.innerText = asciiString[0] + '℃';
  // textHumidLe.innerText = asciiString[1] + '%';
  textBattLe.innerText = asciiString[1] + 'V';
  textTimeLe.innerText = 'Last Update: ' + new Date().toTimeString();
  console.log("onAdvertisementReceived: " + asciiString);
}

/**
 * This function is called when Bluetooth is disconnected.
 * @param {*} state 
 */
function onDisconnected(state) {
  textUniqueName.innerText = '';

  for(let i=0; i<buttonConnect.length; i++) {
    buttonConnect[i].style.display = '';
    buttonDisconnect[i].style.display = 'none';
    buttonConnect[i].disabled = false;
    buttonConnect[i].innerHTML = 'Connect';
  }

  buttonGetData.disabled = true;
  buttonCheckVersion.disabled = true;
  buttonCheckWake.disabled = true;
  buttonCheckSleep.disabled = true;
  buttonCheckSens.disabled = true;
  buttonCheckSave.disabled = true;
  buttonSubmitWake.disabled = true;
  buttonSubmitSleep.disabled = true;
  buttonSubmitSens.disabled = true;
  buttonSubmitSave.disabled = true;
  buttonClearEEPROM.disabled = true;
}

/**
 * Send command to Leafony.
 * If leafony is not connected, show alert popups.
 * @param {string} command
 */
async function sendCommand(command) {
  if (leafony.isConnected()) {
    await leafony.sendCommand(command);
  } else {
    alertController.style.display = "";
  }
}

/**
 * Get Hex Timestamp Value
 */
function timeStamp() {
  let now = new Date();
  timestamp = Math.floor(now.getTime() / 1000);
  return timestamp;
}