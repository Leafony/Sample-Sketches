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
const textDevNameLe = document.getElementById('textDevNameLe');
const textTempLe = document.getElementById('textTempLe');
const textHumidLe = document.getElementById('textHumidLe');
const textBattLe = document.getElementById('textBattLe');
const textTimeLe = document.getElementById('textTimeLe');

const alertBox = document.getElementById('alertBox');
const alertMessage = document.getElementById('alertMessage');

const buttonConnect = document.getElementById('ble-connect-button');
const buttonDisconnect = document.getElementById('ble-disconnect-button');
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

const buttonDownload = document.getElementById("button-download");

let leafony;
let chart_temp, chart_ilum, chart_batt;
let array_temp, array_humd, array_ilum, array_batt;
let array_time;

let recv_state; // string

// Check OS
const platform = navigator.platform;
const isMac = /Mac/.test(platform);
const userAgent = navigator.userAgent;
const isAndroid = /Android/.test(userAgent);
const isScanningSupported = isAndroid || isMac;

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

      buttonConnect.style.display = 'none';
      buttonDisconnect.style.display = '';

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
};


/**
 * Connect button
 */
buttonConnect.addEventListener('click', async () => {
  // Spinner connect button
  buttonConnect.disabled = true;
  buttonConnect.innerHTML = '<span class="spinner-border spinner-border-sm" role="status" aria-hidden="true"></span> Connecting...';

  // initialize display
  clearTable();
  initChart();

  // connect to leafony
  leafony.disableSleep();

  let isConnected = await leafony.connect();

  if (!isConnected) {
    buttonConnect.disabled = false;
    buttonConnect.innerHTML = 'Connect';
  }

  return isConnected();
});


/**
 * Disconnect button
 */
buttonDisconnect.addEventListener('click', () => {
  leafony.disconnect();

  buttonConnect.style.display = '';
  buttonDisconnect.style.display = 'none';
  buttonConnect.disabled = false;
  buttonConnect.innerHTML = 'Connect';
});


/**
 * Get Data button
 */
buttonGetData.addEventListener('click', function () {
  buttonGetData.innerHTML = '<span class="spinner-border spinner-border-sm" role="status" aria-hidden="true"></span> Receiving data...';
  buttonGetData.disabled = true;
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
  recv_state = "checkVersion";
  sendCommand('version');
});


/**
 * Wake time check button
 */
buttonCheckWake.addEventListener('click', function () {
  recv_state = "checkWake";
  sendCommand('getWake');
});


/**
 * Sleep time check button
 */
buttonCheckSleep.addEventListener('click', function () {
  recv_state = "checkSleep";
  sendCommand('getSleep');
});


/**
 * Sens frequency check button
 */
buttonCheckSens.addEventListener('click', function () {
  recv_state = "checkSens";
  sendCommand('getSensFreq');
});


/**
 * Save frequency check button
 */
buttonCheckSave.addEventListener('click', function () {
  recv_state = "checkSave";
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


buttonSetTime.addEventListener('click', function () {
  sendCommand('setTime ' + timeStamp());
});

/**
 * 
 */
function clearTable() {

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
function initChart() {
  array_time = ['times', new Date(0)];
  array_temp = ['Temperature', 0];
  array_humd = ['Humidity', 0];
  array_ilum = ['Illuminance', 0];
  array_batt = ['Battery', 0];

  chart_temp = c3.generate({
    bindto: '#chart_temp',
    data: {
      x: 'times',
      xFormat: '%Y/%m/%d %H:%M:%S',
      columns: [
        array_time,
        array_temp,
        array_humd,
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

  chart_ilum = c3.generate({
    bindto: '#chart_ilum',
    data: {
      x: 'times',
      xFormat: '%Y/%m/%d %H:%M:%S',
      columns: [
        array_time,
        array_ilum,
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

  chart_batt = c3.generate({
    bindto: '#chart_batt',
    data: {
      x: 'times',
      xFormat: '%Y/%m/%d %H:%M:%S',
      columns: [
        array_time,
        array_batt,
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
function updateChart(state) {
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
  var str_time = time.getFullYear()
    + '/' + ('0' + (time.getMonth() + 1)).slice(-2)
    + '/' + ('0' + time.getDate()).slice(-2)
    + ' ' + ('0' + time.getHours()).slice(-2)
    + ':' + ('0' + time.getMinutes()).slice(-2)
    + ':' + ('0' + time.getSeconds()).slice(-2);
  console.log(str_time, unixtime, temp, humd, illm, batt);

  textDeviceName.innerText = state.devn;
  textUniqueName.innerText = state.unin;
  textDateTime.innerText = datetime;
  textTemp.innerText = temp;
  textHumid.innerText = humd;
  textIllum.innerText = illm;
  textBatt.innerText = batt;

  // Append sensors values to array
  let min_date = new Date(2021, 1, 1);
  let max_date = new Date();
  if (time.getTime() > min_date.getTime() && time.getTime() <= max_date.getTime()) {
    array_time.push(str_time);
    array_temp.push(temp);
    array_humd.push(humd);
    array_ilum.push(illm);
    array_batt.push(batt);
  } else {
    console.log('invalid timestamp');
  }

}

/**
 * This function is called when bluetooth reveice data.
 * @param {*} state : received buffer
 */
function onStateChange(state) {
  let data = new Uint8Array(state.data.buffer);
  let recv = new TextDecoder('utf-8').decode(data);

  if (recv_state == 'checkVersion') {
    inputVersionText.value = recv;
    recv_state = 'main';
  }
  else if (recv_state == 'checkSleep') {
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
  else {
    if (recv == 'finish') {
      console.log('Finish!');
      buttonGetData.innerHTML = 'Get Data';
      buttonGetData.disabled = false;

      array_time.pop();
      array_temp.pop();
      array_humd.pop();
      array_ilum.pop();
      array_batt.pop();

      array_time.splice(1, 1); // グラフに最初から登録しているデータを消す
      array_temp.splice(1, 1);
      array_humd.splice(1, 1);
      array_ilum.splice(1, 1);
      array_batt.splice(1, 1);

      // Update charts
      chart_temp.load({
        columns: [
          array_time,
          array_temp,
          array_humd,
        ]
      });

      chart_ilum.load({
        columns: [
          array_time,
          array_ilum,
        ]
      });

      chart_batt.load({
        columns: [
          array_time,
          array_batt,
        ]
      });

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

  buttonConnect.style.display = '';
  buttonDisconnect.style.display = 'none';
  buttonConnect.disabled = false;
  buttonConnect.innerHTML = 'Connect';

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
 * Download CSV button
 */
buttonDownload.addEventListener('click', function () {

  let bom_utf_8 = new Uint8Array([0xEF, 0xBB, 0xBF]);
  let csvText = "";
  // let csvText = "timestamp,temperature,humidity,illuminance,battery_voltage\n";

  for (var i = 0; i < array_temp.length; i++) {
    csvText += array_time[i] + "," + array_temp[i] + "," +
      array_humd[i] + "," + array_ilum[i] + "," + array_batt[i] + '\n';
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
 * Get Hex Timestamp Value
 */
function timeStamp() {
  let now = new Date();
  timestamp = Math.floor(now.getTime() / 1000);
  return timestamp;
}