/**
 * @fileoverview Leafony Bluetoothリーフクラス
 */

/**
 * Leafonyクラス
 * @param none
 */
function Leafony() {

  const SERVICE_UUID = "442f1570-8a00-9a28-cbe1-e1d4212d53eb";
  const CHARACTERISTIC_READ_UUID = "442f1571-8a00-9a28-cbe1-e1d4212d53eb";
  const CHARACTERISTIC_WRITE_UUID = "442f1572-8a00-9a28-cbe1-e1d4212d53eb";

  // const NAME_FILTER = 'Leaf_Z';
  const NAME_PREFIX = 'Leaf_';

  let state = {};
  let char = {};

  let device;
  let server;
  let scan;

  let deviceName;
  let uniqueName;

  let enSleep = false;

  // Check OS
  const os = navigator.platform;
  var isApple = os.match(/iPhone|iPad|iPod|Mac/);

  /**
   * BLEが利用可能かを調べる関数
   */
  function getBleAvailability() {
    return navigator.bluetooth.getAvailability()
  }

  /**
   * Bluetoothリーフと接続する関数
   * @param none
   * @return none
   */
  async function connect() {
    try {
      device = await navigator.bluetooth.requestDevice({
        filters: [
          // {name: NAME_FILTER},  // Devices with name NAME_FILTER
          { namePrefix: NAME_PREFIX },  // Deices with name starting with NAME_PREFIX
        ],
        optionalServices: ['generic_access', SERVICE_UUID],
      });

      console.log('> Unique Name: ' + device.name);
      console.log('> ID: ' + device.id);
      console.log('> Connected: ' + device.gatt.connected);
      uniqueName = String(device.name);

      device.addEventListener('gattserverdisconnected', onDisconnected);

      server = await device.gatt.connect();

      connectService(server);
    } catch (error) {
      console.log(error);
      return false;
    }

    return true;
  }

  /**
   * Bluetooth GATT接続後serviceとcharacteristicに接続する関数
   * @param {*} server
   */
  async function connectService(server) {
    // デバイス名を取得 (Apple製品は未対応)
    if (!isApple) {
      await getDeviceName(server);
    }

    let service = await server.getPrimaryService(SERVICE_UUID);
    char.read = await service.getCharacteristic(CHARACTERISTIC_READ_UUID);
    char.write = await service.getCharacteristic(CHARACTERISTIC_WRITE_UUID);

    await char.read.startNotifications();
    char.read.addEventListener('characteristicvaluechanged', handleData);

    onConnected(uniqueName);
  }


  /**
   * 接続が確立したときに呼び出される関数
   */
  function onConnected() {
    onConnectedCallback(uniqueName);
  }

  /**
   * 接続が確立したときに呼び出されるコールバック
   */
  function onConnectedCallback() { }


  /**
   * 接続を確認する関数
   */
  function isConnected() {
    if (!device) {
      return false;
    }

    return device.gatt.connected;
  }

  /**
   * Characteristicの値が変化した時に呼び出される関数
   * @param {*} event
   */
  function handleData(event) {

    state.devn = deviceName;
    state.unin = uniqueName;
    state.data = event.target.value;

    onStateChangeCallback(state);

  }

  /**
   * characteristicの値が変化した時に呼び出されるユーザ任意処理
   */
  function onStateChangeCallback() { }

  /**
   * Bluetoothと切断する関数
   */
  function disconnect() {

    if (!device) {
      return;
    }
    console.log('Disconnecting from Bluetooth Device...');
    if (device.gatt.connected) {
      device.gatt.disconnect();
    } else {
      console.log('> Bluetooth Device is already disconnected');
    }

  }

  /**
   * Bluetoothと切断された時に呼び出される関数
   * @param {*} event
   */
  function onDisconnected(event) {

    console.log('> Bluetooth Device disconnected');

    onDisconnectedCallback(event);

    if (enSleep) {
      setTimeout(reconnect, 8000);
    }
  }

  /**
   * Bluetoothと切断された時に実行されるユーザ処理
   */
  function onDisconnectedCallback() { }

  /**
   * Sleep有効時、Bluetoothと再接続する関数
   */
  function reconnect() {

    if (!device) {
      return;
    }

    backoff(100 /* max retries */, 1 /* seconds delay */,
      function toTry() {
        console.log('Reconnecting to Bluetooth Devicve...');
        return device.gatt.connect();
      },
      function success(server) {
        console.log('> Bluetooth Device reconnected.');
        connectService(server);
      },
      function fail() {
        console.log('Failed to reconnect.');
        device = null;
        disconnect();
      });
  }

  /**
   * 再接続処理記述を簡単にするための再帰関数
   * @param {*} max
   * @param {*} delay
   * @param {*} toTry
   * @param {*} success
   * @param {*} fail
   */
  function backoff(max, delay, toTry, success, fail) {

    toTry().then(result => success(result))
      .catch(error => {
        if (max === 0) {
          return fail(error);
        }
        console.log('Retrying in ' + delay + 's...(' + max + ' tries left)');
        setTimeout(function () {
          backoff(--max, delay, toTry, success, fail);
        }, delay * 1000);
      });

  }


  /**
   * デバイス名を取得する関数
   * @param {*} server
   */
  async function getDeviceName(server) {

    let generic = await server.getPrimaryService('generic_access');
    let char = await generic.getCharacteristic('gap.device_name');
    let value = await char.readValue();
    deviceName = new TextDecoder().decode(value);
    console.log('> Device Name: ' + deviceName);

  }


  /**
   * Bluetoothにコマンドを送信する関数
   * @param {*} cmd
   */
  async function sendCommand(cmd) {

    if (char.write) {
      let ArrayBuffer = new TextEncoder().encode(cmd);
      await char.write.writeValue(ArrayBuffer);
    }

  }


  /**
   * Start Scanning
   * @param none
   */
  async function lescan() {
    try {
      scan = await navigator.bluetooth.requestLEScan({
        filters: [
          // {name: NAME_FILTER},  // Devices with name NAME_FILTER
          { namePrefix: NAME_PREFIX },  // Deices with name starting with NAME_PREFIX
        ],
        // acceptAllAdvertisements = true,
      });

      console.log('Scan started with:');
      console.log(' acceptAllAdvertisements: ' + scan.acceptAllAdvertisements);
      console.log(' active: ' + scan.active);
      console.log(' keepRepeatedDevices: ' + scan.keepRepeatedDevices);
      console.log(' filters: ' + JSON.stringify(scan.filters));

      navigator.bluetooth.addEventListener('advertisementreceived', event => {
        console.log('Advertisement received.');
        console.log('  Device Name: ' + event.device.name);
        console.log('  Device ID: ' + event.device.id);
        console.log('  RSSI: ' + event.rssi);
        console.log('  TX Power: ' + event.txPower);
        console.log('  UUIDs: ' + event.uuids);
        event.manufacturerData.forEach((valueDataView, key) => {
          logDataView('Manufacturer', key, valueDataView);
          onAdvertisementReceivedCallback(event.device.name, valueDataView.buffer);
        });
        event.serviceData.forEach((valueDataView, key) => {
          logDataView('Service', key, valueDataView);
        });

      });
    } catch (error) {
      console.log(error);
    }
  }


  /**
   * Stop Scanning
   * @param none
   */
  function lestop() {
    try {
      if (scan != null) {
        scan.stop();
        console.log('Scanning is stopped. scan.active = ' + scan.active);
        scan = null;
      }
    } catch (error) {
      console.log(error);
    }
  }

  function onAdvertisementReceivedCallback() { }

  const logDataView = (labelOfDataSource, key, valueDataView) => {
    const hexString = [...new Uint8Array(valueDataView.buffer)].map(b => {
      return b.toString(16).padStart(2, '0');
    }).join(' ');
    const textDecoder = new TextDecoder('ascii');
    const asciiString = textDecoder.decode(valueDataView.buffer);
    console.log(`  ${labelOfDataSource} Data: ` + key +
      '\n    (Hex) ' + hexString +
      '\n    (ASCII) ' + asciiString);
  };


  return {
    getBleAvailability: getBleAvailability,
    connect: connect,
    onConnected: function (callback) {
      onConnectedCallback = callback;
    },
    isConnected: isConnected,
    onStateChange: function (callback) {
      onStateChangeCallback = callback;
    },
    sendCommand: sendCommand,
    disconnect: function () {
      disconnect();
      device = null;
    },
    onDisconnected: function (callback) {
      onDisconnectedCallback = callback;
    },
    enableSleep: function () {
      enSleep = true;
    },
    disableSleep: function () {
      enSleep = false;
    },
    lescan: lescan,
    lestop: lestop,
    onAdvertisementReceived: function (callback) {
      onAdvertisementReceivedCallback = callback;
    }
  }
}