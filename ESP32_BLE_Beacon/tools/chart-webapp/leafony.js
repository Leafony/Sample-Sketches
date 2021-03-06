/**
 * @fileoverview Leafony Bluetoothリーフクラス
 */

 /**
  * Leafonyクラス
  * @param none
  */
function Leafony() {

    const SERVICE_UUID = "ec6ba320-e16b-11ea-87d0-0242ac130003";
    const CHARACTERISTIC_READ_UUID = "ec6ba321-e16b-11ea-87d0-0242ac130003";
    const CHARACTERISTIC_WRITE_UUID = "ec6ba321-e16b-11ea-87d0-0242ac130003";

    let state = {};
    let char = {};

    let device;
    let server;

    let deviceName;
    let uniqueName;

    let enSleep = false;

    // Check OS
    const os = navigator.platform;
    var isApple = os.match(/iPhone|iPad|iPod|Mac/);

    /**
     * Bluetoothリーフと接続する関数
     * @param none
     * @return none
     */
    async function connect () {

        try {
            device = await navigator.bluetooth.requestDevice({
	        filters: [{name: 'Leaf_Z'}],
	        // acceptAllDevices: true,
	        optionalServices: [ 'generic_access', SERVICE_UUID ],
            });

            console.log( '> Unique Name: ' + device.name );
            console.log( '> ID: ' + device.id );
            console.log( '> Connected: ' + device.gatt.connected );
            uniqueName = String( device.name );

            device.addEventListener( 'gattserverdisconnected', onDisconnected );

            server = await device.gatt.connect();

            connectService( server );
        } catch ( error ) {
            console.log( error );
        }

    }

    /**
     * Bluetooth GATT接続後serviceとcharacteristicに接続する関数
     * @param {*} server 
     */
    async function connectService ( server ) {

        // デバイス名を取得 (Apple製品は未対応)
        if (!isApple) {
            await getDeviceName( server );
        }

        let service = await server.getPrimaryService( SERVICE_UUID );
        char.read = await service.getCharacteristic( CHARACTERISTIC_READ_UUID );
        char.write = await service.getCharacteristic( CHARACTERISTIC_WRITE_UUID );

        await char.read.startNotifications();
        char.read.addEventListener( 'characteristicvaluechanged', handleData );

        // ログデータ送信命令
        setTimeout( sendCommand, 1000, 'get' );

    }

    /**
     * Characteristicの値が変化した時に呼び出される関数
     * @param {*} event 
     */
    function handleData( event ) {

        let data = event.target.value;
        let decoder = new TextDecoder( 'utf-8' );
        data = decoder.decode( data );
        data = data.replace( /\r?\n/g, '' );
        data = data.split( ',' );

        console.log(data);

        state.devn = deviceName;
        state.unin = uniqueName;

        state.temp = data[0];
        state.humd = data[1];
        state.illm = data[2];
        state.batt = data[3];

        onStateChangeCallback( state );

    }

    /**
     * characteristicの値が変化した時に呼び出されるユーザ任意処理
     */
    function onStateChangeCallback() {}

    /**
     * Bluetoothと切断する関数
     */
    function disconnect () {

        if ( !device ) {
            return;
        }
        console.log( 'Disconnecting from Bluetooth Device...' );
        if ( device.gatt.connected ) {
            device.gatt.disconnect();
        } else {
            console.log( '> Bluetooth Device is already disconnected' );
        }

    }

    /**
     * Bluetoothと切断された時に呼び出される関数
     * @param {*} event 
     */
    function onDisconnected ( event ) {

        console.log( '> Bluetooth Device disconnected' );

        onDisconnectedCallback( event );

        if ( enSleep ) {
            setTimeout( reconnect, 8000 );
        }
    }

    /**
     * Bluetoothと切断された時に実行されるユーザ処理
     */
    function onDisconnectedCallback () {}

    /**
     * Sleep有効時、Bluetoothと再接続する関数
     */
    function reconnect() {

    	if ( !device ) {
    		return;
    	}

    	backoff( 100 /* max retries */, 1 /* seconds delay */,
    		function toTry () {
    			console.log( 'Reconnecting to Bluetooth Devicve...' );
    			return device.gatt.connect();
    		},
    		function success ( server ) {
    			console.log( '> Bluetooth Device reconnected.' );
    			connectService( server );
    		},
    		function fail () {
    			console.log( 'Failed to reconnect.' );
    			device = null;
    			disconnect();
    		} );
    }

    /**
     * 再接続処理記述を簡単にするための再帰関数
     * @param {*} max 
     * @param {*} delay 
     * @param {*} toTry 
     * @param {*} success 
     * @param {*} fail 
     */
    function backoff( max, delay, toTry, success, fail ) {

    	toTry().then( result => success( result ) )
    		.catch( error => {
    			if ( max === 0 ) {
    				return fail( error );
    			}
    			console.log( 'Retrying in ' + delay + 's...(' + max + ' tries left)' );
    			setTimeout( function () {
    				backoff( --max, delay, toTry, success, fail );
    			}, delay * 1000 );
            } );

    }


    /**
     * デバイス名を取得する関数
     * @param {*} server 
     */
    async function getDeviceName( server ) {

        let generic = await server.getPrimaryService( 'generic_access' );
        let char = await generic.getCharacteristic( 'gap.device_name' );
        let value = await char.readValue();
        deviceName = new TextDecoder().decode( value );
        console.log( '> Device Name: ' + deviceName );

    }


    /**
     * Bluetoothにコマンドを送信する関数
     * @param {*} cmd 
     */
    async function sendCommand( cmd ) {

        if ( char.write ) {
            let ArrayBuffer = new TextEncoder().encode( cmd );
            await char.write.writeValue( ArrayBuffer );
        }

    }


    return {
        connect: connect,
        onStateChange: function ( callback ) {
            onStateChangeCallback = callback;
        },
        sendCommand: sendCommand,
        disconnect: function () {
            disconnect();
            device = null;
        },
        onDisconnected: function ( callback ) {
            onDisconnectedCallback = callback;
        },
        enableSleep: function () {
            enSleep = true;
        },
        disableSleep: function () {
            enSleep = false;
        }
    }
}
