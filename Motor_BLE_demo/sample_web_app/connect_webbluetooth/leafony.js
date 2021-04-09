/**
 * @fileoverview Leafony Bluetooth leaf class
 */

 /**
  * Leafony class
  * @param none
  */
function Leafony() {

    const SERVICE_UUID = "442f1570-8a00-9a28-cbe1-e1d4212d53eb";
    const CHARACTERISTIC_READ_UUID = "442f1571-8a00-9a28-cbe1-e1d4212d53eb";
    const CHARACTERISTIC_WRITE_UUID = "442f1572-8a00-9a28-cbe1-e1d4212d53eb";

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
     * Functions to connect to a BLE leaf
     * @param none
     * @return none
     */
    async function connect () {

        try {
            device = await navigator.bluetooth.requestDevice( {
                acceptAllDevices: true,
                optionalServices: [ 'generic_access', SERVICE_UUID ],               
            } );

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
     * Function to connect to service and characteristic after Bluetooth GATT connection
     * @param {*} server 
     */
    async function connectService ( server ) {

        // Get device name (Apple products are not supported)
        if (!isApple) {
            await getDeviceName( server );
        }

        let service = await server.getPrimaryService( SERVICE_UUID );
        char.read = await service.getCharacteristic( CHARACTERISTIC_READ_UUID );
        char.write = await service.getCharacteristic( CHARACTERISTIC_WRITE_UUID );

        await char.read.startNotifications();
        char.read.addEventListener( 'characteristicvaluechanged', handleData );

        setTimeout( sendCommand, 1000, 'SND' );

    }

    /**
     * Function to be called when the value of Characteristic changes.
     * @param {*} event 
     */
    function handleData( event ) {
        
        let data = event.target.value;
        let decoder = new TextDecoder( 'utf-8' );
        data = decoder.decode( data );
        data = data.replace( /\r?\n/g, '' );
        data = data.split( ',' );

        state.devn = deviceName;
        state.unin = uniqueName;
        state.data1 = data[0];
        state.data2 = data[1];
        state.data3 = data[2];
        state.data4 = data[3];
        state.data5 = data[4];
//        state.data6 = data[5];

        onStateChangeCallback( state );

        if ( enSleep ) {
            sendCommand( 'STP' );
            disconnect();
        }
    }

    /**
     * User-defined process called when the value of a characteristic changes.
     */
    function onStateChangeCallback() {}

    /**
     * Function to disconnect from Bluetooth
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
     * Function called when disconnected from Bluetooth.
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
     * User process to be executed when disconnected from Bluetooth.
     */
    function onDisconnectedCallback () {}

    /**
     * Function to reconnect with Bluetooth when Sleep is enabled.
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
     * Recursive functions to simplify the description of reconnection processes.
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
     * Function to get the device name
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
     * Function to send commands to Bluetooth
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