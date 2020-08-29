'use strict';

const noble = require('@abandonware/noble');
const textEncoding = require('text-encoding');

require('date-utils');

//discovered BLE device
const discovered = (peripheral) => {
    const TextDecoder = textEncoding.TextDecoder;
    const device = {
        name: peripheral.advertisement.localName,
        uuid: peripheral.uuid,
        rssi: peripheral.rssi,
        data: peripheral.advertisement.manufacturerData
    };

    if (String(device.name).match(/^Leaf_[A-Z]$/) != null){
        let dt = new Date();
        let dt_s = dt.toFormat('YYYY/MM/DD,HH24:MI:SS');
        let data_s = (new TextDecoder('utf-8')).decode(device.data);
        console.log(`${device.name},${dt_s},${data_s},${device.rssi}`);
    }
}

//BLE scan start
const scanStart = () => {
    noble.startScanning([], true);
    noble.on('discover', discovered);
    console.log('Start scanning...');
}

if(noble.state === 'poweredOn'){
    scanStart();
}else{
    noble.on('stateChange', scanStart);
}