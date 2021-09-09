'use strict';

const fs = require('fs');
const readline = require('readline');
const {google} = require('googleapis');
const noble = require('@abandonware/noble');
const textEncoding = require('text-encoding');
require('date-utils');

// If modifying these scopes, delete token.json.
const SCOPES = ['https://www.googleapis.com/auth/spreadsheets'];
// The file token.json stores the user's access and refresh tokens, and is
// created automatically when the authorization flow completes for the first
// time.
const TOKEN_PATH = 'token.json';

let authenticate;
const spreadsheetId = '1HYl9B-q_beEfXjmkPzGVbWkzENH3AKXh-43sfCywBhQ';
const sheetName = 'Sheet1';

// Load client secrets from a local file.
fs.readFile('credentials.json', (err, content) => {
  if (err) return console.log('Error loading client secret file:', err);
  // Authorize a client with credentials, then call the Google Sheets API.
  authorize(JSON.parse(content), scanBeacon);
});

/**
 * Create an OAuth2 client with the given credentials, and then execute the
 * given callback function.
 * @param {Object} credentials The authorization client credentials.
 * @param {function} callback The callback to call with the authorized client.
 */
const authorize = (credentials, callback) => {
  const {client_secret, client_id, redirect_uris} = credentials.installed;
  const oAuth2Client = new google.auth.OAuth2(
      client_id, client_secret, redirect_uris[0]);

  // Check if we have previously stored a token.
  fs.readFile(TOKEN_PATH, (err, token) => {
    if (err) return getNewToken(oAuth2Client, callback);
    oAuth2Client.setCredentials(JSON.parse(token));
    callback(oAuth2Client);
  });
}

/**
 * Get and store new token after prompting for user authorization, and then
 * execute the given callback with the authorized OAuth2 client.
 * @param {google.auth.OAuth2} oAuth2Client The OAuth2 client to get token for.
 * @param {getEventsCallback} callback The callback for the authorized client.
 */
const getNewToken = (oAuth2Client, callback) => {
  const authUrl = oAuth2Client.generateAuthUrl({
    access_type: 'offline',
    scope: SCOPES,
  });
  console.log('Authorize this app by visiting this url:', authUrl);
  const rl = readline.createInterface({
    input: process.stdin,
    output: process.stdout,
  });
  rl.question('Enter the code from that page here: ', (code) => {
    rl.close();
    oAuth2Client.getToken(code, (err, token) => {
      if (err) return console.error('Error while trying to retrieve access token', err);
      oAuth2Client.setCredentials(token);
      // Store the token to disk for later program executions
      fs.writeFile(TOKEN_PATH, JSON.stringify(token), (err) => {
        if (err) return console.error(err);
        console.log('Token stored to', TOKEN_PATH);
      });
      callback(oAuth2Client);
    });
  });
}

/**
 * Write column titles:
 * @param {google.auth.OAuth2} auth The authenticated Google OAuth client.
 */
const writeTitles = (auth) => {
  const sheets = google.sheets({version: 'v4', auth});
  sheets.spreadsheets.values.update({
    spreadsheetId: spreadsheetId,
    range: sheetName + '!A1:G1',
    valueInputOption: "USER_ENTERED",
    resource: {
        values: [
          ['Device', 'Datetime', 'Temperature', 'Humidity', 'Illuminum', 'Battery', 'RSSI']
        ],
    },
  }, (err, res) => {
    if (err) {
        console.log('The API returned an error: ' + err);
        return;
    }
  });
}

/**
 * Append values in a spreadsheet:
 * @param {google.auth.OAuth2} auth The authenticated Google OAuth client.
 * @param {Array} value The list of row data
 */
const appendData = (auth, values) => {
  const sheets = google.sheets({version: 'v4', auth});
  sheets.spreadsheets.values.append({
    spreadsheetId: spreadsheetId,
    range: sheetName,
    valueInputOption: "USER_ENTERED",
    resource: {
        values: values,
    },
  }, (err, res) => {
    if (err) {
        console.log('The API returned an error: ' + err);
        return;
    }
  });
}

const scanBeacon = (auth) => {
    authenticate = auth;
    writeTitles(auth);

    if(noble.state === 'poweredOn'){
        scanStart();
    }else{
        noble.on('stateChange', scanStart);
    }
}

//BLE scan start
const scanStart = () => {
    noble.startScanning([], true);
    noble.on('discover', discovered);
    console.log('Start scanning...');
}

//discovered BLE device
const discovered = (peripheral) => {
    const TextDecoder = textEncoding.TextDecoder;
    const device = {
        name: peripheral.advertisement.localName,
        uuid: peripheral.uuid,
        rssi: peripheral.rssi,
        data: peripheral.advertisement.manufacturerData
    };

    if (String(device.name).match(/^Leaf_\w+$/) != null){
        let dt = new Date();
        let dt_s = dt.toFormat('YYYY/MM/DD HH24:MI:SS');

        // Decode sensors data (for STM32 MCU)
        let light = (device.data[0] << 8) + device.data[1];
        let temperature = ((device.data[2] << 8) + device.data[3]) / 256;
        let humid = ((device.data[4] << 8) + device.data[5]) / 256;
        let battery = ((device.data[6] << 8) + device.data[7]) / 256;

        let values = [
          [device.name, dt_s, temperature, humid, light, battery, device.rssi]
        ];
        appendData(authenticate, values);
        console.log(`${device.name},${dt_s}`);
    }
}
