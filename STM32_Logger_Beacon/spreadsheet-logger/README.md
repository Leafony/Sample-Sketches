# spreadsheet-logger
A data logger for Leafony beacons with Google SpreadSheet.

![image](./img/spreadsheet.png)

## Prerequisites
Enable Google SpreadSheet API and download `credientials.json`.

## Install

### Linux
#### Ubuntu/Debian

```
sudo apt-get install bluetooth libbluetooth-dev libudev-dev
git clone https://github.com/Leafony/Sample-Sketches/
cd STM32_Logger_Beacon/spreadsheet-logger
npm install
```

#### Raspberry Pi (Raspbian)

```
sudo apt update
sudo apt upgrade
sudo apt install -y bluetooth libbluetooth-dev libudev-dev git
git clone https://github.com/Leafony/Sample-Sketches/
cd STM32_Logger_Beacon/spreadsheet-logger
npm install
```

#### Fedra/ Other-RPM based

```
sudo yum install bluez bluez-libs bluez-libs-devel
git clone https://github.com/Leafony/Sample-Sketches/
cd STM32_Logger_Beacon/spreadsheet-logger
npm install
```

## Usage

Change `spreadsheetID` and `sheetName` according to your sheet.

```Javascript
const spreadsheetId = '<YOUR SPREADSHEET ID HERE>';
const sheetName = 'Sheet1';
```

Put your `credentials.json` in this project directory and run this script.

``` 
sudo node .
``` 

Only the first run, authentication messages will appear. Please follow the steps and copy & paste your auth ID.

## License
MIT