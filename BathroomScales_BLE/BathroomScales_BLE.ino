//=====================================================================
//  Leafony Platform sample sketch
//     Application  : Bathroom Scales BLE demo
//     Processor    : ATmega328P (3.3V /8MHz)
//     Confirmed in Arduino IDE 1.8.7
//
//     Leaf configuration
//       (1) AP01 AVR MCU
//       (2) AC02 BLE Sugar
//       (3) AZ01 USB
//       (4) HX711
//
//    (c)2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01 First release
//=====================================================================
//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <MsTimer2.h>                       // Timer
#include <SoftwareSerial.h>                 // Software UART
#include <Wire.h>                           // I2C
#include "TBGLib.h"                         // BLE
#include <HX711_ADC.h>
#include <EEPROM.h>

//===============================================
// BLE Unique Name (Local device name)
// Up to 16 characters (ASCII code)
//===============================================
//                     |1234567890123456|
String strDeviceName = "Leafony_AW";

//===============================================
// Output to serial monitor
//      #define SERIAL_MONITOR = With output
//    //#define SERIAL_MONITOR = Without output (Comment out)
//===============================================
#define SERIAL_MONITOR

//===============================================
// Debug output to serial monitor
//      #define DEBUG = With output
//    //#define DEBUG = Without output (Comment out)
//===============================================
//#define DEBUG

//-----------------------------------------------
// Setting the transmission interval
//  SEND_INTERVAL  :transmission interval (Set the interval for sending sensor data in 1 second increments.)
//-----------------------------------------------
#define SEND_INTERVAL   (1)                 // 1s

//-----------------------------------------------
// IO pin name definition
// Define it according to the leaf to be connected.
//-----------------------------------------------
const int HX711_dout    =  4;      //mcu > HX711 dout pin
const int HX711_sck     =  5;      //mcu > HX711 sck pin
const int D7_BLE_WAKEUP =  7;
const int D15_BLE_TX    = 15;
const int D16_BLE_RX    = 16;

//-----------------------------------------------
// Define constants to be used in the program
//-----------------------------------------------
//------------------------------
// I2C address
//------------------------------
#define BATT_ADC_ADDR   0x50                // Battery ADC

//------------------------------
// Loop interval
// Timer interrupt interval (ms)
//------------------------------
#define LOOP_INTERVAL 125                   // 125ms interval

//------------------------------
// BLE
//------------------------------
#define BLE_STATE_STANDBY               (0)
#define BLE_STATE_SCANNING              (1)
#define BLE_STATE_ADVERTISING           (2)
#define BLE_STATE_CONNECTING            (3)
#define BLE_STATE_CONNECTED_MASTER      (4)
#define BLE_STATE_CONNECTED_SLAVE       (5)

//---------------------------------------------------------------------
// object
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);


//------------------------------
// BLE
//------------------------------
SoftwareSerial Serialble(D16_BLE_RX, D15_BLE_TX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0 );

//---------------------------------------------------------------------
// Define variables to be used in the program
//---------------------------------------------------------------------
//------------------------------
// Loop counter
//------------------------------
uint8_t iLoop1s = 0;
uint8_t iSendCounter = 0;

//------------------------------
// Event
//------------------------------
bool event1s = false;

//------------------------------
// Interval Timer interrupt
//------------------------------
volatile bool bInterval = false;

//------------------------------
// HX711
//------------------------------
float newCalibrationValue = 19800;

const int calVal_eepromAdress = 0;
unsigned long t = 0;
float dataWeight = 0;
float totalWeight = 0;
int addTimes = 0;

//------------------------------
// Battery
//------------------------------
float dataBatt = 0;

//------------------------------
// BLE
//------------------------------
bool bBLEconnect = false;
bool bBLEsendData = false;

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0;         // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF;        // 0xFF = no bonding, otherwise = bonding handle

//=====================================================================
// setup
//=====================================================================
void setup() {
  delay(500);

  Serial.begin(115200);     // UART 115200bps
  Wire.begin();             // I2C 100kHz
#ifdef SERIAL_MONITOR
    Serial.println(F("========================================="));
    Serial.println("Starting...");
#endif

  setupPort();
  delay(10);
  setupSensor();
  setupBLE();

  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));
  delay(1000);

  noInterrupts();
  setupTCInt();
  interrupts();

  MsTimer2::start();                        // Timer2 inverval start
#ifdef SERIAL_MONITOR
    Serial.println();
    Serial.println("=========================================");
    Serial.println(F("loop start"));
    Serial.println();
#endif
}

//-----------------------------------------------
// IO pin input/output settings
// Configure the settings according to the leaf to be connected.
//-----------------------------------------------
void setupPort(){
  pinMode(D7_BLE_WAKEUP, OUTPUT);           // PD7 : digital 7 = BLE Wakeup/Sleep
  digitalWrite(D7_BLE_WAKEUP, HIGH);        // BLE Wakeup

}

//---------------------------------------------------------------------
// Initial settings for each device
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
void setupSensor(){
// LoadCell Setup
  LoadCell.begin();
  unsigned long stabilizingtime = 2000;   // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                   //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
//    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    Serial.println("Timeout, check MCU>HX711");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0);           // user set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate();                            //start calibration procedure
}

//=====================================================================
// Interrupt
//=====================================================================
//----------------------------------------------
// Interrupt initialization
// Timer interrupt (interval=125ms, int=overflow)
// Timer interrupt setting for main loop
//----------------------------------------------
void setupTCInt(){
  MsTimer2::set(LOOP_INTERVAL, intTimer);
}

//----------------------------------------------
// Timer INT
// Timer interrupt function
//----------------------------------------------
void intTimer(){
  bInterval = true;
}

//=====================================================================
// Loop
//=====================================================================
//---------------------------------------------------------------------
// Main loop
//---------------------------------------------------------------------
void loop(){
  loopHX711();

  //-----------------------------------------------------
  // Timer interval Loop once in 125ms
  //-----------------------------------------------------
  if (bInterval == true){
     bInterval = false;

    //--------------------------------------------
    loopCounter();                    // loop counter
    //--------------------------------------------
    // Run once in 1s
    //--------------------------------------------
    if(event1s == true){
      event1s = false;                // initialize parameter
      loopSensor();                   // sensor read
      bt_sendData();                  // Data send
    }
  }
  loopBleRcv();
}

//---------------------------------------------------------------------
// Counter
// Count the number of loops in the main loop and turn on sensor data acquisition
// and BLE transmission at 1-second intervals
//---------------------------------------------------------------------
void loopCounter(){
  iLoop1s += 1;
  //--------------------
  // 1s period
  //--------------------
  if (iLoop1s >=  8){                 // 125ms x 8 = 1s
    iLoop1s = 0;

    iSendCounter  += 1;
    if (iSendCounter >= SEND_INTERVAL){
      iSendCounter = 0;
      event1s = true;
    }
  }
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
void loopHX711(){
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;                //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();

      if (addTimes < 10){
        ++addTimes;
        totalWeight = totalWeight + i;
      }
      else {
        dataWeight = totalWeight / addTimes;
        totalWeight = 0;
        addTimes = 0;
        }

//      Serial.println(i);
//      Serial.print("total val: ");
//      Serial.println(totalWeight);
//      Serial.print("addTimes: ");
//      Serial.println(addTimes);

      newDataReady = 0;
      t = millis();

      }
    }
 
  // receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();      //tare
    else if (inByte == 'r') calibrate();            //calibrate
    else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
  }

  // check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}

//---------------------------------------------------------------------
// Sensor
// When sensor data acquisition is ON, data is acquired from each sensor
// Serial output of measured values and calculation results when console output is ON
//---------------------------------------------------------------------
void loopSensor(){

  //-------------------------
  // ADC081C027（ADC)
  // Battery leaf battery voltage acquisition
  //-------------------------
  double temp_mv;
  uint8_t adcVal1 = 0;
  uint8_t adcVal2 = 0;

  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR,2);
  adcVal1 = Wire.read();
  adcVal2 = Wire.read();

  if (adcVal1 == 0xff && adcVal2 == 0xff) {
    // If the measured value is FF, the battery leaf is not connected.
    adcVal1 = adcVal2 = 0;
  }

  // Voltage calculation :　ADC　* ((Reference voltage(3.3V)/ ADC resolution(256)) * Divided voltage ratio(2)
  temp_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  dataBatt = (float)(temp_mv / 1000);
}

//---------------------------------------------------------------------
// Send sensor data
// Convert sensor data into a string to be sent to Central and send the data to BLE Leaf.
//---------------------------------------------------------------------
void bt_sendData(){
  float value;
  char weight[7], battVolt[7];
  char sendData[40];
  uint8 sendLen;

  //-------------------------
  // Convert sensor data to strings
  // dtostrf(Number to be converted, number of characters to be converted, number of decimal places, where to store the converted characters);
  // If the number of characters to be converted is set to -, the converted characters will be left-justified; if +, they will be right-justified.
  //-------------------------

  //-------------------------
  // Weight
  //-------------------------
  value = dataWeight;
  if (value < 0){
   value = 0;
  }
  dtostrf(value,4,1,weight);

  //-------------------------
  // Battery Voltage (4Byte)
  //-------------------------
  value = dataBatt;
  if (value >= 10){
   value = 9.99;
  }
  dtostrf(value, 4, 2, battVolt);

  //-------------------------
  trim(weight);
  trim(battVolt);

  //-------------------------
  // BLE Send Data
  //-------------------------
  if( bBLEsendData == true ){     // BLE transmission
    // Format for WebBluetooth application
    sendLen = sprintf(sendData, "%04s,%04s\n", weight, battVolt);
    // Send to BLE device
    ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)sendData );
    while (ble112.checkActivity(1000));
  }
    //-------------------------
    // Serial monitor display
    //-------------------------
#ifdef SERIAL_MONITOR
/*
    // To display on multiple lines
    Serial.println("--- sensor data ---");    
    Serial.println("  Weight[kg]     = " + String(dataWeight));
    Serial.println("  Bat[V]        = " + String(dataBatt));
*/
  Serial.println("SensorData: Weight=" + String(weight) + ", Vbat=" + String(battVolt));
#endif
}
//====================================================================

//---------------------------------------
// trim
// Removing SP from a string array
//---------------------------------------
void trim(char * data){
  int i = 0, j = 0;

  while (*(data + i) != '\0'){
    if (*(data + i) != ' '){
      *(data + j) = *(data + i);
      j++;
    }
    i++;
  }
  *(data + j) = '\0';
}

//=====================================================================
// HX711
//=====================================================================
//-----------------------------------------------
// 
//-----------------------------------------------
void calibrate() {
  Serial.println("Start calibration:");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("set the tare offset.");

  boolean _resume = false;
  boolean _resume2 = false;

  while (_resume == false) {
    LoadCell.update();

    if (_resume2 == false) {
      LoadCell.tareNoDelay();
      _resume2 =true;
    }

    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

//  float newCalibrationValue = 18793;
  LoadCell.setCalFactor(newCalibrationValue);

  EEPROM.put(calVal_eepromAdress, newCalibrationValue);
  EEPROM.get(calVal_eepromAdress, newCalibrationValue);

  Serial.print("New calibration value is: ");
  Serial.println(newCalibrationValue);
  Serial.println("End calibration");
}

//-----------------------------------------------
// 
//-----------------------------------------------
void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value ");
  float newCalibrationValue =0;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;

  Serial.print("Save this value to EEPROM ");

  EEPROM.put(calVal_eepromAdress, newCalibrationValue);
  EEPROM.get(calVal_eepromAdress, newCalibrationValue);

  Serial.print("New calibration value is: ");
  Serial.println(newCalibrationValue);

  Serial.println("End change value");
}

//=====================================================================
// I2C control function
//=====================================================================
//-----------------------------------------------
//I2C Write 1 byte to the slave device
//-----------------------------------------------
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}

//-----------------------------------------------
//I2C Read 1 byte from the slave device
//-----------------------------------------------
unsigned char i2c_read_byte(int device_address, int reg_address){
  int read_data = 0;

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, 1);
  read_data = Wire.read();

  return read_data;
}

//=====================================================================
// BLE
//=====================================================================
//-----------------------------------------------
//  Setup BLE
//-----------------------------------------------
void setupBLE(){
    uint8  stLen;
    uint8 adv_data[31];

    // set up internal status handlers (these are technically optional)
    ble112.onBusy = onBusy;
    ble112.onIdle = onIdle;
    ble112.onTimeout = onTimeout;
    // ONLY enable these if you are using the <wakeup_pin> parameter in your firmware's hardware.xml file
    // BLE module must be woken up before sending any UART data

    // set up BGLib response handlers (called almost immediately after sending commands)
    // (these are also technicaly optional)

    // set up BGLib event handlers
    /* [gatt_server] */
    ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value;    /* [BGLib] */
    /* [le_connection] */
    ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;                /* [BGLib] */
    ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed;              /* [BGLib] */
    /* [system] */
    ble112.ble_evt_system_boot = my_evt_system_boot;                                /* [BGLib] */

    ble112.ble_evt_system_awake = my_evt_system_awake;
    ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;
    /*  */

    Serialble.begin(9600);

    /* setting */
    /* [set Advertising Data] */
    uint8 ad_data[21] = {
        (2),                                    // field length
        BGLIB_GAP_AD_TYPE_FLAGS,                // field type (0x01)
        (6),                                    // data
        (1),                                    // field length (1 is a temporary default value.)
        BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE    // field type (0x09)
    };
    /*  */
    size_t lenStr2 = strDeviceName.length();
    ad_data[3] = (lenStr2 + 1);                     // field length
    uint8 u8Index;
    for( u8Index=0; u8Index < lenStr2; u8Index++){
      ad_data[5 + u8Index] = strDeviceName.charAt(u8Index);
    }
    /*   */
    stLen = (5 + lenStr2);
    ble112.ble_cmd_le_gap_set_adv_data( SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data );
    while (ble112.checkActivity(1000));                 /* Receive check */

    /* interval_min :   40ms( =   64 x 0.625ms ) */
    /* interval_max : 1000ms( = 1600 x 0.625ms ) */
    ble112.ble_cmd_le_gap_set_adv_parameters( 64, 1600, 7 );    /* [BGLIB] <interval_min> <interval_max> <channel_map> */
    while (ble112.checkActivity(1000));                         /* [BGLIB] Receive check */

    /* start */
    //ble112.ble_cmd_le_gap_start_advertising(1, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);
    ble112.ble_cmd_le_gap_start_advertising( 0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE );    // index = 0
    while (ble112.checkActivity(1000));                 /* Receive check */
    /*  */
}

//-----------------------------------------
// If data is sent from the BLE, acquire the data
// and perform processing according to the acquired data.
//-----------------------------------------
void loopBleRcv( void ){
    // keep polling for new data from BLE
    ble112.checkActivity(0);                    /* Receive check */

    /*  */
    if (ble_state == BLE_STATE_STANDBY) {
        bBLEconnect = false;                    /* [BLE] connection state */
    } else if (ble_state == BLE_STATE_ADVERTISING) {
        bBLEconnect = false;                    /* [BLE] connection state */
    } else if (ble_state == BLE_STATE_CONNECTED_SLAVE) {
        /*  */
        bBLEconnect = true;                     /* [BLE] connection state */
        /*  */
    }
}

//=====================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
//=====================================================================
//-----------------------------------------------
// called when the module begins sending a command
void onBusy() {
    // turn LED on when we're busy
    //digitalWrite( D13_LED, HIGH );
}

//-----------------------------------------------
// called when the module receives a complete response or "system_boot" event
void onIdle() {
    // turn LED off when we're no longer busy
    //digitalWrite( D13_LED, LOW );
}

//-----------------------------------------------
// called when the parser does not read the expected response in the specified time limit
void onTimeout() {
    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false;                    /* [BLE] connection state */
    bBLEsendData = false;
}

//-----------------------------------------------
// called immediately before beginning UART TX of a command
void onBeforeTXCommand() {
}

//-----------------------------------------------
// called immediately after finishing UART TX
void onTXCommandComplete() {
    // allow module to return to sleep (assuming here that digital pin 5 is connected to the BLE wake-up pin)
}
/*  */

//-----------------------------------------------
void my_evt_gatt_server_attribute_value( const struct ble_msg_gatt_server_attribute_value_evt_t *msg ) {
    uint16 attribute = (uint16)msg -> attribute;
    uint16 offset = 0;
    uint8 value_len = msg -> value.len;

    uint8 value_data[20];
    String rcv_data;
    rcv_data = "";
    for (uint8_t i = 0; i < value_len; i++) {
        rcv_data += (char)(msg -> value.data[i]);
    }

#ifdef DEBUG
        Serial.print(F("###\tgatt_server_attribute_value: { "));
        Serial.print(F("connection: ")); Serial.print(msg -> connection, HEX);
        Serial.print(F(", attribute: ")); Serial.print((uint16_t)msg -> attribute, HEX);
        Serial.print(F(", att_opcode: ")); Serial.print(msg -> att_opcode, HEX);

        Serial.print(", offset: "); Serial.print((uint16_t)msg -> offset, HEX);
        Serial.print(", value_len: "); Serial.print(msg -> value.len, HEX);
        Serial.print(", value_data: "); Serial.print(rcv_data);

        Serial.println(F(" }"));
#endif

    if( rcv_data.indexOf("SND") == 0 ){
        bBLEsendData = true;
    } else if( rcv_data.indexOf("STP") == 0 ){
        bBLEsendData = false;
    } else if(rcv_data.indexOf("PLS") == 0){
    } else if(rcv_data.indexOf("MNS") == 0){
    }
}
/*  */

//-----------------------------------------------
void my_evt_le_connection_opend( const ble_msg_le_connection_opend_evt_t *msg ) {
    #ifdef DEBUG
        Serial.print(F("###\tconnection_opend: { "));
        Serial.print(F("address: "));
        // this is a "bd_addr" data type, which is a 6-byte uint8_t array
        for (uint8_t i = 0; i < 6; i++) {
            if (msg -> address.addr[i] < 16) Serial.write('0');
            Serial.print(msg -> address.addr[i], HEX);
        }
        Serial.println(" }");
    #endif
    /*  */
    ble_state = BLE_STATE_CONNECTED_SLAVE;
}
/*  */
//-----------------------------------------------
void my_evt_le_connection_closed( const struct ble_msg_le_connection_closed_evt_t *msg ) {
    #ifdef DEBUG
        Serial.print(F("###\tconnection_closed: { "));
        Serial.print(F("reason: ")); Serial.print((uint16_t)msg -> reason, HEX);
        Serial.print(F(", connection: ")); Serial.print(msg -> connection, HEX);
        Serial.println(F(" }"));
    #endif

    // after disconnection, resume advertising as discoverable/connectable (with user-defined advertisement data)
    //ble112.ble_cmd_le_gap_set_mode( LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE );
    //ble112.ble_cmd_le_gap_start_advertising(1, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);
    ble112.ble_cmd_le_gap_start_advertising( 0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE );    // index = 0
    while (ble112.checkActivity(1000));

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false;                    /* [BLE] connection state */
    bBLEsendData = false;
}
/*  */

//-----------------------------------------------
void my_evt_system_boot( const ble_msg_system_boot_evt_t *msg ) {
    #ifdef DEBUG
        Serial.print( "###\tsystem_boot: { " );
        Serial.print( "major: " ); Serial.print(msg -> major, HEX);
        Serial.print( ", minor: " ); Serial.print(msg -> minor, HEX);
        Serial.print( ", patch: " ); Serial.print(msg -> patch, HEX);
        Serial.print( ", build: " ); Serial.print(msg -> build, HEX);
        Serial.print( ", bootloader_version: " ); Serial.print( msg -> bootloader, HEX );           /*  */
        Serial.print( ", hw: " ); Serial.print( msg -> hw, HEX );
        Serial.println( " }" );
    #endif

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;
}

//-----------------------------------------------
void my_evt_system_awake(const ble_msg_system_boot_evt_t *msg ) {
   ble112.ble_cmd_system_halt( 0 );
   while (ble112.checkActivity(1000));
}

//-----------------------------------------------
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg ){
#ifdef DEBUG
  Serial.print( "###\tsystem_get_bt_address: { " );
  Serial.print( "address: " );
  for (int i = 0; i < 6 ;i++){
    Serial.print(msg->address.addr[i],HEX);
  }
  Serial.println( " }" );
#endif
  unsigned short addr = 0;
  char cAddr[30];
  addr = msg->address.addr[0] + (msg->address.addr[1] *0x100);
  sprintf(cAddr, "Device name is Leaf_A_#%05d ",addr);
  Serial.println(cAddr);
}
