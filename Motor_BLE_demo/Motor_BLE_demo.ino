//=====================================================================
//  Leafony Platform sample sketch
//     Application  : Motor BLE demo
//     Processor    : ATmega328P (3.3V/8MHz)
//     Confirmed in Arduino IDE 1.8.13
//
//     Leaf configuration
//       (1) AC02 BLE Sugar
//       (2) AI01 4-Sensors
//       (4) AP01 AVR MCU
//       (5) AZ01 USB
//       (6) Motor
//
//    (c)2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/11/06 First release
//=====================================================================
//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <MsTimer2.h>                       // Timer
#include <SoftwareSerial.h>                 // Software UART
#include <Wire.h>                           // I2C

#include "TBGLib.h"                         // BLE
#include <Adafruit_HTS221.h>                // humidity and temperature sensor
#include <ClosedCube_OPT3001.h>             // Ambient Light Sensor

//===============================================
// BLE Unique Name (Local device name)
// Up to 16 characters (ASCII code)
//===============================================
//                     |1234567890123456|
String strDeviceName = "Motor_AC02";

//===============================================
// Output to serial monitor
//      #define SERIAL_MONITOR = With output
//    //#define SERIAL_MONITOR = Without output (Comment out)
//===============================================
//#define SERIAL_MONITOR

//===============================================
// Debug output to serial monitor
//      #define DEBUG = With output
//    //#define DEBUG = Without output (Comment out)
//===============================================
//#define DEBUG

//-----------------------------------------------
// Setting the transmission interval
//  SEND_INTERVAL  : transmission interval (Set the interval for sending sensor data in 1 second increments.)
//-----------------------------------------------
#define SEND_INTERVAL   (1)             // 1s

//-----------------------------------------------
// IO pin name definition
// Define it according to the leaf to be connected.
//-----------------------------------------------
#define VM_ON           3                   // Motor power control
#define A_IN1           12                  // Rotation control 1
#define A_IN2           4                   // Rotation control 2
#define A_PWM           5                   // Speed control PWM

#define D7_BLE_WAKEUP   7                   // PD7
#define D13_LED         13                  // PB5  (SCK/LED)

#define rdMoisture      A0                  // Soil moisture sensor input
#define D15_BLE_TX      15                  // [A1] PC1
#define D16_BLE_RX      16                  // [A2] PC2

//-----------------------------------------------
// Define constants to be used in the program
//-----------------------------------------------
//------------------------------
// I2C address
//------------------------------
#define OPT3001_ADDRESS         0x45        // Ambient Light Sensor (ADDR pin = VCC)
#define BATT_ADC_ADDR           0x50        // Battery ADC

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
// BLE
//------------------------------
SoftwareSerial Serialble(D16_BLE_RX, D15_BLE_TX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0 );

//------------------------------
// Sensor
//------------------------------
ClosedCube_OPT3001 light;

//---------------------------------------------------------------------
// Define variables to be used in the program
//---------------------------------------------------------------------
String receiveData = "MNS";                   // MNS:Motor stop PLS:Motor start

//------------------------------
// Data for serial monitor display
//------------------------------
#ifdef SERIAL_MONITOR
String titleData1 = "Moi";
String titleData2 = "Tmp";
String titleData3 = "Hum";
String titleData4 = "Lum";
String titleData5 = "Bat";

String unitData1 = "%";
String unitData2 = "degC";
String unitData3 = "%RH";
String unitData4 = "lx";
String unitData5 = "V";
#endif

//------------------------------
// Sensor data
//------------------------------
float data1 = 0;          // Moisture
float data2 = 0;          // Temperature
float data3 = 0;          // Humidity
float data4 = 0;          // Illuminance
float data5 = 0;          // Battery Voltage

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

//---------------------------
// HTS221 : Temperature/Humidity
//---------------------------
Adafruit_HTS221 hts;

//--------------------
// Data for two-point correction
//--------------------
// Temperature correction data 0
float TL0 = 25.0;     // 4-Sensors Temperature measurement value
float TM0 = 25.0;     // Thermometer and other measurements value
// Temperature correction data 1
float TL1 = 40.0;     // 4-Sensors Temperature measurement value
float TM1 = 40.0;     // Thermometer and other measurements value

// Humidity correction data 0
float HL0 = 60.0;     // 4-Sensors Humidity measurement value
float HM0 = 60.0;     // Hygrometer and other measurements value
// Humidity correction data 1
float HL1 = 80.0;     // 4-Sensors Humidity measurement value
float HM1 = 80.0;     // Hygrometer and other measurements value

//------------------------------
// BLE
//------------------------------
bool bBLEconnect = false;
bool bBLEsendData = false;
volatile bool bSystemBootBle = false;

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0;         // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF;        // 0xFF = no bonding, otherwise = bonding handle

//=====================================================================
// setup
//=====================================================================
void setup() {
  Wire.begin();             // I2C 100kHz
  Serial.begin(115200);     // UART 115200bps
#ifdef SERIAL_MONITOR
    Serial.println(F("========================================="));
    Serial.println(F("setup start"));
#endif

  setupPort();
  delay(10);

  noInterrupts();
  setupTCInt();
  interrupts();

  setupSensor();
  setupBLE();

  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));
  delay(1000);

  MsTimer2::start();                        // Timer2 inverval start
#ifdef SERIAL_MONITOR
    Serial.println(F(""));
    Serial.println("=========================================");
    Serial.println(F("loop start"));
    Serial.println(F(""));
#endif
}

//-----------------------------------------------
// IO pin input/output settings
// Configure the settings according to the leaf to be connected.
//-----------------------------------------------
void setupPort(){
  pinMode(VM_ON, OUTPUT);                   // Motor power control
  digitalWrite(VM_ON, LOW);                 // Motor power off

  // ch Aの制御用ピン設定
  pinMode(A_IN1, OUTPUT);                   // Rotation control 1
  pinMode(A_IN2, OUTPUT);                   // Rotation control 2
  pinMode(A_PWM, OUTPUT);                   // Speed control by PWM (0-255)

  pinMode(D7_BLE_WAKEUP, OUTPUT);           // D7  PD7 : BLE Wakeup/Sleep
  digitalWrite(D7_BLE_WAKEUP, HIGH);        // BLE Wakeup

  pinMode(D13_LED, OUTPUT);                 // D13 PB5 : LED
  digitalWrite(D13_LED, LOW);               // LED off

  pinMode(rdMoisture, INPUT);               // A0 Soil moisture sensor
}

//---------------------------------------------------------------------
// Initial settings for each device
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
void setupSensor(){
  //----------------------------
  // HTS221 : Humidity and Temperature sensor
  //----------------------------
  while (!hts.begin_I2C()) {
#ifdef DEBUG
    Serial.println("Failed to find HTS221");
#endif
    delay(10);
  }
  delay(10);

  //----------------------------
  // OPT3001 : Ambient Light Sensor
  //----------------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  light.begin(OPT3001_ADDRESS);

  newConfig.RangeNumber = B1100;                    // automatic full scale
  newConfig.ConvertionTime = B1;                    // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;        // continous conversion
  newConfig.Latch = B0;                             // hysteresis-style

  errorConfig = light.writeConfig(newConfig);

  if(errorConfig != NO_ERROR){
    errorConfig = light.writeConfig(newConfig);     // retry
  }
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

      if(receiveData == "PLS"){
        digitalWrite(VM_ON, HIGH);          // Motor power On
        delay(10);
        chA_CW();                           // Motor A: Forward rotation
        analogWrite(A_PWM, 255);            // Motor A: full speed
      }
      else if(receiveData == "MNS"){
        analogWrite(A_PWM, 0);              // Motor A: 0
        chA_stop();
        delay(10);
        digitalWrite(VM_ON, LOW);          // Motor power Off
      }
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
// Sensor
// When sensor data acquisition is ON, data is acquired from each sensor
// Serial output of measured values and calculation results when console output is ON
//---------------------------------------------------------------------
void loopSensor(){
    //----------------------------
    // Moisture
    //----------------------------
    data1 = analogRead(rdMoisture);

    //-------------------------
    // HTS221
    // Temperature and humidity sensor data acquisition
    //-------------------------
    sensors_event_t humidity;
    sensors_event_t temp;
    hts.getEvent(&humidity, &temp);               // populate temp and humidity objects with fresh data
    data2 = temp.temperature;                     // Temperature
    data3 =humidity.relative_humidity;            // Humidity

    //-------------------------
    // Two-point correction for temperature and humidity
    //-------------------------
    data2=TM0+(TM1-TM0)*(data2-TL0)/(TL1-TL0);    // Temperature correction
    data3=HM0+(HM1-HM0)*(data3-HL0)/(HL1-HL0);    // Humidity correction

    //-------------------------
    // OPT3001
    // Illuminance sensor data acquisition
    //-------------------------
    OPT3001 result = light.readResult();

    if(result.error == NO_ERROR){
      data4 = result.lux;
    }

  //-------------------------
  // ADC081C027（ADC)
  // Battery leaf battery voltage acquisition
  //-------------------------
  double Vbat_mv;
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
  Vbat_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  data5 = (float)(Vbat_mv / 1000);
}

//---------------------------------------------------------------------
// Send sensor data
// Convert sensor data into a string to be sent to Central and send the data to BLE Leaf.
//---------------------------------------------------------------------
void bt_sendData(){
  float value;
  char sendData1[7], sendData2[7], sendData3[7], sendData4[7],sendData5[7];
  char sendData[40];
  uint8 sendLen;

  //-------------------------
  // Convert sensor data to strings
  // dtostrf(Number to be converted, number of characters to be converted, number of decimal places, where to store the converted characters);
  // If the number of characters to be converted is set to -, the converted characters will be left-justified; if +, they will be right-justified.
  //-------------------------
  //-------------------------
  // Data1 Moisture
  //-------------------------
  value = data1;
//  if(value >= 100){
//    value = 99.9;
//  }
  dtostrf(value,4,0,sendData1);

  //-------------------------
  // Data2 Temperature (4Byte)
  //-------------------------
  value = data2;
  if(value >= 100){
    value = 99.9;
  }
  else if(value <= -10){
    value = -9.9;
  }
  dtostrf(value,4,1,sendData2);

  //-------------------------
  // Data3 Humidity (4Byte)
  //-------------------------
  value = data3;
  dtostrf(value,4,1,sendData3);

  //-------------------------
  // Data4 Ambient Light (5Byte)
  //-------------------------
  value = data4;
  if(value >= 100000){
    value = 99999;
  }
  dtostrf(value,5,0,sendData4);

  //-------------------------
  // Data5 Battery Voltage (4Byte)
  //-------------------------
  value = data5;
  if (value >= 10){
   value = 9.99;
  }
  dtostrf(value, 4, 1, sendData5);

  //-------------------------
  trim(sendData1);
  trim(sendData2);
  trim(sendData3);
  trim(sendData4);
  trim(sendData5);

  //-------------------------
  // BLE Send Data
  //-------------------------
  if( bBLEsendData == true ){
    // Format for WebBluetooth application
    sendLen = sprintf(sendData, "%04s,%04s,%04s,%04s,%04s\n", sendData1, sendData2, sendData3, sendData4, sendData5);
    // Send to BLE device
    ble112.ble_cmd_gatt_server_send_characteristic_notification(1, 0x000C, sendLen, (const uint8 *)sendData);

    while (ble112.checkActivity(1000));
  }
    //-------------------------
    // Serial monitor display
    //-------------------------
#ifdef SERIAL_MONITOR
    Serial.println("--- sensor data ---");    
    Serial.println(titleData1 + " =" + "\t" + String(data1) + " [" + unitData1 +"]");
    Serial.println(titleData2 + " =" + "\t" + String(data2) + " [" + unitData2 +"]");
    Serial.println(titleData3 + " =" + "\t" + String(data3) + " [" + unitData3 +"]");
    Serial.println(titleData4 + " =" + "\t" + String(data4) + " [" + unitData4 +"]");
    Serial.println(titleData5 + " =" + "\t" + String(data5) + " [" + unitData5 +"]");

/*
    // To display in one line
    Serial.print("###\tSensor data: { ");
    Serial.print(titleData1 + "=" + String(data1) + ", ");
    Serial.print(titleData2 + "=" + String(data2) + ", ");
    Serial.print(titleData3 + "=" + String(data3) + ", ");
    Serial.print(titleData4 + "=" + String(data4) + ", ");
    Serial.print(titleData5 + "=" + String(data5) + ", ");
*/
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
// Motor control function
//=====================================================================
//-----------------------------------------------
// ch A forward
//-----------------------------------------------
void chA_CW(){
  digitalWrite(A_IN1, HIGH);
  digitalWrite(A_IN2, LOW);
}

//-----------------------------------------------
// ch A reverse
//-----------------------------------------------
void chA_CCW(){
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, HIGH);
}

//-----------------------------------------------
// ch A brake
//-----------------------------------------------
void chA_brake(){
  digitalWrite(A_IN1, HIGH);
  digitalWrite(A_IN2, HIGH);
}

//-----------------------------------------------
// ch A Stop (Free)
//-----------------------------------------------
void chA_stop(){
  digitalWrite(A_IN1, LOW);
  digitalWrite(A_IN2, LOW);
}

//=====================================================================
// I2C control function
//=====================================================================
//-----------------------------------------------
// I2C Write 1 byte to the slave device
//-----------------------------------------------
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}

//-----------------------------------------------
// I2C Read 1 byte from the slave device
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

    uint8_t tm=0;
    Serialble.begin(9600);
    while (!Serialble && tm <150){                                // Waiting for Serial to start Timeout 1.5s
      tm++;
      delay(10);
    }

    tm=0;
    while (!bSystemBootBle && tm <150){                           // Waiting for BLE to start
      ble112.checkActivity(100);
      tm++;
      delay(10);
    }

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
    digitalWrite( D13_LED, HIGH );
}

//-----------------------------------------------
// called when the module receives a complete response or "system_boot" event
void onIdle() {
    // turn LED off when we're no longer busy
    digitalWrite( D13_LED, LOW );
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
      receiveData = "PLS";
    } else if(rcv_data.indexOf("MNS") == 0){
      receiveData = "MNS";
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
#if 0
        Serial.print(", address_type: "); Serial.print(msg -> address_type, HEX);
        Serial.print(", master: "); Serial.print(msg -> master, HEX);
        Serial.print(", connection: "); Serial.print(msg -> connection, HEX);
        Serial.print(", bonding: "); Serial.print(msg -> bonding, HEX);
        Serial.print(", advertiser: "); Serial.print(msg -> advertiser, HEX);
#endif
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
#if 0
    #ifdef DEBUG
        Serial.print( "###\tsystem_boot: { " );
        Serial.print( "major: " ); Serial.print(msg -> major, HEX);
        Serial.print( ", minor: " ); Serial.print(msg -> minor, HEX);
        Serial.print( ", patch: " ); Serial.print(msg -> patch, HEX);
        Serial.print( ", build: " ); Serial.print(msg -> build, HEX);
    //    SerialUSB.print(", ll_version: "); Serial.print(msg -> ll_version, HEX);
        Serial.print( ", bootloader_version: " ); Serial.print( msg -> bootloader, HEX );           /*  */
    //    Serial.print(", protocol_version: "); Serial.print(msg -> protocol_version, HEX);
        Serial.print( ", hw: " ); Serial.print( msg -> hw, HEX );
        Serial.println( " }" );
    #endif
#endif

     bSystemBootBle = true;

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
#ifdef SERIAL_MONITOR
  unsigned short addr = 0;
  char cAddr[30];
  addr = msg->address.addr[0] + (msg->address.addr[1] *0x100);
  sprintf(cAddr, "Device name is Leaf_A_#%05d ",addr);
  Serial.println(cAddr);
#endif
}