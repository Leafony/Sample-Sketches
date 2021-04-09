//=====================================================================
//  Leafony Platform sample sketch
//     Application  : BLE 4-Sensers demo
//     Processor    : STM32L452RE (Nucleo-64/Nucleo L452RE)
//     Arduino IDE  : 1.8.13
//     STM32 Core   : Ver1.9.0
//
//     Leaf configuration
//       (1) AC02 BLE Sugar
//       (2) AI01 4-Sensors
//       (3) AP03 STM32 MCU
//       (4) AZ01 USB
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/12/08 First release
//=====================================================================
//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <SoftwareSerial.h>                 // Software UART
#include <Wire.h>                           // I2C

#include <Adafruit_LIS3DH.h>                // 3-axis accelerometer
#include <Adafruit_HTS221.h>                // humidity and temperature sensor
#include <ClosedCube_OPT3001.h>             // Ambient Light Sensor
#include "TBGLib.h"                         // BLE
#include <ST7032.h>                         // LCD


//===============================================
// BLE Unique Name (Local device name)
// Up to 16 characters (ASCII code)
//===============================================
//                     |1234567890123456|
String strDeviceName = "Leafony_AC02";

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
// IO Pin List
//-----------------------------------------------
//  D0  PA3  (UART2_RXD)
//  D1  PA2  (UART2_TXD)
//  D2  PC7  (INT0)
//  D3  PB3  (INT1)
//  D4  PB5
//  D5  PB4
//  D6  PA8
//  D7  PB12
//  D8  PA9  (UART1_TX)
//  D9  PA10 (UART1_RX)
//  D10 PB6  (SS)
//  D11 PA7  (MOSI)
//  D12 PA6  (MISO)
//  D13 PA5  (SCK)
//  D14 PB9  (SDA)
//  D15 PB8  (SCL)
//  A0  PA4
//  A1  PA0  (UART4_TX)
//  A2  PA1  (UART4_RX)
//  A3  PB0
//  A4  PC1
//  A5  PC0

//-----------------------------------------------
// IO pin name definition
// Define it according to the leaf to be connected.
//-----------------------------------------------
#define BLE_WAKEUP      PB12                // D7   PB12
#define BLE_RX          PA1                 // [A2] PA1
#define BLE_TX          PA0                 // [A1] PA0

//-----------------------------------------------
// Define constants to be used in the program
//-----------------------------------------------
//------------------------------
// I2C address
//------------------------------
#define LIS2DH_ADDRESS          0x19        // Accelerometer (SD0/SA0 pin = VCC)
#define OPT3001_ADDRESS         0x45        // Ambient Light Sensor (ADDR pin = VCC)
#define LCD_I2C_EXPANDER_ADDR   0x1A        // LCD I2C Expander
#define BATT_ADC_ADDR           0x50        // Battery ADC

//------------------------------
// Loop interval
// Timer interrupt interval (ms)
//------------------------------
#define LOOP_INTERVAL 125000                // 125000us = 125ms interval

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
// LCD
//------------------------------
  ST7032 lcd;

//------------------------------
// Sensor
//------------------------------
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;

//------------------------------
// BLE
//------------------------------
HardwareSerial Serialble(BLE_RX, BLE_TX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0 );

//---------------------------------------------------------------------
// Define variables to be used in the program
//---------------------------------------------------------------------
//------------------------------
// LCD
//------------------------------
bool dispLCD = 0;                           // Set to 1 to display on LCD.
int8_t lcdSendCount = 0;

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
// interval Timer2 interrupt
//------------------------------
volatile bool bInterval = false;

//------------------------------
// LIS2DH : Accelerometer
//------------------------------
float dataX_g, dataY_g, dataZ_g;
float dataTilt = 0;
uint8_t dataPips;

//------------------------------
// HTS221 : Humidity and Temperature sensor
//------------------------------
Adafruit_HTS221 hts;

float dataTemp = 0;
float dataHumid = 0;

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
// OPT3001 : Ambient Light Sensor
//------------------------------
float dataLight;

//------------------------------
// BLE
//------------------------------
bool bBLEconnect = false;
bool bBLEsendData = false;
volatile bool bSystemBootBle = false;

volatile uint8_t ble_state = BLE_STATE_STANDBY;
volatile uint8_t ble_encrypted = 0;         // 0 = not encrypted, otherwise = encrypted
volatile uint8_t ble_bonding = 0xFF;        // 0xFF = no bonding, otherwise = bonding handle

//------------------------------
// Battery
//------------------------------
float dataBatt = 0;

//=====================================================================
// setup
//=====================================================================
void setup(){
//  delay(1000);

  Serial.begin(115200);     // UART 115200bps
  Wire.begin();             // I2C 100kHz
#ifdef SERIAL_MONITOR
    Serial.println(F(""));
    Serial.println(F("========================================="));
    Serial.println(F("setup start"));
#endif

  if (dispLCD==1){
    i2c_write_byte(LCD_I2C_EXPANDER_ADDR, 0x03, 0xFE);
    i2c_write_byte(LCD_I2C_EXPANDER_ADDR, 0x01, 0x01);      // LCD power ON
    // LCD設定
    lcd.begin(8, 2);
    lcd.setContrast(30);
    lcd.clear();
    lcd.print("NOW");
    lcd.setCursor(0, 1);
    lcd.print("BOOTING!");
  }

  setupPort();
  setupSensor();
  setupBLE();

  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));

  setupTimerInt();                            // Timer inverval start

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
  pinMode(BLE_WAKEUP, OUTPUT);           // [D7] : BLE Wakeup/Sleep
  digitalWrite(BLE_WAKEUP, HIGH);        // BLE Wakeup
}

//=====================================================================
// Interrupt
//=====================================================================
//-----------------------------------------------
// Timer interrupt (interval=125ms, int=overflow)
// Timer interrupt setting for main loop
//-----------------------------------------------
void setupTimerInt(){
  HardwareTimer *timer2 = new HardwareTimer (TIM2);

  timer2->setOverflow(LOOP_INTERVAL, MICROSEC_FORMAT);       // 125ms
  timer2->attachInterrupt(intTimer);
  timer2->resume();
}

//---------------------------------------------------------------------
// Initial settings for each device
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
void setupSensor(){
  //----------------------------
  // LIS2DH (accelerometer)
  //----------------------------
  accel.begin(LIS2DH_ADDRESS);

  accel.setClick(0, 0);                      // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);          // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_10_HZ);  // Data rate = 10Hz

  //----------------------------
  // HTS221 (temperature / humidity)
  //----------------------------
  while (!hts.begin_I2C()) {
#ifdef SERIAL_MONITOR
    Serial.println("Failed to find HTS221");
#endif
    delay(10);
  }

  //----------------------------
  // OPT3001 (light)
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
//====================================================================
// Loop
//=====================================================================
//---------------------------------------------------------------------
// Main loop
//---------------------------------------------------------------------
void loop() {
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
      event1s = false;
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
// Sensor
// When sensor data acquisition is ON, data is acquired from each sensor
// Serial output of measured values and calculation results when console output is ON
//---------------------------------------------------------------------
void loopSensor(){
  double temp_mv;
    //-------------------------
    // LIS2DH
    // Data acquisition for 3-axis sensors
    //-------------------------
    accel.read();
    dataX_g = accel.x_g;    // X-axis
    dataY_g = accel.y_g;    // Y-axis
    dataZ_g = accel.z_g;    // Z-axis

    if(dataZ_g >= 1.0){
      dataZ_g = 1.00;
    } else if (dataZ_g <= -1.0){
      dataZ_g = -1.00;
    }

    dataTilt = acos(dataZ_g) / PI * 180;

    // Calculate the position of the dice.
    // For each eye, the sensor takes the following values
    //     X  Y  Z
    //  1  0  0  1
    //  2  0  1  0
    //  3 -1  0  0
    //  4  1  0  0
    //  5  0 -1  0
    //  6  0  0 -1
    if ((-0.5 <= dataX_g && dataX_g < 0.5) && (-0.5 <= dataY_g && dataY_g < 0.5) && (0.5 <= dataZ_g)){
      dataPips = 1;
    }
    else if ((-0.5 <= dataX_g && dataX_g < 0.5) && (0.5 <= dataY_g) && (-0.5 <= dataZ_g && dataZ_g < 0.5)){
      dataPips = 2;
    }
    else if ((dataX_g < -0.5) && (-0.5 <= dataY_g && dataY_g < 0.5) && (-0.5 <= dataZ_g && dataZ_g < 0.5)){
      dataPips = 3;
    }
    else if ((0.5 <= dataX_g) && (-0.5 <= dataY_g && dataY_g < 0.5) && (-0.5 <= dataZ_g && dataZ_g < 0.5)){
      dataPips = 4;
    }
    else if ((-0.5 <= dataX_g && dataX_g < 0.5) && (dataY_g < -0.5) && (-0.5 <= dataZ_g && dataZ_g < 0.5)){
      dataPips = 5;
    }
    else if ((-0.5 <= dataX_g && dataX_g < 0.5) && (-0.5 <= dataY_g && dataY_g < 0.5) && (dataZ_g < -0.5)){
      dataPips = 6;
    }
//    else{
//      dataPips = 0;
//    }

    //-------------------------
    // HTS221
    // Temperature and humidity sensor data acquisition
    //-------------------------
    sensors_event_t humidity;
    sensors_event_t temp;

    hts.getEvent(&humidity, &temp);             // populate temp and humidity objects with fresh data
    dataTemp = temp.temperature;
    dataHumid =humidity.relative_humidity;

    //-------------------------
    // Two-point correction for temperature and humidity
    //-------------------------
    dataTemp=TM0+(TM1-TM0)*(dataTemp-TL0)/(TL1-TL0);      // Temperature correction
    dataHumid=HM0+(HM1-HM0)*(dataHumid-HL0)/(HL1-HL0);    // Humidity correction

    //-------------------------
    // OPT3001
    // Illuminance sensor data acquisition
    //-------------------------
    OPT3001 result = light.readResult();

    if(result.error == NO_ERROR){
      dataLight = result.lux;
    }

  //-------------------------
  // ADC081C027（ADC)
  // Battery leaf battery voltage acquisition
  //-------------------------
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
  char temp[7], humid[7], light[7], tilt[7],battVolt[7], pips[7];
  char sendData[40];
  uint8 sendLen;

  //-------------------------
  // Convert sensor data to strings
  // dtostrf(Number to be converted, number of characters to be converted, number of decimal places, where to store the converted characters);
  // If the number of characters to be converted is set to -, the converted characters will be left-justified; if +, they will be right-justified.
  //-------------------------
  //-------------------------
  // Temperature (4Byte)
  //-------------------------
  value = dataTemp;
  if(value >= 100){
    value = 99.9;
  }
  else if(value <= -10){
    value = -9.9;
  }
  dtostrf(value,4,1,temp);

  //-------------------------
  // Humidity (4Byte)
  //-------------------------
  value = dataHumid;
  dtostrf(value,4,1,humid);

  //-------------------------
  // Ambient Light (5Byte)
  //-------------------------
  value = dataLight;
  if(value >= 100000){
    value = 99999;
  }
  dtostrf(value,5,0,light);

  //-------------------------
  // Tilt (4Byte)
  //-------------------------
  value = dataTilt;
  if(value < 3){
    value = 0;
  }
  dtostrf(value,4,0,tilt);

  //-------------------------
  // dice
  //-------------------------
 value = dataPips;
  if (value > 6){
    value = 0;
  }
  dtostrf(value,4,0,pips);
  
  //-------------------------
  // Battery Voltage (4Byte)
  //-------------------------
  value = dataBatt;

  if (value >= 10){
   value = 9.99;
  }
  dtostrf(value, 4, 2, battVolt);

  //-------------------------
  trim(temp);
  trim(humid);
  trim(light);
  trim(tilt);
  trim(battVolt);
  trim(pips);

  lcd.clear();

  if (dispLCD==1){
    switch (lcdSendCount){
      case 0:                 // BLE not connected
        lcd.print("Waiting");
        lcd.setCursor(0, 1);
        lcd.print("connect");
        break;
      case 1:                 // Tmp XX.X [degC]
        lcd.print("Temp");
        lcd.setCursor(0, 1);
        lcd.print( String(temp) +" C");
        break;
      case 2:                 // Hum xx.x [%]
        lcd.print("Humidity");
        lcd.setCursor(0, 1);
        lcd.print( String(humid) +" %");
        break;
      case 3:                 // Lum XXXXX [lx]
        lcd.print("Luminous");
        lcd.setCursor(0, 1);
        lcd.print( String(light) +" lx");
        break;
      case 4:                 // Ang XXXX [arc deg]
        lcd.print("Angle");
        lcd.setCursor(0, 1);
        lcd.print( String(tilt) +" deg");
        break;
      case 5:                 // Bat X.XX [V]
        lcd.print("Battery");
        lcd.setCursor(0, 1);
        lcd.print( String(battVolt) +" V");
        break;
      default:
        break;
    }
    if (lcdSendCount < 5){
      lcdSendCount++;
    }
    else{
        if( bBLEsendData == true ){                           // Start from 1 during BLE transmission.
          lcdSendCount = 1;
        }
        else{
          lcdSendCount = 0;
        }
    }
  }

  //-------------------------
  // BLE Send Data
  //-------------------------
  if( bBLEsendData == true ){                                 // BLE transmission
    // Format for WebBluetooth application
    sendLen = sprintf(sendData, "%04s,%04s,%04s,%04s,%04s,%01s\n", temp, humid, light, tilt, battVolt, pips);
    // Send to BLE device
    ble112.ble_cmd_gatt_server_send_characteristic_notification( 1, 0x000C, sendLen, (const uint8 *)sendData );
    while (ble112.checkActivity(1000));
  }

    //-------------------------
    // Serial monitor display
    //-------------------------
#ifdef SERIAL_MONITOR
// To display on multiple lines
/*
  Serial.println("--- sensor data ---");    
  Serial.println("  Tmp[degC]     = " + String(dataTemp));
  Serial.println("  Hum[%]        = " + String(dataHumid));
  Serial.println("  Lum[lx]       = " + String(dataLight));
  Serial.println("  Ang[arc deg]  = " + String(dataTilt));
  Serial.println("  Bat[V]        = " + String(dataBatt));
*/
// To display on a single line
  Serial.println("SensorData: Temp=" + String(temp) + ", Humid=" + String(humid) + ", Light=" + String(light) + ", Tilt=" + String(tilt) + ", Vbat=" + String(battVolt) + ", Dice=" + String(pips));
#endif
}
//====================================================================

//==============================================
// Interrupt
//==============================================

//----------------------------------------------
// Timer INT
// Timer interrupt function
//----------------------------------------------
//void intTimer(HardwareTimer*){      // STM32 Core 1.7.0
void intTimer(void){                  // STM32 Core 1.9.0
  bInterval = true;
}

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
// Read 1 byte from the slave device
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
    ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;                    /* [BGLib] */
    ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed;                  /* [BGLib] */
    /* [system] */
    ble112.ble_evt_system_boot = my_evt_system_boot;                                    /* [BGLib] */

    ble112.ble_evt_system_awake = my_evt_system_awake;
    ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;

    uint8_t tm=0;
    Serialble.begin(9600);
    while (!Serialble && tm <150){                              // Wait for Serial to start Timeout 1.5s
      tm++;
      delay(10);
    }

    tm=0;
    while (!bSystemBootBle && tm <150){                         // Waiting for BLE to start
      ble112.checkActivity(100);
      tm++;
      delay(10);
    }

    /* setting */
    /* [set Advertising Data] */
    uint8 ad_data[21] = {
        (2),                                                    // field length
        BGLIB_GAP_AD_TYPE_FLAGS,                                // field type (0x01)
        (6),                                                    // data
        (1),                                                    // field length (1は仮の初期値)
        BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE                    // field type (0x09)
    };

    /*  */
    size_t lenStr2 = strDeviceName.length();

    ad_data[3] = (lenStr2 + 1);                                 // field length
    uint8 u8Index;
    for( u8Index=0; u8Index < lenStr2; u8Index++){
      ad_data[5 + u8Index] = strDeviceName.charAt(u8Index);
    }

    /*   */
    stLen = (5 + lenStr2);

    //ble112.ble_cmd_le_gap_bt5_set_adv_data(0,SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);
    ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, ad_data);

    while (ble112.checkActivity(1000));                         /* Receive check */
    delay(20);

    /* interval_min :   40ms( =   64 x 0.625ms ) */
    //ble112.ble_cmd_le_gap_bt5_set_adv_parameters( 0, 64, 1600, 7, 0 );/* [BGLIB] <handle> <interval_min> <interval_max> <channel_map> <report_scan>*/
    /* interval_max : 1000ms( = 1600 x 0.625ms ) */
    ble112.ble_cmd_le_gap_set_adv_parameters( 64, 1600, 7 );    /* [BGLIB] <interval_min> <interval_max> <channel_map> */

    while (ble112.checkActivity(1000));                         /* [BGLIB] Receive check */

    /* start */
//    ble112.ble_cmd_le_gap_bt5_set_mode(0,LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE,0,2);
//    ble112.ble_cmd_le_gap_set_mode(LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE);
//    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);     // index = 0
    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);                // index = 0
    while (ble112.checkActivity(1000));                         /* Receive check */
    /*  */
}

//-----------------------------------------
// If data is sent from the BLE, acquire the data
// and perform processing according to the acquired data.
//-----------------------------------------
void loopBleRcv(void){
    // keep polling for new data from BLE
    ble112.checkActivity(0);                                    /* Receive check */

    /*  */
    if (ble_state == BLE_STATE_STANDBY) {
        bBLEconnect = false;                                    /* [BLE] connection state */
    } else if (ble_state == BLE_STATE_ADVERTISING) {
        bBLEconnect = false;                                    /* [BLE] connection state */
    } else if (ble_state == BLE_STATE_CONNECTED_SLAVE) {
        /*  */
        bBLEconnect = true;                                     /* [BLE] connection state */
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
    bBLEconnect = false;                                        /* [BLE] connection state */
    bBLEsendData = false;
#ifdef DEBUG
     Serial.println(F("on time out"));
#endif
}

//-----------------------------------------------
// called immediately before beginning UART TX of a command
void onBeforeTXCommand() {
}

//-----------------------------------------------
// called immediately after finishing UART TX
void onTXCommandComplete() {
    // allow module to return to sleep (assuming here that digital pin 5 is connected to the BLE wake-up pin)
#ifdef DEBUG
    Serial.println(F("onTXCommandComplete"));
#endif
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
        Serial.print(", address_type: "); Serial.print(msg -> address_type, HEX);
        Serial.print(", master: "); Serial.print(msg -> master, HEX);
        Serial.print(", connection: "); Serial.print(msg -> connection, HEX);
        Serial.print(", bonding: "); Serial.print(msg -> bonding, HEX);
        Serial.print(", advertiser: "); Serial.print(msg -> advertiser, HEX);
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
//    ble112.ble_cmd_le_gap_start_advertising(1, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);
//    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_UNDIRECTED_CONNECTABLE);              // index = 0
//    ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE);   // index = 0
//     ble112.ble_cmd_le_gap_set_mode(LE_GAP_GENERAL_DISCOVERABLE, LE_GAP_UNDIRECTED_CONNECTABLE );
     ble112.ble_cmd_le_gap_set_mode(LE_GAP_USER_DATA,LE_GAP_UNDIRECTED_CONNECTABLE); 

    while (ble112.checkActivity(1000));

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
    /*  */
    bBLEconnect = false;                                        /* [BLE] connection state */
    bBLEsendData = false;
}
/*  */

//-----------------------------------------------
void my_evt_system_boot( const ble_msg_system_boot_evt_t *msg ){
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

     bSystemBootBle = true;

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;
}

//-----------------------------------------------
void my_evt_system_awake(void){
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
