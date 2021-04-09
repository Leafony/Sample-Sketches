//=====================================================================
//  Leafony Platform sample sketch
//     Application  : BLE 4-Sensers demo
//     Processor    : ATmega328P (3.3V /8MHz)
//     Confirmed in Arduino IDE 1.8.7
//
//     Leaf configuration
//       (1) AC02 BLE Sugar
//       (2) AI01 4-Sensors
//       (3) (AI04 LCD)
//       (4) AP01 AVR MCU
//       (5) AZ01 USB
//
//    (c)2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2019/08/29 First release
//      Rev.01 2020/07/29 Modification of appearance such as deletion of unnecessary parts
//=====================================================================
//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <MsTimer2.h>                       // Timer
#include <SoftwareSerial.h>                 // Software UART
#include <Wire.h>                           // I2C

#include <Adafruit_LIS3DH.h>                // 3-axis accelerometer
#include <HTS221.h>                         // humidity and temperature sensor
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
// IO pin name definition
// Define it according to the leaf to be connected.
//-----------------------------------------------
#define D0              0                   // PD0  (RXD)
#define D1              1                   // PD1  (TXD)
#define D2              2                   // PD2  (INT0)
#define D3              3                   // PD3  (INT1)
#define D4              4                   // PD4
#define D5              5                   // PD5
#define D6              6                   // PD6
#define D7_BLE_WAKEUP   7                   // PD7
#define D8              8                   // PB0  (S-UART2_RX)
#define D9              9                   // PB1  (S-UART2_TX)
#define D10             10                  // PB2  (SS)
#define D11             11                  // PB3  (MOSI)
#define D12             12                  // PB4  (MISO)
#define D13_LED         13                  // PB5  (SCK/LED)

#define D14             14                  // [A0] PC0
#define D15_BLE_TX      15                  // [A1] PC1
#define D16_BLE_RX      16                  // [A2] PC2
#define D17             17                  // [A3] PC3

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
SoftwareSerial Serialble(D16_BLE_RX, D15_BLE_TX);
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
// Interval Timer interrupt
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

//------------------------------
// LED
//------------------------------
uint8_t iLed = 0;
volatile uint8_t iToggle = 8;
bool bToggle = 0;

//=====================================================================
// setup
//=====================================================================
void setup() {
  delay(500);

  Serial.begin(115200);     // UART 115200bps
  Wire.begin();             // I2C 100kHz
#ifdef SERIAL_MONITOR
    Serial.println(F("========================================="));
    Serial.println(F("setup start"));
#endif

  if (dispLCD==1){
    i2c_write_byte(LCD_I2C_EXPANDER_ADDR, 0x03, 0xFE);
    i2c_write_byte(LCD_I2C_EXPANDER_ADDR, 0x01, 0x01);      // LCD power ON
    // LCD settings
    lcd.begin(8, 2);
    lcd.setContrast(30);
    lcd.clear();

    lcd.print("NOW");
    lcd.setCursor(0, 1);
    lcd.print("BOOTING!");
  }

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

  MsTimer2::start();                        // Timer inverval start
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
  pinMode(D2, INPUT);                       // PD2 : digital 2 = PIR/LCD-SW interrupt
  pinMode(D3, INPUT);                       // PD3 : digital 3 = sensor interrupt
  pinMode(D4, INPUT);                       // PD4 : digital 4 = not used
  pinMode(D5, INPUT);                       // PD5 : digital 5 = not used
  pinMode(D6, INPUT);                       // PD6 : digital 6 = not used

  pinMode(D7_BLE_WAKEUP, OUTPUT);           // PD7 : digital 7 = BLE Wakeup/Sleep
  digitalWrite(D7_BLE_WAKEUP, HIGH);        // BLE Wakeup

  pinMode(D8, INPUT);                       // PB0 : digital 8 = not used
  pinMode(D9, INPUT);                       // PB1 : digital 9 = not used

  pinMode(D10, INPUT);                      // PB2 : digital 10 = not used
  pinMode(D11, INPUT);                      // PB3 : digital 11 = not used
  pinMode(D12, INPUT);                      // PB4 : digital 12 = not used

  pinMode(D13_LED, OUTPUT);                 // PB5 : digital 13 = LED
  digitalWrite(D13_LED, LOW);               // LED off

  pinMode(D14, INPUT);                      // PC0 : digital 14 = not used
  pinMode(D17, INPUT);                      // PC3 : digital 17 = not used
}

//---------------------------------------------------------------------
// Initial settings for each device
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
void setupSensor(){
  //----------------------------
  // LIS2DH : Accelerometer
  //----------------------------
  accel.begin(LIS2DH_ADDRESS);

  accel.setClick(0, 0);                             // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);                 // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_10_HZ);         // Data rate = 10Hz

  //----------------------------
  // HTS221 : Humidity and Temperature sensor
  //----------------------------
  smeHumidity.begin();

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
    // LED
    //--------------------------------------------
    if(bBLEsendData == true){
      iLed += 1;
      if(iLed >= iToggle){
        iLed = 0;
        digitalWrite(D13_LED, bToggle);
        bToggle = !bToggle;
      }
    } else{
      digitalWrite(D13_LED, LOW);
      iLed = 0;
    }
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
    dataTemp = (float)smeHumidity.readTemperature();  // Temperature
    dataHumid = (float)smeHumidity.readHumidity();    // Humidity

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
        if( bBLEsendData == true ){     // Start from 1 during BLE transmission.
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
  if( bBLEsendData == true ){     // BLE transmission
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
/*
    // To display on multiple lines
    Serial.println("--- sensor data ---");    
    Serial.println("  Tmp[degC]     = " + String(dataTemp));
    Serial.println("  Hum[%]        = " + String(dataHumid));
    Serial.println("  Lum[lx]       = " + String(dataLight));
    Serial.println("  Ang[arc deg]  = " + String(dataTilt));
    Serial.println("  Bat[V]        = " + String(dataBatt));
*/
  Serial.println("SensorData: Temp=" + String(temp) + ", Humid=" + String(humid) + ", Light=" + String(light) + ", Tilt=" + String(tilt) + ", Vbat=" + String(battVolt) + ", Dice=" + String(pips));
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
        iToggle = 8;
    } else if( rcv_data.indexOf("STP") == 0 ){
        bBLEsendData = false;
    } else if(rcv_data.indexOf("PLS") == 0){
      if(iToggle < 16){
        iToggle += 2;
      }
    } else if(rcv_data.indexOf("MNS") == 0){
      if(iToggle > 2){
        iToggle -= 2;
      }
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
