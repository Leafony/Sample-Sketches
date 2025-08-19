//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : STM32
//     Processor    : STM32 MCU
//     Application  : 4-Sensors data upload via TCP
//
//     Leaf configuration
//       (1) AI01 4-Sensors 
//       (2) AP03 STM32 MCU 
//       (3) AC08 LTE-M Mary
//       (4) AV06 1.8V–5.5V
//
//    (c) 2025 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensourc .org/licenses/MIT
//
//=====================================================================

//---------------------------------------------------------------------
// libraries
//---------------------------------------------------------------------
#include "arduino_secrets.h"

#include <LpwaV4.h>                 // LTE-M

#include <Wire.h>                   // I2C
#include <Adafruit_LIS3DH.h>        // 3-axis acceleration
#include <HTS221.h>                 // humidity and temperature
#include <ClosedCube_OPT3001.h>     // Ambient Light

//-----------------------------------
// use the following STM32duino libraries version
//
// download STM32duino Low Power Ver. 1.2.2
// https://github.com/stm32duino/STM32LowPower/tree/1.2.2
//
// download STM32duino RTC Ver. 1.2.0
// https://github.com/stm32duino/STM32RTC/tree/1.2.0
//----------------------------------
#include "STM32LowPower.h"          // use STM32duino Low Power Ver. 1.2.2
#include "STM32RTC.h"               // use STM32duino RTC Ver. 1.2.0


//---------------------------------------------------------------------
// define
//---------------------------------------------------------------------
//-----------------------------------
// Select LTE carrier
//-----------------------------------
//#define LTE_CARRIER LPWA_V4_GPRS_BAND_KDDI
#define LTE_CARRIER LPWA_V4_GPRS_BAND_DOCOMO 

//-----------------------------------
// I2C address (4-sensors)
//-----------------------------------
#define LIS3DH_ADDRESS  0x19      // Accelerometer
#define OPT3001_ADDRESS 0x45      // Ambient Light


//---------------------------------------------------------------------
// Intance
//---------------------------------------------------------------------
//-----------------------------------
// LTE-M
//-----------------------------------
LpwaClient client;
GPRS gprs;
LpwaAccess lpwaAccess;
LpwaCtrl pmctrl;
LpwaModem lpwaModem;
LpwaScanner scannerNetworks;

//-----------------------------------
// 4-sensors
//-----------------------------------
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 illum;


//---------------------------------------------------------------------
// Variables
//---------------------------------------------------------------------
//-----------------------------------
// deep sleep
//-----------------------------------
bool active = false;

int curHour;
const int HOUR_START = 6;   // 24 hour
const int HOUR_STOP = 5;    // 24 hour

int cnt;
const int CNT_MAX = 100;

const uint32_t DEEPSLEEP_TIME = 600000; // 10 min. deepsleep time(msec)

//-----------------------------------
// LTE-M
//-----------------------------------
bool connected = false;

String curTime;
String signalPwrStr;
float  signalPwr;

String sendStr;

//------------------
// arduino_secrets.h
//------------------
const char GPRS_APN[] = SECRET_GPRS_APN;
const char GPRS_LOGIN[] = SECRET_GPRS_LOGIN;
const char GPRS_PASSWORD[] = SECRET_GPRS_PASSWORD;

//------------------
// URL, path and port
//------------------
//char server[] = "harvest.soracom.io";
char server[] = "unified.soracom.io";

char path[] = "/";

// int port = 80; // port 80 is the default for HTTP
// int port = 8514; // port 8514 is used for SORACOM Harvest Data
int port = 23080; // port 23080 is used for SORACOM Unified Endpoint

//-----------------------------------
// sensor data
//-----------------------------------
float dataX_g, dataY_g, dataZ_g;
float dataTemp;
float dataHumid;
float dataIllum;
float dataBat;


//---------------------------------------------------------------------
// initialization
//---------------------------------------------------------------------
//--------------------------------------------
// 4-sensors
//--------------------------------------------
void init4sensors(){

  //-------------------------------------
  // LIS3DH (accelerometer)
  //-------------------------------------
  accel.begin(LIS3DH_ADDRESS);                  // I2C address
  delay(10);

  accel.setClick(0, 0);                         // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);             // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_10_HZ);     // Data rate = 10Hz

  //-------------------------------------
  // HTS221 (Humidity and Temperature sensor)
  //-------------------------------------
  smeHumidity.begin(); 
  delay(10);

  //-------------------------------------
  // OPT3001 (Ambient Light Sensor)
  //-------------------------------------
  OPT3001_Config illumConfig;
  OPT3001_ErrorCode illumErrorConfig;

  illum.begin(OPT3001_ADDRESS);                  // I2C address

  illumConfig.RangeNumber = B1100;               // automatic full scale
  illumConfig.ConvertionTime = B1;               // convertion time = 800ms
  illumConfig.ModeOfConversionOperation = B11;   // continous conversion
  illumConfig.Latch = B0;                        // hysteresis-style

  illumErrorConfig = illum.writeConfig(illumConfig);

  if(illumErrorConfig != NO_ERROR){
    
    illumErrorConfig = illum.writeConfig(illumConfig);   //retry
  }
}


//---------------------------------------------------------------------
// Function
//---------------------------------------------------------------------
//--------------------------------------------
// LTE-M: connect to the network
//--------------------------------------------
void connectNetwork(){

  connected = false;

  while (!connected){

    if ((lpwaAccess.begin() == LPWA_READY) &&
        (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD, gprs.LTE_CARRIER) == GPRS_READY)){

      connected = true;

      Serial.println("");
      Serial.println("  -connected");
      Serial.println("");
    }
    else{

      Serial.println("");
      Serial.println("  -connecting.");
      Serial.println("");
      delay(1000);
    } 
  }

  //---------------------------
  // read informations
  //---------------------------
    //-------------
  // fetch time
  //-------------
  curTime = lpwaModem.getTime();
  curHour = curTime.substring(curTime.indexOf(',')+1, curTime.indexOf(':')).toInt();

  Serial.print("  -time: ");
  Serial.println(curTime);
  Serial.println("");

  //-------------
  // carrier name
  //-------------  
  String CA = scannerNetworks.getCurrentCarrier();

  Serial.print("  -current carrier: ");
  Serial.println(CA);
  Serial.println("");

  //-------------
  // RSSI
  //-------------
  signalPwrStr = scannerNetworks.getSignalStrength();

  if (signalPwrStr == "") {
  
    signalPwrStr = scannerNetworks.getSignalStrength();   // retry 1 time
  }

  signalPwr = signalPwrStr.toFloat()*2.0-113.0;
  Serial.println("  -signal Strength: " + String(signalPwr) + "dBm");
  Serial.println("");
}

//--------------------------------------------
// 4-sensors: read data
//--------------------------------------------
void read4sensors(){

  dataX_g   = 0.0;
  dataY_g   = 0.0;
  dataZ_g   = 0.0;
  dataTemp  = 0.0;
  dataHumid = 0.0;
  dataIllum = 0.0;
  dataBat   = 0.0;

  //---------------------------
  // acceleration
  //---------------------------
  accel.read();
  dataX_g = accel.x_g;    // X-axis
  dataY_g = accel.y_g;    // Y-axis
  dataZ_g = accel.z_g;    // Z-axis

  Serial.println("  -acceleration X: " + String(dataX_g) + " g");
  Serial.println("  -acceleration Y: " + String(dataY_g) + " g");
  Serial.println("  -acceleration Z: " + String(dataZ_g) + " g");
  Serial.println("");

  //---------------------------
  // temperature /humidity
  //---------------------------
  dataTemp = (float)smeHumidity.readTemperature();
  dataHumid = (float)smeHumidity.readHumidity();
  
  Serial.println("  -temperature: " + String(dataTemp) + " degC");
  Serial.println("  -humidity   : " + String(dataHumid) + " %");
  Serial.println("");

  //---------------------------
  // ambient light
  //---------------------------
  OPT3001 result = illum.readResult();
    
  if(result.error == NO_ERROR){

    dataIllum = result.lux;
  }

  Serial.println("  -ambient light: " + String(dataIllum) + " lux");
  Serial.println("");

  //---------------------------
  // battery voltage
  //---------------------------
  dataBat = pmctrl.getBattLevel() / 1000.0;
  Serial.println("  -battery volt: " + String(dataBat) + " V");
  Serial.println("");
}

//--------------------------------------------
// LTE-M: send data
//--------------------------------------------
void sendData(){

  //---------------------------
  // JSON data format
  //---------------------------
    sendStr =  "{\"Accel_X\":\"";
    sendStr += dataX_g;
    sendStr += "\", \"Accel_Y\":\"";
    sendStr += dataY_g;
    sendStr += "\", \"Accel_Z\":\"";
    sendStr += dataZ_g;
    sendStr += "\", \"Temperature\":\"";
    sendStr += dataTemp;
    sendStr += "\", \"Humidity\":\"";
    sendStr += dataHumid;
    sendStr += "\", \"Illumination\":\"";
    sendStr += dataIllum;
    sendStr += "\", \"Battery Level\":\"";
    sendStr += dataBat;
    sendStr += "\", \"Signal Power\":\"";
    sendStr += signalPwr;
    sendStr += "\"}";

  //---------------------------
  // send data to the soracom 
  //---------------------------
  if (client.connect(server, port)) {

    Serial.println("connected");

    client.print(sendStr);

  }
  else {

    Serial.println("connection failed");
  }
}


//---------------------------------------------------------------------
// setup
//---------------------------------------------------------------------
void setup() {

  //-----------------------------------
  // initialize UART 
  //-----------------------------------
  Serial.begin(115200);
  delay(10);

  //-----------------------------------
  // initialize STM32 power status 
  //-----------------------------------
  LowPower.begin();

  //---------------------------
  // initilaize I2C bus
  //---------------------------
  Wire.begin();

  //---------------------------
  // initilaize 4-sensors
  //---------------------------
  init4sensors();

  //---------------------------
  // LTE-M scan network
  //---------------------------
  scannerNetworks.begin();

  Serial.println("----------------------------------");
  Serial.println("initializetion finished");
  Serial.println("");
}

//---------------------------------------------------------------------
// loop
//---------------------------------------------------------------------
void loop() {

  //-------------------------------------
  // 1)LTE-M: connect to the network
  //-------------------------------------
  Serial.println("----------------------------------");
  Serial.println("connect to the network");
  Serial.println("");

  connectNetwork();

  //-------------------------------------
  // 2) read sensor data
  //-------------------------------------
  Serial.println("----------------------------------");
  Serial.println("read sensor data");
  Serial.println("");

  read4sensors();

  //-------------------------------------
  // 3) send data to the soracom
  //-------------------------------------
  Serial.println("----------------------------------");
  Serial.println("send data to the soracom server");
  Serial.println("");

  sendData();

  //---------------------------
  // 4) disconnect and power off
  //---------------------------
  Serial.println("");
  Serial.println("----------------------------------");
  Serial.println("disconnect and power off");
  Serial.println("");

  String rsp = "";
  cnt = 0;
    
  while (1) {
 
    //------------------------
    // check incoming bytes
    //------------------------
    if (client.available()) {

      char c = client.read();
      rsp += c;
    }
    
    if ((!client.available() && !client.connected()) || rsp.length() == 4 || cnt > CNT_MAX) {

      Serial.print(rsp);
      Serial.println("  -disconnecting.");
      Serial.println("");
        
      client.stop();
      delay(100);
  
      Serial.println("  -lpwaAccess end");
      Serial.println("");

      lpwaAccess.end();        
      pmctrl.powerDown(LPWA_OFF);

      //---------------------------
      // 5) deepsleep
      //---------------------------
      Serial.println("");
      Serial.println("----------------------------------");
      Serial.print("Starting deep sleep for ");
      Serial.print(DEEPSLEEP_TIME/1000);
      Serial.println(" sec");
      Serial.println("");
      delay(10);

      LowPower.deepSleep(DEEPSLEEP_TIME); // msec
        
      Serial.println("  -Woke up from deep sleep");
      Serial.println("");
        
      break;
    }

    cnt++;
  }
}

