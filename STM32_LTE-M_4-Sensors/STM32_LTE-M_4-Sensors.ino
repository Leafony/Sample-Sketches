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
//    https://opensource.org/licenses/MIT
//
//    Rev.01 2025/9/1  改行
//=====================================================================
/*
  Web client

 This sketch connects to a website through a LPWA board. Specifically,
 this example downloads the URL "http://www.example.org/" and
 prints it to the Serial monitor.

 Circuit:
 * LPWA board
 * SIM card with a data plan

 created 8 Mar 2012
 by Tom Igoe
*/

// libraries
#include <LpwaV4.h>

#include "arduino_secrets.h"
#include "STM32LowPower.h"
#include <Wire.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>

#define OPT3001_ADDRESS 0x45      // illuminance sensor ADDR pin = VCC

#define SENSOR4_ON  // comment out this code if 4-sensor leaf is removed
// #define SEND_BINARY  // comment out this code if the sending data format is json

// Select LTE CARRIER
// #define LTE_CARRIER LPWA_V4_GPRS_BAND_KDDI
#define LTE_CARRIER LPWA_V4_GPRS_BAND_DOCOMO 

/* Prototype function */
static void restartSequenceWhenModemError(unsigned long msDelay);

const uint32_t DEEPSLEEP_TIME = 600000; // deepsleep time during active time (msec)

const int HOUR_START = 6;  // 24 hour
const int HOUR_STOP = 5; // 24 hour
const int CNT_MAX = 100;

// Please enter your sensitive data in the Secret tab or arduino_secrets.h
// APN data
const char GPRS_APN[] = SECRET_GPRS_APN;
const char GPRS_LOGIN[] = SECRET_GPRS_LOGIN;
const char GPRS_PASSWORD[] = SECRET_GPRS_PASSWORD;

// initialize the library instance
LpwaClient client;
GPRS gprs;
LpwaAccess lpwaAccess;
LpwaCtrl pmctrl;
LpwaModem lpwaModem;
LpwaScanner scannerNetworks;

// URL, path and port (for example: example.org)
//char server[] = "harvest.soracom.io";
char server[] = "unified.soracom.io";
char path[] = "/";
// int port = 80; // port 80 is the default for HTTP
// int port = 8514; // port 8514 is used for SORACOM Harvest Data
int port = 23080; // port 23080 is used for SORACOM Unified Endpoint

#ifdef SENSOR4_ON
//---------------------------
// Data for two-point correction
//---------------------------
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

ClosedCube_OPT3001 illum;
#endif

static void restartSequenceWhenModemError(unsigned long msDelay) {
  lpwaAccess.end();
  pmctrl.powerDown(LPWA_OFF);
  LowPower.deepSleep(msDelay);
  return;
}

void setup() {
  // initialize serial communications and wait for port to open:
  Serial.begin(115200);
#ifdef USBD_USE_CDC
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif //  USBD_USE_CDC

  LowPower.begin();

#ifdef SENSOR4_ON
  //---------------------------
  // 4-Sensor setup
  //---------------------------
  Wire.begin();
  // initialize i2c communication with HTS221:
  smeHumidity.begin();
  delay(10);

  OPT3001_Config illumConfig;
  OPT3001_ErrorCode illumErrorConfig;

  illum.begin(OPT3001_ADDRESS);

  illumConfig.RangeNumber = B1100;               // automatic full scale
  illumConfig.ConvertionTime = B1;               // convertion time = 800ms
  illumConfig.ModeOfConversionOperation = B11;   // continous conversion
  illumConfig.Latch = B0;                        // hysteresis-style
  illumErrorConfig = illum.writeConfig(illumConfig);

  if(illumErrorConfig != NO_ERROR){
    illumErrorConfig = illum.writeConfig(illumConfig);   //retry
  }
#endif

  scannerNetworks.begin();
}

void loop() {

  //---------------------------
  // beginning LPWA client
  //---------------------------
  Serial.println("========== Starting Arduino tcp client ==========");

  // After starting the modem with LpwaAccess.begin()
  // attach the shield to the GPRS network with the APN, login and password
  if (lpwaAccess.begin() != LPWA_READY) {
    Serial.println("modem will restart after 5 sec");
    restartSequenceWhenModemError(5000);
    return;
  }

  if (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD, gprs.LTE_CARRIER) != GPRS_READY) {
    Serial.println("modem will restart after 5 sec");
    restartSequenceWhenModemError(5000);
    return;
  }

  Serial.println("LPWA connected");

// Fetch time
  String curTime = lpwaModem.getTime();
  Serial.print("time: ");
  Serial.println(curTime);
  int curHour = curTime.substring(curTime.indexOf(',')+1, curTime.indexOf(':')).toInt();

// Get modem hardware version
  String model = lpwaModem.getModel();
  Serial.print("Modem model: ");
  Serial.println(model);

// Get modem firmware version
  String version = lpwaModem.getFwVersion();
  Serial.print("Firmware version: ");
  Serial.println(version);

// Get current carrier name
  String CA = scannerNetworks.getCurrentCarrier();
  Serial.print("Current carrier: ");
  Serial.println(CA);

  // Get signal strength
  // returns strength and ber
  // signal strength in 0-31 scale. 31 means power > 51dBm
  // BER is the Bit Error Rate. 0-7 scale. 99=not detectable
  String signalPwrStr = scannerNetworks.getSignalStrength();
  if (signalPwrStr == "") {
    Serial.println("No data are fetched");
    Serial.println("lpwaAccess end");
    restartSequenceWhenModemError(5000);
    return;
  }
  float signalPwr = signalPwrStr.toFloat()*2.0-113.0;
  Serial.println("Signal Strength: " + String(signalPwr) + "dBm");
  
  //---------------------------
  // Check active time or not
  //---------------------------
  bool active = false;
  if (HOUR_START <= HOUR_STOP ) {
    if ((curHour >= HOUR_START) && (curHour <= HOUR_STOP)) {
      active = true;
    }
  } else {
    if (((curHour >= HOUR_START) && (curHour <= 24)) || ((curHour <= HOUR_STOP) && (curHour >= 0))) {
      active = true;
    }
  }

  if (active == true) {  //............... if active time
    float dataTemp = 0.0;
    float dataHumid = 0.0;
    float dataIllum = 0.0;

 #ifdef SENSOR4_ON
    //---------------------------
    // obtaining sensor values
    //---------------------------
    dataTemp = (float)smeHumidity.readTemperature();
    dataHumid = (float)smeHumidity.readHumidity();
  
    // calibration:
    dataTemp = TM0 + (TM1 - TM0) * (dataTemp - TL0) / (TL1 - TL0);      // Temperature correction
    dataHumid = HM0 + (HM1 - HM0) * (dataHumid - HL0) / (HL1 - HL0);    // Humidity correction
  
    Serial.println("Temperature: " + String(dataTemp) + "degC");
    Serial.println("Humidity: " + String(dataHumid) + "%");

    OPT3001 result = illum.readResult();
    
    if(result.error == NO_ERROR){
      dataIllum = result.lux;
    }
    Serial.println("Illuminance: " + String(dataIllum) + "lx");
#endif

    //---------------------------
    // Measuring battery level
    //---------------------------
    float batt = pmctrl.getBattLevel() / 1000.0;
    Serial.println("Volt: " + String(batt) + "V");

  
    //---------------------------
    // transfer sensing data to soracom cloud
    //---------------------------
#ifndef SEND_BINARY
    String sendStr;
    sendStr = "{\"Temperature\":\"";
    sendStr += dataTemp;
    sendStr += "\", \"Humidity\":\"";
    sendStr += dataHumid;
    sendStr += "\", \"Illumination\":\"";
    sendStr += dataIllum;
    sendStr += "\", \"Battery Level\":\"";
    sendStr += batt;
    sendStr += "\", \"Signal Power\":\"";
    sendStr += signalPwr;
    sendStr += "\"}";
#else
    // 1st 4bytes [0:3]: Temprature
    // 2nd 4bytes [4:7]: Humidity
    // 3rd 4bytes [8:11]: Illumination
    // 4th 4bytes [12:15]: Battery Level
    // 5th 4bytes [16:19]: Signal Power
    uint8_t sendBinary[20];
    for (int i=0; i<4; i++) {
      sendBinary[i] = *(((uint8_t *)(&dataTemp))+3-i);
      sendBinary[i+4] = *(((uint8_t *)(&dataHumid))+3-i);
      sendBinary[i+8] = *(((uint8_t *)(&dataIllum))+3-i);
      sendBinary[i+12] = *(((uint8_t *)(&batt))+3-i);
      sendBinary[i+16] = *(((uint8_t *)(&signalPwr))+3-i);
    }
#endif
    // if you get a connection, report back via serial:
    if (client.connect(server, port)) {
      Serial.println("connected");
      // Make a HTTP request:
      // client.print("{\"value1\":\"200\", \"value2\":\"300\"}"); // JSON形式で送る
#ifndef SEND_BINARY
      client.print(sendStr);
#else
      client.write(sendBinary, sizeof(sendBinary)/sizeof(sendBinary[0]));
#endif
    } else {
      // if you didn't get a connection to the server:
      Serial.println("connection failed");
    }
  
    //---------------------------
    // receiving a response code
    //---------------------------
    String rsp = "";
    int cnt = 0;
    while (1) {
      // if there are incoming bytes available
      // from the server, read them and print them:
      
      if (client.available()) {
        char c = client.read();
        rsp += c;
      }
    
      // if the server's disconnected, stop the client:
      if ((!client.available() && !client.connected()) || rsp.length() == 4 || cnt > CNT_MAX) {
        Serial.print(rsp);
        Serial.println("disconnecting.");
        client.stop();
        delay(100);
  
        Serial.println("lpwaAccess end");
        lpwaAccess.end();
        pmctrl.powerDown(LPWA_OFF);
        Serial.print("Starting deep sleep for ");
        Serial.print(DEEPSLEEP_TIME/1000);
        Serial.println(" sec");
        delay(10);
        LowPower.deepSleep(DEEPSLEEP_TIME); // msec
        Serial.println("Woke up from deep sleep");
        break;
      }
      cnt++;
    }
    
  } else { //............................ if non-active time
    //---------------------------
    // Calculate deepSleep time
    //---------------------------
    int deepSleepTimeNonActive = HOUR_START - curHour; // deepsleep time during active time (msec)
    if (deepSleepTimeNonActive < 0) {
      deepSleepTimeNonActive += 24;
    }
    deepSleepTimeNonActive = deepSleepTimeNonActive * 3600 * 1000; // convert hour -> msec

    Serial.println("lpwaAccess end");
    lpwaAccess.end();
    pmctrl.powerDown(LPWA_OFF);
    Serial.print("Starting deep sleep for ");
    Serial.print(deepSleepTimeNonActive/1000);
    Serial.println(" sec");
    delay(10);
    LowPower.deepSleep(uint32_t(deepSleepTimeNonActive)); // msec
    Serial.println("Woke up from deep sleep");
  }
}
