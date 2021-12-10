//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : ESP32
//     Processor    : ESP32-WROOM-32
//     Application  : Post Notification
//
//     Leaf configuration
//       (1) AP02 ESP MCU
//       (2) AI01 4-Sensors
//       (3) AX08 29pin header
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/8/17  First release
//=====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include "esp_wpa2.h"
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>
#include "MyLIS3DH.h"

// I2C Address definition
#define OPT3001_ADDRESS 0x45    // ADDR pin = VCC
#define BATT_ADC_ADDR 0x50

// definition for the buzzer pwm
#define LEDC_CHANNEL_0     0      // use first channel of 16 channels (started from zero)
#define LEDC_TIMER_13_BIT  13     // use 13 bit precission for LEDC timer
#define LEDC_BASE_FREQ     5000   // use 5000 Hz as a LEDC base frequency
// buzzer pin
#define BUZZER_OUT 13

// define if using the temperature, humidity, and bright sensors
// #define USE_HTS221
// #define USE_OPT3001
#define USE_BATTERY_READ

// define if using WiFi
#define USE_WiFi
// if connecting to eduroam, also define USE_WiFi_ENTERPRISE
// #define USE_WiFi_ENTERPRISE

// Unique ID
String UniqueID = "Leafony_AP02";

/////////////////////////////////////////////////////////////////////////////////////////////////////
// parameters for connecting eduroam enterprise
/////////////////////////////////////////////////////////////////////////////////////////////////////
#define EAP_IDENTITY "login" //if connecting from another corporation, use identity@organisation.domain in Eduroam 
#define EAP_PASSWORD "password" //your Eduroam password
const char* SSID_ENT = "eduroam"; // Eduroam SSID
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Connecting WiFi Settings
const char* SSID = "wifi-ssid";           // WiFi SSID
const char* PASSWORD = "wifi-psw";   // WiFi Password
// Accessed Google Script Settings
const char* APP_SERVER = "script.google.com";
const char* KEY = "google-apps-script-key";

// Device sleep time (sec) to reduce Joule heat

// mode definition
// 0: practical mode
// 1: setup mode
uint8_t appMode = 0;

// instance to handle a bright sensor
#ifdef USE_OPT3001
ClosedCube_OPT3001 illum;
#endif

// instance to handle an accelerometer sensor
MyLIS3DH lis;


#ifdef USE_WiFi
void accessToGoogleSheets(float temperature, float humidity, float illumination, float battery) {
    HTTPClient http;
    String URL = "https://script.google.com/macros/s/";

    URL += KEY;
    URL += "/exec?";
    URL += "UniqueID=";
    URL += UniqueID;
    URL += "&temperature=";
    URL += temperature;
    URL += "&humidity=";
    URL += humidity;
    URL += "&illumination=";
    URL += illumination;
    URL += "&battery=";
    URL += battery;

    Serial.println("[HTTP] begin...");
    Serial.println(URL);
    // access to your Google Sheets
    Serial.println();
    // configure target server and url
    http.begin(URL);

    Serial.println("[HTTP] GET...");
    // start connection and send HTTP header
    int httpCode = http.GET();

    // httpCode will be negative on error
    if(httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        Serial.print("[HTTP] GET... code: ");
        Serial.println(httpCode);

        // file found at server
        if(httpCode == HTTP_CODE_OK) {
            String payload = http.getString();
            Serial.println(payload);
        }
    } else {
        Serial.print("[HTTP] GET... failed, error: ");
        Serial.println(http.errorToString(httpCode).c_str());
    }
}
#endif

#ifdef USE_HTS221
float getTemperature() {
    // sensor calibration data
    float TL0 = 25.0;
    float TM0 = 25.0;
    float TL1 = 40.0;
    float TM1 = 40.0;

    // read temperature
    float temperature = (float)smeHumidity.readTemperature();
    // calibrate temperature
    temperature = TM0 + (TM1 - TM0) * (temperature - TL0) / (TL1 - TL0);
    return temperature;
}
#endif

#ifdef USE_HTS221
float getHumidity() {
    // sensor calibration data
    float HL0 = 60.0;
    float HM0 = 60.0;
    float HL1 = 80.0;
    float HM1 = 80.0;

    // read temperature
    float humidity = (float)smeHumidity.readHumidity();
    // calibrate temperature
    humidity = HM0 + (HM1 - HM0) * (humidity - HL0) / (HL1 - HL0);
    return humidity;
}
#endif

#ifdef USE_OPT3001
float getillumination() {
    OPT3001 result = illum.readResult();
    float illumination;

    if(result.error == NO_ERROR){
        illumination = result.lux;
    } else {
        illumination = -99.9;
    }
    return illumination;
}
#endif

#ifdef USE_BATTERY_READ
float getBatteryLevel(void) {
    Wire.beginTransmission(BATT_ADC_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(BATT_ADC_ADDR, 2);
    uint8_t adcVal1 = Wire.read();
    uint8_t adcVal2 = Wire.read();
    if (adcVal1 == 0xFF && adcVal2 == 0xFF) {
        adcVal1 = 0;
        adcVal2 = 0;
    }
    // voltage mV = adcVal * Vref(3.3V) / resolution(8bit) * Vdiv(2)
    double tempMillivolt = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
    float dataBatt = (float)(tempMillivolt / 1000);
    return dataBatt;
}
#endif

void espDeepSleep() {
    // esp_sleep_enable_timer_wakeup(60 * 1000 * 1000);  // set deep sleep time (60 sec)
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, LOW);
    esp_deep_sleep_start();   // enter deep sleep
}

#ifdef USE_WiFi
void myWifiStart() {
    // WiFi connection controlling parameters
    int statusCheckCounter = 0;
    const int CHECK_NUM_MAX = 100;

#ifdef USE_WiFi_ENTERPRISE
    WiFi.mode(WIFI_STA);
    esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)); //provide identity
    esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)); //provide username --> identity and username is same
    esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD)); //provide password
    esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();
    esp_wifi_sta_wpa2_ent_enable(&config);
    WiFi.begin(SSID_ENT); //connect to wifi
#else
    WiFi.begin(SSID, PASSWORD);
#endif
    Serial.println("Wi-Fi connecting");
    Serial.print("Wi-Fi Status: ");
    // Wait until succeed connecting.
    // If the number of checks is more than CHECK_NUM_MAX, give up connecting and
    // start deepsleep to prevent Joule heat from affecting next measurements.
    while (WiFi.status() != WL_CONNECTED) {
        if(statusCheckCounter > CHECK_NUM_MAX) {
            WiFi.disconnect(true);
            Serial.println("failed");
            Serial.println("Deepsleep Start");
            espDeepSleep();
        }
        Serial.print(WiFi.status());
        delay(500);
        statusCheckCounter++;
    }
    Serial.println("\nconnected");
}

void myWifiStop() {
    // WiFi Connection killed
    WiFi.disconnect(true);
    Serial.println("\nWiFi is disconnected");
}
#endif

void setup() {

#ifdef USE_OPT3001
    OPT3001_Config illumConfig;
    OPT3001_ErrorCode illumErrorConfig;
#endif

    Serial.begin(115200);

    uint16_t touchref = 45;
    uint16_t t5data = touchRead(T5);
    uint16_t t6data = touchRead(T6);

    if (t5data+t6data < touchref) {
        ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
        ledcAttachPin(BUZZER_OUT, LEDC_CHANNEL_0);
        appMode = 1;
    }

    // I2C for bright sensor is initialized
    // intialize i2c communication
    Wire.begin();
#ifdef USE_OPT3001
    illum.begin(OPT3001_ADDRESS);
#endif

#ifdef USE_HTS221
    // I2C for temperature and humidity sensor is initialized
    // Data pin is initialized exceptionally for ESP32 Leaf's I2C Bugs
    pinMode(21, OUTPUT);
    digitalWrite(21, 0);
    Wire.begin();
    smeHumidity.begin();
#endif

#ifdef USE_OPT3001
    illumConfig.RangeNumber = B1100;              // automatic full scale
    illumConfig.ConvertionTime = B1;              // convertion time = 800ms
    illumConfig.ModeOfConversionOperation = B11;  // continous conversion
    illumConfig.Latch = B0;                       // hysteresis-style
    illumErrorConfig = illum.writeConfig(illumConfig);
    if (illumErrorConfig != NO_ERROR) {
        illumErrorConfig = illum.writeConfig(illumConfig);  //retry
    }
#endif

    while (!lis.begin(0x19)) {   // change this to 0x19 for alternative i2c address
        Serial.println("LIS3DH Could not start");
        delay(500);
    }
    Serial.println("LIS3DH found!");
    lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
}


void loop() {

    switch (appMode) {
        case 0:
            {
                float temperature;
                float humidity;
                float illumination;
                float battery;
#ifdef USE_HTS221
                // get sensor values
                temperature = getTemperature();
                humidity = getHumidity();
#else
                temperature = 99.9;
                humidity = 99.9;
#endif
                Serial.print("\ntemperature : ");
                Serial.println(temperature);
                Serial.print("humidity    : ");
                Serial.println(humidity);

#ifdef USE_OPT3001
                illumination = getillumination();
#else
                illumination = 99.9;
#endif
                Serial.print("illumination  : ");
                Serial.println(illumination);

#ifdef USE_BATTERY_READ
                battery = getBatteryLevel();
#else
                battery = 99.9;
#endif
                Serial.print("battery level  : ");
                Serial.println(battery);

#ifdef USE_WiFi
                myWifiStart();
                // send sensor values to google sheets
                accessToGoogleSheets(temperature, humidity, illumination, battery);
                myWifiStop();
#endif

                // ESP32 start deep sleep
                // After calling "esp_deep_sleep_start" function, any following codes will not be executed
                // When restarting ESP32, all variables are restored and the program will start from the beginning
                lis.intrWhenDetect6dOrientation();
                Serial.println("Deepsleep Start");
                espDeepSleep();
            }
            break;

        case 1:
            {
                lis.intrWhenDetect6dOrientation();
                while (1) {
                    lis.read();
                    Serial.print("X [g] = " + String(lis.x_g));
                    Serial.print(", ");
                    Serial.print("Y [g] = " + String(lis.y_g));
                    Serial.print(", ");
                    Serial.print("Z [g] = " + String(lis.z_g));
                    Serial.print(", ");
                    uint8_t flag = lis.getRegINT1SRC();
                    Serial.print("INT1SRC: ");
                    Serial.print(flag, BIN);
                    Serial.print(", ");
                    if (flag >= 0x40) {
                        Serial.print("Interrupt Occured");
                    }
                    Serial.println("");
                    delay(100);
                }
            }
            break;

        defalut:
            delay(1000);
            break;
    }
    return;
}
