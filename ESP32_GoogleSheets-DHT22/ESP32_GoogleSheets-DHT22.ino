//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : ESP32
//     Processor    : ESP32-WROOM-32
//     Application  : Sending Sensor Data to Google Sheets
//
//     Leaf configuration
//       (1) AP02 ESP MCU
//       (2) AX07 Back to back
//       (3) AX08 29pin header
//
//       AX08 29pin             DHT22
//       +-----------+         +-----------+
//       |         1 +-- VCC --+ 1   ##### |
//       |        15 +-- DAT --+ 2   ##### |
//       |        27 +-- GND --+ 3   ##### |
//       +-----------+         +-----------+
//
//    (c) 2022 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2022/10/19  First release
//=====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <HTS221.h>
#include <DHT.h>
#include <DHT_U.h>

//#define ENTERPRISE

#ifdef ENTERPRISE   // Enterprise
#include <esp_wpa2.h>
#endif

#define DHT_PIN 5       // D5 = pin15
DHT dht(DHT_PIN,DHT22);

// Unique ID
String UniqueID = "Leafony_A";

#ifdef ENTERPRISE   // Enterprise
#define EAP_IDENTITY "login" //if connecting from another corporation, use identity@organisation.domain in Eduroam 
#define EAP_PASSWORD "password" //your Eduroam password
const char* SSID_ENT = "eduroam"; // Eduroam SSID
#endif

// Connecting WiFi Settings
const char* SSID = "wifi_ssid";           // WiFi SSID
const char* PASSWORD = "wifi_password";   // WiFi Password
// Accessed Google Script Settings
const char* APP_SERVER = "script.google.com";
const char* KEY = "google_scripts_key";

// Device sleep time (sec) to reduce Joule heat
uint64_t DEEP_SLEEP_TIME_SEC = 60;


void accessToGoogleSheets(float temperature, float humidity, float illumination) {
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

float getTemperature() {
    // sensor calibration data
    float TL0 = 25.0;
    float TM0 = 25.0;
    float TL1 = 40.0;
    float TM1 = 40.0;

    // read temperature
    float temperature = (float)dht.readTemperature();
    // calibrate temperature
    temperature = TM0 + (TM1 - TM0) * (temperature - TL0) / (TL1 - TL0);
    return temperature;
}

float getHumidity() {
    // sensor calibration data
    float HL0 = 60.0;
    float HM0 = 60.0;
    float HL1 = 80.0;
    float HM1 = 80.0;

    // read temperature
    float humidity = (float)dht.readHumidity();
    // calibrate temperature
    humidity = HM0 + (HM1 - HM0) * (humidity - HL0) / (HL1 - HL0);
    return humidity;
}

void espDeepSleep() {
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME_SEC * 1000 * 1000);  // set deep sleep time
    esp_deep_sleep_start();   // enter deep sleep
}

void setup() {

    // WiFi connection controlling parameters
    int statusCheckCounter = 0;
    const int CHECK_NUM_MAX = 100;

    Serial.begin(115200);

    // I2C for bright sensor is initialized
    // intialize i2c communication
    Wire.begin();

    dht.begin();
    
#ifdef ENTERPRISE   // Enterprise
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

    Serial.print("WiFi connecting");
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

void loop() {

    // get sensor values
    float temperature = getTemperature();
    float humidity = getHumidity();
    float illumination = 0;

    Serial.print("\ntemperature : ");
    Serial.println(temperature);
    Serial.print("humidity    : ");
    Serial.println(humidity);
    Serial.print("illumination  : ");
    Serial.println(illumination);

    // send sensor values to google sheets
    accessToGoogleSheets(temperature, humidity, illumination);

    // WiFi Connection killed
    Serial.println("\nWiFi is disconnected");
    WiFi.disconnect(true);

    // ESP32 start deep sleep
    // After calling "esp_deep_sleep_start" function, any following codes will not be executed
    // When restarting ESP32, all variables are restored and the program will start from the beginning
    Serial.println("Deepsleep Start");
    espDeepSleep();

}
