//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : ESP32
//     Processor    : ESP32-WROOM-32
//     Application  : Sending Sensor Data to Google Sheets
//
//     Leaf configuration
//       (1) AP02 ESP MCU
//       (2) AI01 4-Sensors
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/8/17  First release
//=====================================================================


#include <Arduino.h>
#include <Wire.h>
#include <HTTPClient.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>

// Connecting WiFi Settings
const char* SSID = "wifi_ssid";    // WiFi SSID
const char* PASSWORD = "wifi_password";  // WiFi Password

// Accessed Google Script Settings
const char* APP_SERVER = "script.google.com";
const char* KEY = "google_scripts_key";

// Device sleep time (sec) to reduce Joule heat
uint64_t DEEP_SLEEP_TIME_SEC = 60;

// instance to handle a bright sensor
ClosedCube_OPT3001 illum;

// I2C Address definition
#define OPT3001_ADDRESS 0x45  // ADDR pin = VCC


void accessToGoogleSheets(float temperature, float humidity, float brightness) {
    HTTPClient http;
    String URL = "https://script.google.com/macros/s/";

    URL += KEY;
    URL += "/exec?";
    URL += "temperature=";
    URL += temperature;
    URL += "&humidity=";
    URL += humidity;
    URL += "&brightness=";
    URL += brightness;

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
    float temperature = (float)smeHumidity.readTemperature();
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
    float humidity = (float)smeHumidity.readHumidity();
    // calibrate temperature
    humidity = HM0 + (HM1 - HM0) * (humidity - HL0) / (HL1 - HL0);
    return humidity;
}

float getBrightness() {
    OPT3001 result = illum.readResult();
    float brightness;

    if(result.error == NO_ERROR){
        brightness = result.lux;
    } else {
        brightness = -99.9;
    }
    return brightness;
}

void espDeepSleep() {
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME_SEC * 1000 * 1000);  // set deep sleep time
    esp_deep_sleep_start();   // enter deep sleep
}

void setup() {

    OPT3001_Config illumConfig;
    OPT3001_ErrorCode illumErrorConfig;
    // WiFi connection controlling parameters
    int statusCheckCounter = 0;
    const int CHECK_NUM_MAX = 100;

    Serial.begin(115200);

    // I2C for bright sensor is initialized
    // intialize i2c communication
    Wire.begin();
    illum.begin(OPT3001_ADDRESS);

    // I2C for temperature and humidity sensor is initialized
    // Data pin is initialized exceptionally for ESP32 Leaf's I2C Bugs
    pinMode(21, OUTPUT);
    digitalWrite(21, 0);
    Wire.begin();
    smeHumidity.begin();

    illumConfig.RangeNumber = B1100;              // automatic full scale
    illumConfig.ConvertionTime = B1;              // convertion time = 800ms
    illumConfig.ModeOfConversionOperation = B11;  // continous conversion
    illumConfig.Latch = B0;                       // hysteresis-style
    illumErrorConfig = illum.writeConfig(illumConfig);
    if (illumErrorConfig != NO_ERROR) {
        illumErrorConfig = illum.writeConfig(illumConfig);  //retry
    }

    WiFi.begin(SSID, PASSWORD);
    Serial.print("WiFi connecting");
    // Wait until succeed connecting.
    // If the number of checks is more than CHECK_NUM_MAX, give up connecting and
    // start deepsleep to prevent Joule heat from affecting next measurements.
    while (WiFi.status() != WL_CONNECTED) {
        if(statusCheckCounter > CHECK_NUM_MAX) {
            WiFi.disconnect();
            Serial.println("failed");
            Serial.println("Deepsleep Start");
            espDeepSleep();
        }
        Serial.print(".");
        delay(500);
        statusCheckCounter++;
    }
    Serial.println("\nconnected");

}

void loop() {

    // get sensor values
    float temperature = getTemperature();
    float humidity = getHumidity();
    float brightness = getBrightness();

    Serial.print("\ntemperature : ");
    Serial.println(temperature);
    Serial.print("humidity    : ");
    Serial.println(humidity);
    Serial.print("brightness  : ");
    Serial.println(brightness);

    // send sensor values to google sheets
    accessToGoogleSheets(temperature, humidity, brightness);

    // WiFi Connection killed
    Serial.println("\nWiFi is disconnected");
    WiFi.disconnect();

    // ESP32 start deep sleep
    // After calling "esp_deep_sleep_start" function, any following codes will not be executed
    // When restarting ESP32, all variables are restored and the program will start from the beginning
    Serial.println("Deepsleep Start");
    espDeepSleep();

}
