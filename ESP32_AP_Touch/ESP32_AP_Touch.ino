//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : ESP32
//     Processor    : ESP32-WROOM-32
//     Application  : ESP32 Touch Sensor
//
//     Leaf configuration
//       (1) AP02A ESP MCU
//       (2) AX08A 29pin header
//
//    (c) 2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01  First release
//=====================================================================
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "web_server.h"
#include "esp32_ap_touch.h"

const char WIFI_SSID[] = "Leafony_ESP32-AP";
const char WIFI_PASSWORD[] = "password";
WiFiServer server(80);

void setup(){ 
    SERIAL_BEGIN(115200);
    SERIAL_PRINTLN("Wi-Fi & Touch Sensor Test");

    // WiFi settings
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    IPAddress myIP = WiFi.softAPIP();

    SERIAL_PRINT("AP IP address: ");
    SERIAL_PRINTLN(myIP);

    // Wifi server start
    server.begin();

    SERIAL_PRINTLN("Server started");
}

void loop(){
    int ret;
    int refreshTime = 1;                                // HTML page refresh time (sec)
    WiFiClient client = server.available();             // listen for incoming clients

    if (client) {                                       // if you get a client
        String currentLine = "";                        // make a String to hold incoming data from the client
        while (client.connected()) {                    // loop while the client's connected
            if (client.available()) {                   // if there's bytes to read from the client
                char byteData = client.read();          // read a byte, then
                if (byteData == '\n') {                 // if the byte is newline character
                    // if the current line is blank, you got two newline characters in a row.
                    // That's the end of the client HTTP request, so send a response:
                    if (currentLine.length() == 0) {
                        htmlTouchSensorMain(            // send html contents
                            client,
                            refreshTime,
                            WiFi.softAPIP());
                        break;                          // break out of the while loop:
                    } else {                            // if you got a newline
                        // check if the data from client contains "GET /?INT="
                        ret = currentLine.indexOf("GET /?INT=");
                        if (ret > -1) {     // if found,
                            // extract the number corresponding to required page refresh period from currentLine
                            // and cast it to String, and cast it again to int.
                            // Then, assign to refreshTime
                            refreshTime = String(currentLine.charAt(ret + 10)).toInt();
                            SERIAL_PRINT("refresh time updated :");
                            SERIAL_PRINT(refreshTime);
                            SERIAL_PRINTLN("s");
                        }
                        currentLine = "";   // clear the current Line to store new incoming data.
                    }
                } else if (byteData != '\r') {          // if you got anything else but a carriage return character,
                    currentLine += String(byteData);    // add it to the end of the currentLine
                }
            }
        }
    }
}
