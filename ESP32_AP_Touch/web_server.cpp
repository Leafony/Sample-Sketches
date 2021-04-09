#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "web_server.h"
#include "esp32_ap_touch.h"

static int printMsgIfTouched(WiFiClient client, uint16_t touchData, String htmlMsg) {
    uint16_t touchTemp;
    const uint16_t THRESHOLD = 20;

    client.print(htmlMsg);
    for (touchTemp = 0; touchTemp <= touchData / 5; touchTemp++) {
        client.print("*");
    }
    client.print(":" + String(touchData));
    if (touchData > THRESHOLD) {
        client.println("<br>");
        return -1;
    }
    client.println(" :Touch detected");
    client.println("<br>");
    return 0;
}

static int printHtmlHeader(WiFiClient client, int update, String ipAddress) {
    client.println("<head><title>Test Page</title>");
    client.println("<meta http-equiv=\"Content-type\" content=\"text/html; charset=UTF-8\">");
    if (update) {
        client.print("<meta http-equiv=\"refresh\" content=\"");
        client.print(update);
        client.print(";URL=http://");
        client.print(ipAddress);
        client.println("/\">");
    }
    client.println("</head>");
    return 0;
}

static int printHtmlBody(WiFiClient client, int update, String ipAddress) {
    uint16_t touchData;

    client.println("<body>");
    client.println("<h3>Wi-Fi & Touch Sensor Test</h3>");

    touchData = touchRead(T6);
    if (printMsgIfTouched(client, touchData, "F7/D6") == 0) {
        SERIAL_PRINTLN("T6/D6/ 7pin :" + String(touchData) + " :Touch detected");
    }

    touchData = touchRead(T3);
    if (printMsgIfTouched(client, touchData, "F9/D7") == 0) {
        SERIAL_PRINTLN("T3/D7/ 9pin :" + String(touchData) + " :Touch detected");
    }

    touchData = touchRead(T0);
    if (printMsgIfTouched(client, touchData, "F22/D2") == 0) {
        SERIAL_PRINTLN("T0/D2/22pin :" + String(touchData) + " :Touch detected");
    }

    touchData = touchRead(T5);
    if (printMsgIfTouched(client, touchData, "F26/D4") == 0) {
        SERIAL_PRINTLN("T5/D4/26pin :" + String(touchData) + " :Touch detected");
    }

    touchData = touchRead(T4);
    if (printMsgIfTouched(client, touchData, "F28/D5") == 0) {
        SERIAL_PRINTLN("T4/D5/28pin :" + String(touchData) + " :Touch detected");
    }

    client.print("Update interval = ");
    client.print(update);
    client.println(" second</p>");
    client.println("<hr>");
    client.print("<p>http://");
    client.print(ipAddress);
    client.println("<form method=\"GET\" action=\"http://" + ipAddress + "/\">");
    client.println("automatic update:<input type=\"submit\" name=\"INT\" value=\"0 Stop\">");
    client.println("<input type=\"submit\" name=\"INT\" value=\"1 second\">");
    client.println("</form>");
    client.println("</body>");
    return 0;
}

static String ipUintToString(uint32_t ip) {
    char ipCharArray[16];
    sprintf(ipCharArray,"%d.%d.%d.%d",
        ip & 255,
        ip>>8 & 255,
        ip>>16 & 255,
        ip>>24
    );
    return String(ipCharArray);
}

static void connectHtml(WiFiClient client) {
    client.println("HTTP/1.1 200 OK");              // response HTTP OK
    client.println("Content-Type: text/html");
    client.println("Connection: close");            // close session after sending all data
    client.println();
}

void htmlTouchSensorMain(WiFiClient client, int update, uint32_t ip) {
    String ipAddress = ipUintToString(ip);

    client.println("<html>");

    printHtmlHeader(client, update, ipAddress);

    printHtmlBody(client, update, ipAddress);

    client.println("</html>");
}
