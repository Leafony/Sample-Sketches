#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "web_server.h"

static int printMsgIfTouched(WiFiClient client, uint16_t touchData, String htmlMsg) {
    uint16_t touchTemp;
    const uint16_t THRESHOLD = 20;

    client.print(htmlMsg);
    for (touchTemp = 0; touchTemp <= touchData/5; touchTemp++) {
        client.print("*");
    }
    client.print(":"+String(touchData));
    if (touchData > THRESHOLD) {
        return -1;
    }
    client.println(" :Touch detected");
    client.println("<br>");
    return 0;
}

static int printHtmlHeader(WiFiClient client, int update, char *s_ip) {
    client.println("<head><title>Test Page</title>");
    client.println("<meta http-equiv=\"Content-type\" content=\"text/html; charset=UTF-8\">");
    if (update) {
        client.print("<meta http-equiv=\"refresh\" content=\"");
        client.print(update);
        client.print(";URL=http://");
        client.print(s_ip);
        client.println("/\">");
    }
    client.println("</head>");
    return 0;
}

static int printHtmlBody(WiFiClient client, int update, char *s_ip) {
    uint16_t touchData;
    char s[65];

    client.println("<body>");
    client.println("<h3>Wi-Fi & Touch Sensor Test</h3>");

    touchData = touchRead(T6);
    if (printMsgIfTouched(client, touchData, "F7/D6") == 0) {
        Serial.println("T6/D6/ 7pin :" + String(touchData) + " :Touch detected");
    }

    touchData = touchRead(T3);
    if (printMsgIfTouched(client, touchData, "F9/D7") == 0) {
        Serial.println("T3/D7/ 9pin :" + String(touchData) + " :Touch detected");
    }

    touchData = touchRead(T0);
    if (printMsgIfTouched(client, touchData, "F22/D2") == 0) {
        Serial.println("T0/D2/22pin :" + String(touchData) + " :Touch detected");
    }
    /*
    touchData = touchRead(T7);
    if (printMsgIfTouched(client, touchData, "F24/D3") == 0) {
        Serial.println("T7/D3/24pin :" + String(touchData) + " :Touch detected");
    }
    */
    touchData = touchRead(T5);
    if (printMsgIfTouched(client, touchData, "F26/D4") == 0) {
        Serial.println("T5/D4/26pin :" + String(touchData) + " :Touch detected");
    }

    touchData = touchRead(T4);
    if (printMsgIfTouched(client, touchData, "F28/D5") == 0) {
        Serial.println("T4/D5/28pin :" + String(touchData) + " :Touch detected");
    }

    client.print("更新間隔 = ");
    client.print(update);
    client.println(" 秒</p>");
    client.println("<hr>");
    client.print("<p>http://");
    client.print(s_ip);
    sprintf(s,"<form method=\"GET\" action=\"http://%s/\">",s_ip);
    client.println(s);

    client.println("自動更新:<input type=\"submit\" name=\"INT\" value=\"0 停止\">");
    client.println("<input type=\"submit\" name=\"INT\" value=\"1 秒\">");

    client.println("</form>");
    client.println("</body>");

    return 0;
}

static void parseIP(uint32_t ip, char *s_ip) {
    sprintf(s_ip,"%d.%d.%d.%d",
        ip & 255,
        ip>>8 & 255,
        ip>>16 & 255,
        ip>>24
    );
}

static void connectHtml(WiFiClient client) {
    client.println("HTTP/1.1 200 OK");              // HTTP OKを応答
    client.println("Content-Type: text/html");      // HTMLコンテンツ
    client.println("Connection: close");            // 応答終了後にセッションを閉じる
    client.println();
}

void htmlTouchSensorMain(WiFiClient client, int update, uint32_t ip) {
    char s_ip[16];

    parseIP(ip, s_ip);
    client.println("<html>");

    printHtmlHeader(client, update, s_ip);

    printHtmlBody(client, update, s_ip);

    client.println("</html>");
}

void htmlMsgMain(WiFiClient client, char *txt, uint32_t ip) {
    char s_ip[16];
    
    parseIP(ip, s_ip);
    connectHtml(client);

    client.println("<html>");

    client.println("<head><title>Test Page</title>");
    client.println("<meta http-equiv=\"Content-type\" content=\"text/html; charset=UTF-8\">");
    client.print("<meta http-equiv=\"refresh\" content=\"3;URL=http://");
    client.print(s_ip);
    client.println("/\">");
    client.print("<p>");
    client.print(txt);
    client.println("</p>");
    client.println("</body>");
    client.println("</html>");
}