//=====================================================================
//  Leafony Platform sample sketch
//     Application  : html
//
//    (c) 2019 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2019/11/07  First release
//=====================================================================
//====================================================================
// Web表示
//====================================================================
void html(WiFiClient &client,  int update, uint32_t ip){
    char s[65],s_ip[16];
    uint16_t touchData;
    uint16_t touchTemp;
    sprintf(s_ip,"%d.%d.%d.%d",
        ip & 255,
        ip>>8 & 255,
        ip>>16 & 255,
        ip>>24
    );
    client.println("HTTP/1.1 200 OK");              // HTTP OKを応答
    client.println("Content-Type: text/html");      // HTMLコンテンツ
    client.println("Connection: close");            // 応答終了後にセッションを閉じる
    client.println();
    client.println("<html>");
    client.println("<head><title>Test Page</title>");
    client.println("<meta http-equiv=\"Content-type\" content=\"text/html; charset=UTF-8\">");
    if(update){
        client.print("<meta http-equiv=\"refresh\" content=\"");
        client.print(update);
        client.print(";URL=http://");
        client.print(s_ip);
        client.println("/\">");
    }
    client.println("</head>");
    client.println("<body>");
    client.println("<h3>Wi-Fi & Touch Sensor Test</h3>");
// ------ T6 ------
            touchData=touchRead(T6);
            client.print("F7 /D6:");
            for (touchTemp=0; touchTemp<=touchData/5; touchTemp++){
              client.print("*");
              }
            client.print(":"+String(touchData));
//            if(touch5detected){
            if(touchData<=threshold){
//              touch5detected = false;
              Serial.println("T6/D6/ 7pin :"+String(touchData)+" :Touch detected");
              client.println(" :Touch detected");
            }
            client.println("<br>");
// ------ T3 ------
            touchData=touchRead(T3);
            client.print("F9 /D7:");
            for (touchTemp=0; touchTemp<=touchData/5; touchTemp++){
              client.print("*");
              }
            client.print(":"+String(touchData));
//            if(touch2detected){
            if(touchData<=threshold){
//              touch2detected = false;
              Serial.println("T3/D7/ 9pin :"+String(touchData)+" :Touch detected");
              client.println(" :Touch detected");
            }
            client.println("<br>");
// ------ T0 ------
            touchData=touchRead(T0);
            client.print("F22/D2:");
            for (touchTemp=0; touchTemp<=touchData/5; touchTemp++){
              client.print("*");
              }
            client.print(":"+String(touchData));
//            if(touch1detected){
            if(touchData<=threshold){
//              touch1detected = false;
              Serial.println("T0/D2/22pin :"+String(touchData)+" :Touch detected");
              client.println(" :Touch detected");
            }
            client.println("<br>");
/*
// ------ T7 ------
            touchData=touchRead(T7);
            client.print("F24/D3:");
            for (touchTemp=0; touchTemp<=touchData/5; touchTemp++){
              client.print("*");
              }
            client.print(":"+String(touchData));
//            if(touch6detected){
            if(touchData<=threshold){
//              touch6detected = false;
              Serial.println("T7/D3/24pin :"+String(touchData)+" :Touch detected");
              client.println(" :Touch detected");
            }
            client.print("<br>");
*/
// ------ T5 ------
            touchData=touchRead(T5);
            client.print("F26/D4:");
            for (touchTemp=0; touchTemp<=touchData/5; touchTemp++){
              client.print("*");
              }
            client.print(":"+String(touchData));
//            if(touch4detected){
            if(touchData<=threshold){
//              touch4detected = false;
              Serial.println("T5/D4/26pin :"+String(touchData)+" :Touch detected");
              client.println(" :Touch detected");
            }
            client.println("<br>");
// ------ T4 ------
            touchData=touchRead(T4);
            client.print("F28/D5:");
            for (touchTemp=0; touchTemp<=touchData/5; touchTemp++){
              client.print("*");
              }
            client.print(":"+String(touchData));
//            if(touch3detected){
            if(touchData<=threshold){
//              touch3detected = false;
              Serial.println("T4/D5/28pin :"+String(touchData)+" :Touch detected");
              client.println(" :Touch detected");
            }
            client.println("<br>");

    client.print("更新間隔 = ");
    client.print(update);
    client.println(" 秒</p>");
    client.println("<hr>");
//    client.println("<h3>HTTP GET</h3>");
    client.print("<p>http://");
    client.print(s_ip);
    sprintf(s,"<form method=\"GET\" action=\"http://%s/\">",s_ip);
    client.println(s);

    client.println("自動更新:<input type=\"submit\" name=\"INT\" value=\"0 停止\">");
    client.println("<input type=\"submit\" name=\"INT\" value=\"1 秒\">");
//    client.println("<input type=\"submit\" name=\"INT\" value=\"2 秒\">");
//    client.println("<input type=\"submit\" name=\"INT\" value=\"5 秒\">");
//    client.println("<input type=\"submit\" name=\"INT\" value=\"10 秒\">");
//    client.println("　<input type=\"submit\" name=\"RESET\" value=\"リセット\"><br><br>");

    client.println("</form>");
    client.println("</body>");
    client.println("</html>");
}

//====================================================================
// コマンド送信
//====================================================================
void htmlMesg(WiFiClient &client, char *txt, uint32_t ip){
    char s_ip[16];
    
    sprintf(s_ip,"%d.%d.%d.%d",
        ip & 255,
        ip>>8 & 255,
        ip>>16 & 255,
        ip>>24
    );
    client.println("HTTP/1.1 200 OK");              // HTTP OKを応答
    client.println("Content-Type: text/html");      // HTMLコンテンツ
    client.println("Connection: close");            // 応答終了後にセッションを閉じる
    client.println();
    client.println("<html>");
    client.println("<head><title>Test Page2</title>");
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
