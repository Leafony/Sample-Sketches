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
//    (c) 2019 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2019/11/21  First release
//=====================================================================
#include <WiFi.h>                                             // ESP32用WiFiライブラリ
#include <WiFiClient.h>
#include <WiFiAP.h>

//----------------------------------------------
// Wi-Fi Set these to your desired credentials.
//----------------------------------------------
const char *ssid = "Leafony_ESP32-AP";
const char *password = "password";

WiFiServer server(80);

#define TIMEOUT 20000                                         // タイムアウト 20秒
int update=1;                                                 // ブラウザのページ更新間隔(秒)初期値

//----------------------------------------------
// Touch
//----------------------------------------------
int threshold = 20;                                           // Touch 検出閾値
// bool touch1detected = false;
// bool touch2detected = false;
// bool touch3detected = false;
// bool touch4detected = false;
// bool touch5detected = false;
// bool touch6detected = false;

//----------------------------------------------
// Touch 割り込み
//----------------------------------------------
// ------ T0 ------
// void gotTouch1(){
//  touch1detected = true;
// }
// ------ T3 ------
// void gotTouch2(){
//  touch2detected = true;
// }
// ------ T4 ------
// void gotTouch3(){
//  touch3detected = true;
// }
// ------ T5 ------
// void gotTouch4(){
//  touch4detected = true;
// }
// ------ T6 ------
// void gotTouch5(){
//  touch5detected = true;
// }
// ------ T7 ------
// void gotTouch6(){
//  touch6detected = true;
// }

//====================================================================
void setup(){ 
    Serial.begin(115200);                                     // デバッグシリアル出力開始
    Serial.println("Wi-Fi & Touch Sensor Test");              // シリアル出力表示

    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    server.begin();                                           // サーバ起動
    delay(1000);                                              // Wait
    Serial.println("Server started");

//----------------------------------------------
// Touch 割り込み設定
//----------------------------------------------
//    touchAttachInterrupt(T0, gotTouch1, threshold);           // D2/22pin
//    touchAttachInterrupt(T3, gotTouch2, threshold);           // D7/9pin
//    touchAttachInterrupt(T4, gotTouch3, threshold);           // D5/28pin
//    touchAttachInterrupt(T5, gotTouch4, threshold);           // D4/26pin
//    touchAttachInterrupt(T6, gotTouch5, threshold);           // D6/7pin
//    touchAttachInterrupt(T7, gotTouch6, threshold);           // D3/24pin
}

//====================================================================
void loop(){
    delay(1000);                                              // Wait：これが短いと”このページは動作していません"が頻発する
    WiFiClient client = server.available();                   // 接続中のクライアントからデータを受信

    char read_data;                                           // 文字変数を定義
    char s[65];                                               // 文字列変数を定義 65バイト64文字
    byte data[32];                                            // 画像転送用の一時保存変数
    int len=0;                                                // 文字列等の長さカウント用
    int t_wait=0;                                             // 更新時間カウント用
    int i,f_size;
    
    delay(500);                                               // Wait：これが短いと”このページは動作していません"が頻発する
    client = server.available();                              // 接続されたクライアントを生成

    if(!client)return;                                        // loop()の先頭に戻る
//    Serial.println("Connected");                              // シリアル出力表示
    while(client.connected()){                                // 当該クライアントの接続状態を確認
        if(client.available()){                               // クライアントからのデータを確認
            t_wait=0;                                         // 待ち時間変数をリセット
            read_data=client.read();                          // データを文字変数read_dataに代入

            if(read_data=='\n'){                              // 改行を検出した時
                if(len>5 && strncmp(s,"GET /",5)==0) break;
                len=0;                                        // 文字列長を0に
            }else if(read_data!='\r' && read_data!='\0'){
                s[len]=read_data;                             // 文字列変数に文字read_dataを追加
                len++;                                        // 変数lenに1を加算
                s[len]='\0';                                  // 文字列を終端
                if(len>=64) len=63;                           // 文字列変数の上限
            }
        }
        t_wait++;                                             // 変数t_waitの値を1増加させる
        if(t_wait>TIMEOUT) break; else delay(1);              // TIMEOUTに到達したらwhileを抜ける
    }

    delay(1);                                                 // クライアント側の応答待ち時間
    if(!client.connected()||len<6) return;                    // 切断された場合はloop()の先頭へ
//    Serial.println(s);                                        // 受信した命令をシリアル出力表示
    if(strncmp(s,"GET / ",6)==0){                             // コンテンツ取得命令時
      //----------------------------------------------
      // Web 表示
      //----------------------------------------------
        html(client,update,WiFi.softAPIP());                  // コンテンツ表示
        client.flush();                                       // ESP32用 ERR_CONNECTION_RESET対策
//        client.stop();                                      // クライアントの切断
        return;                                               // 処理の終了・loop()の先頭へ
    }

    //----------------------------------------------
    // 更新時間の設定
    //----------------------------------------------
    if(strncmp(s,"GET /?INT=",10)==0){                        // 更新時間の設定命令を受けた時
        update = atoi(&s[10]);                                // 受信値を変数updateに代入
    }

    //----------------------------------------------
    // コマンド送信
    //----------------------------------------------
    for(i=6;i<strlen(s);i++) if(s[i]==' '||s[i]=='+') s[i]='\0';
    htmlMesg(client,&s[6],WiFi.softAPIP());                   // メッセージ表示
    client.flush();                                           // ESP32用 ERR_CONNECTION_RESET対策
//    client.stop();                                          // クライアント切断
    Serial.println("Sent HTML");                              // シリアル出力表示
}
