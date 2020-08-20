//=====================================================================
//  Sound level meter
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/05/17  First release
//=====================================================================
#include <Wire.h>

#define AVR_LED         13
#define LED1             6
#define LED2             7
#define LED3             8
#define LED4             9
#define LED5            10
#define LED6            11

#define MIC         16
#define VR          17

unsigned int dataVR;
unsigned int dataMic;

//-----------------------------------------------
// VRとMICデータを取得する
//-----------------------------------------------
void getSensorData()
{
    dataVR = analogRead(A3);
    dataMic = analogRead(A2);

    Serial.println("---------------------");
    Serial.print("MIC  = ");
    Serial.println(dataMic,DEC);
    Serial.print("VR   = ");
    Serial.println(dataVR,DEC);
}

//-----------------------------------------------
// VRとMICの値にあわせてLEDを点灯、消灯させる
//-----------------------------------------------
void dispSensorData()
{
    if (dataVR < 400)
    {
      digitalWrite(LED4,LOW);
      digitalWrite(LED5,LOW);
      digitalWrite(LED6,LOW);
    }
    else if (dataVR >= 400 && dataVR < 800)
    {
       digitalWrite(LED4,LOW);
       digitalWrite(LED5,LOW);
       digitalWrite(LED6,HIGH);
    }
    else if (dataVR >= 800 && dataVR < 900)
    {
       digitalWrite(LED4,LOW);
       digitalWrite(LED5,HIGH);
       digitalWrite(LED6,HIGH);
    }
    else
    {
       digitalWrite(LED4,HIGH);
       digitalWrite(LED5,HIGH);
       digitalWrite(LED6,HIGH);
    }

    if (dataMic < 700)
    {
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,LOW);
    }
    else if (dataMic >= 700 && dataMic < 800)
    {
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,HIGH);
    }
    else if (dataMic >= 800 && dataMic < 850)
    {
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,HIGH);
    }
    else
    {
      digitalWrite(LED1,HIGH);
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,HIGH);
    }
}


void setup() {
  Serial.begin(115200);
  Serial.println("start!!");

  pinMode(AVR_LED, OUTPUT);
  digitalWrite(AVR_LED, LOW);

  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED3, LOW);
  pinMode(LED4, OUTPUT);
  digitalWrite(LED4, LOW);
  pinMode(LED5, OUTPUT);
  digitalWrite(LED5, LOW);
  pinMode(LED6, OUTPUT);
  digitalWrite(LED6, LOW);

  pinMode(MIC, INPUT);
  pinMode(VR, INPUT);

  //起動テスト(LED)
  digitalWrite(LED1,HIGH);
  delay(200);
  digitalWrite(LED2,HIGH);
  delay(200);
  digitalWrite(LED3,HIGH);
  delay(200);
  digitalWrite(LED4,HIGH);
  delay(200);
  digitalWrite(LED5,HIGH);
  delay(200);
  digitalWrite(LED6,HIGH);
  delay(200);
  digitalWrite(LED1,LOW);
  digitalWrite(LED2,LOW);
  digitalWrite(LED3,LOW);
  digitalWrite(LED4,LOW);
  digitalWrite(LED5,LOW);
  digitalWrite(LED6,LOW);
  delay(200);
}


void loop() {
  getSensorData();
  dispSensorData();
  delay(100);
}