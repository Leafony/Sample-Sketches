//=====================================================================
//  ESP32 Sound level meter
//
//    (c) 2020 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2020/08/20  First release
//=====================================================================

// LED4-6:VR level display
// LED1-3:MIC sound pressure level display

#define LED1              14            // D6  IO14
#define LED2              15            // D7  IO15
#define LED3              17            // D8  IO17
#define LED4              16            // D9  IO16
#define LED5              5             // D10 IO5
#define LED6              23            // D11 IO23
#define MIC               26            // A2  IO26
#define VR                39            // A3  IO39

unsigned int dataVR;
unsigned int dataMic;

//-----------------------------------------------
// Get VR and MIC data
//-----------------------------------------------
void getSensorData(){
    dataVR = analogRead(VR);
    dataMic = analogRead(MIC);

    Serial.println("---------------------");
    Serial.print("MIC  = ");
    Serial.println(dataMic,DEC);
    Serial.print("VR   = ");
    Serial.println(dataVR,DEC);
}

//-----------------------------------------------
// Turn the LEDs on and off according to the VR and MIC values.
//-----------------------------------------------
void dispSensorData(){
    if (dataVR < 1500){
      digitalWrite(LED4,LOW);
      digitalWrite(LED5,LOW);
      digitalWrite(LED6,LOW);
    }
    else if (dataVR >= 1500 && dataVR < 2600){
       digitalWrite(LED4,LOW);
       digitalWrite(LED5,LOW);
       digitalWrite(LED6,HIGH);
    }
    else if (dataVR >= 2600 && dataVR < 3600){
       digitalWrite(LED4,LOW);
       digitalWrite(LED5,HIGH);
       digitalWrite(LED6,HIGH);
    }
    else{
       digitalWrite(LED4,HIGH);
       digitalWrite(LED5,HIGH);
       digitalWrite(LED6,HIGH);
    }

    if (dataMic < 400){
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,LOW);
    }
    else if (dataMic >= 400 && dataMic < 600){
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,HIGH);
    }
    else if (dataMic >= 600 && dataMic < 950){
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,HIGH);
    }
    else{
      digitalWrite(LED1,HIGH);
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,HIGH);
    }
}


void setup(){
  Serial.begin(115200);
  Serial.println("start!!");

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
