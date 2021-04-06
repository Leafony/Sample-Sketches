//=====================================================================
//  STM32 Sound level meter
//
//    (c) 2021 Trillion-Node Study Group
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/01/26  First release
//=====================================================================
#define LED1            PA8       // D6
#define LED2            PB12      // D7
#define LED3            PA9       // D8
#define LED4            PA10      // D9
#define LED5            PB6       // D10
#define LED6            PA7       // D11

#define MIC             PA1       // A2
#define VR              PB0       // A3

unsigned int dataVR;
unsigned int dataMic;

//-----------------------------------------------
// Get VR and MIC data
//-----------------------------------------------
void getSensorData()
{
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

    if (dataMic < 300)
    {
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,LOW);
    }
    else if (dataMic >= 300 && dataMic < 350)
    {
      digitalWrite(LED1,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,HIGH);
    }
    else if (dataMic >= 350 && dataMic < 450)
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
