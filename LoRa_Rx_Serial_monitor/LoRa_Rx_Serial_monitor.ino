//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : LoRa Rx
//     Processor    : ATmega328P (3.3V /8MHz)
//     Application  : transmit sensor data on LoRa communication
//
//     Leaf configuration
//       (1) AC03 LoRa Easy
//       (2) AI01 4-Sensors
//       (3) AP01 AVR MCU
//
//		(c) 2020  Trillion-Node Study Group
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2019/08/01  First release
//=====================================================================

//=====================================================================
// difinition
//=====================================================================
#include <MsTimer2.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <HTS221.h>
#include <ClosedCube_OPT3001.h>

#include <SoftwareSerial.h>

// --------------------------------------------
#define PCTX   0
#define PCRX   1
#define INT0   2
#define INT1   3
#define LED    4     // Change from SLEEP to LED
#define SLEEP  4
#define CN3_D5 5
#define DISCN  6
#define BLSLP  7

#define LORATX  8
#define LORARX  9
#define SS      10
#define MOSI    11
#define MISO    12
#define SCK     13

#define CN2_D14 14
#define CN2_D15 15
#define WFTX    16
#define WFRX    17
#define SDA     18
#define SCL     19

#define CPULED 4
#define DBGLED 14

//-----------------------------------------------
// LoRa
//-----------------------------------------------
#define TIMER_CHECK 1000    // 1s
#define TIMEOUT     20      // TIMER_CHECK x 20 = 20s
#define RX_MAX      64      // max 64 byte


//=====================================================================
// object
//=====================================================================
//-----------------------------------------------
// BLE
//-----------------------------------------------
SoftwareSerial lora(LORARX, LORATX);

//=====================================================================
// RAM data
//=====================================================================
//---------------------------
// LoRa
//---------------------------
String  rxData;
String  inputString;
uint8_t analysisStage;
uint8_t receiveCheckCnt;

//=====================================================================
// setup
//=====================================================================
//-----------------------------------------------
// port
//-----------------------------------------------
void setupPort(){

  pinMode(INT0,INPUT);          // PD2 : digital 2 = BLE interrupt
  pinMode(INT1,INPUT);          // PD3 : digital 3 = sensor interrupt

  pinMode(LED,OUTPUT);          // PD4 : digital 4 = LED
  digitalWrite(LED,LOW);

  pinMode(CN3_D5,OUTPUT);       // PD5 : digital 5 = not used
  digitalWrite(CN3_D5,LOW);

  pinMode(DISCN,OUTPUT);        // PD6 : digital 6 = BLE disconnect
  digitalWrite(DISCN,HIGH);

  pinMode(BLSLP, OUTPUT);       // PD7 : digital 7 = BLE sleep
  digitalWrite(BLSLP, HIGH);


  pinMode(SS,OUTPUT);           // PB2 : digital 10 = not used
  digitalWrite(SS,LOW);

  pinMode(MOSI,OUTPUT);         // PB3 : digital 11 = not used
  digitalWrite(MOSI,LOW);

  pinMode(MISO,OUTPUT);         // PB4 : digital 12 = not used
  digitalWrite(MISO,LOW);

  pinMode(SCK,OUTPUT);          // PB5 : digital 13 =LED on 8bit-Dev. Leaf
  digitalWrite(SCK,LOW);

  pinMode(CN2_D14,OUTPUT);       // PC0 : digital 14 = not used
  digitalWrite(CN2_D14,LOW);

  pinMode(CN2_D15,OUTPUT);       // PC1 : digital 15 = not used
  digitalWrite(CN2_D15,LOW);

  pinMode(WFTX,OUTPUT);          // PC2 : digital 16  = not used
  digitalWrite(WFTX,LOW);

  pinMode(WFRX,OUTPUT);          // PC3 : digital 17  = not used
  digitalWrite(WFRX,LOW);


  pinMode(CPULED, OUTPUT);
  digitalWrite(CPULED, LOW);

  pinMode(DBGLED, OUTPUT);
  digitalWrite(DBGLED, LOW);
}

//-----------------------------------------------
// external interrupt
//-----------------------------------------------
void setupExtInt(){
}

//-----------------------------------------------
// timer2 interrupt (interval=125ms, int=overflow)
//-----------------------------------------------
void setupTC2Int(){

  MsTimer2::set(TIMER_CHECK, timeout);
}

//-----------------------------------------------
// watchdog tiner INT (interval=8s, int=overflow)
//-----------------------------------------------
void setupWdtInt(){
  MCUSR = 0;
  WDTCSR |= 0b00011000;               //WDCE WDE set
  WDTCSR =  0b01000000 | 0b100001;    //WDIE set, WDIF set  scale 8 seconds
}

//-----------------------------------------------
// LoRa
//-----------------------------------------------
void setupLoRa() {
  lora.begin(9600);             // UART speed = 9600bps
}

//=====================================================================
// interrupt
//=====================================================================
//---------------------------------------------
// Watchdog Timer INT
//---------------------------------------------
ISR(WDT_vect){
}

//---------------------------------------------
// LoRa Timeout
//---------------------------------------------
void timeout() {
  if (receiveCheckCnt >= TIMEOUT) {
    receiveCheckCnt = 0;
    Serial.println("Receive Time Out!!");
  }
  else {
    receiveCheckCnt++;
  }
}

//====================================================================
// functions
//====================================================================

//====================================================================
// setup
//====================================================================
void setup() {

  Serial.begin(9600);       // UART 9600bps

  Wire.begin();              // I2C 100KHz

  Serial.println("Receive Start!");

  setupPort();
  delay(10);

  receiveCheckCnt = 0;

  noInterrupts();
  setupExtInt();
  setupTC2Int();
  setupWdtInt();
  interrupts();

  setupLoRa();

  MsTimer2::start();      // Timer2 inverval start

  rxData = "";
  inputString = "";
  analysisStage = 0;
}

//====================================================================
// LoRa
//====================================================================
void loopLoRa()
{
  if (lora.available()) {
    char c = lora.read();

    if (analysisStage == 0) {
      if (c == '{') {
        inputString += c;
        analysisStage = 1;
      }
    }
    else if (analysisStage == 1) {
      inputString += c;
      if (c == '}') {
        Serial.println(inputString);
        inputString = "";
        analysisStage = 0;
        receiveCheckCnt = 0;
      }
    }
    else {
      inputString = "";
      analysisStage = 0;
    }
  }
}

//====================================================================
// loop
//====================================================================
void loop() {

  //--------------------------------------------
  // loop LoRa
  //--------------------------------------------
  loopLoRa();
}
