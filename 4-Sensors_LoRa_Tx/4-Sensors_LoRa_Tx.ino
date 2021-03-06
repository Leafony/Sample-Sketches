//=====================================================================
//  Leafony Platform sample sketch
//     Platform     : LoRa Tx
//     Processor    : ATmega328P (3.3V /8MHz)
//     Application  : transmit sensor data on LoRa communication
//
//     Leaf configuration
//       (1) AC03 LoRa Easy
//       (2) AI01 4-Sensors
//       (3) AP01 AVR MCU
//
//		(c) 2021 LEAFONY SYSTEMS Co., Ltd
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01 First release
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

//-----------------------------------------------
// loop interval
//-----------------------------------------------
#define LOOP_INTERVAL 125         // 125ms interval

//-----------------------------------------------
// number of sensor's average data
//-----------------------------------------------
#define SENSOR_SAMPLE 4         // sampling number

//-----------------------------------------------
// Sleep Transition
//-----------------------------------------------
#define SLEEP_INTERVAL 3         // 1s x 9 = 9s

//-----------------------------------------------
// I2C
//-----------------------------------------------
#define LIS2DH_ADDRESS  0x19
#define OPT3001_ADDRESS 0x45

//-----------------------------------------------
// LIS2DH
//-----------------------------------------------
#define DIVIDER_2G 16383          // full scale 2G  (=0xFFFF/4)
#define DIVIDER_4G 8191           // full scale 4G  (=0xFFFF/4/2)
#define DIVIDER_8G 4096           // full scale 8G  (=0xFFFF/4/4)
#define DIVIDER_16G 1365          // full scale 16G (=0xFFFF/4/12)


//=====================================================================
// object
//=====================================================================
//-----------------------------------------------
// Sensor
//-----------------------------------------------
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;

//-----------------------------------------------
// LoRa
//-----------------------------------------------
SoftwareSerial lora(LORARX, LORATX);


//=====================================================================
// RAM data
//=====================================================================
//---------------------------
// loop counter
//---------------------------
uint8_t iLoop1s = 0;
uint8_t iLoopSample = 0;

//---------------------------
// event
//---------------------------
boolean eventSleepCheck = 0;
boolean eventSensorRead = 0;

//---------------------------
// interval Timer2 interrupt
//---------------------------
volatile boolean bInterval = 0;

//---------------------------
// Sleep, Watchdog Timer
//---------------------------
volatile boolean bSleep = 0;
volatile boolean eventWakeupWDT = 0;

volatile int countWDT = 0;
volatile int wakeupWDT = SLEEP_INTERVAL;

//---------------------------
// LIS2DH : accelerometer
//---------------------------
int16_t dataX, dataY, dataZ;
float dataX_g, dataY_g, dataZ_g;
float dataTiltX;
float dataTiltY;
float dataTiltZ;

//---------------------------
// HTS221 : Temperature/Humidity
//---------------------------
float dataTemp;
float dataHumid;

//---------------------------
// Data for two-point correction
//---------------------------
// Temperature correction data 0
float TL0 = 25.0;     // 4-Sensors Temperature measurement value
float TM0 = 25.0;     // Thermometer and other measurements value
// Temperature correction data 1
float TL1 = 40.0;     // 4-Sensors Temperature measurement value
float TM1 = 40.0;     // Thermometer and other measurements value

// Humidity correction data 0
float HL0 = 60.0;     // 4-Sensors Humidity measurement value
float HM0 = 60.0;     // Hygrometer and other measurements value
// Humidity correction data 1
float HL1 = 80.0;     // 4-Sensors Humidity measurement value
float HM1 = 80.0;     // Hygrometer and other measurements value

//---------------------------
// OPT3001 : Light
//---------------------------
float dataLight;

volatile boolean bWakeupINT0 = 0;
volatile boolean bWakeupINT1 = 0;

//---------------------------
// LoRa
//---------------------------
char trans[20];

//=====================================================================
// setup
//=====================================================================
//-----------------------------------------------
// port
//-----------------------------------------------
void setupPort(){

  pinMode(INT0, INPUT);         // PD2 : digital 2 = BLE interrupt
  pinMode(INT1, INPUT);         // PD3 : digital 3 = sensor interrupt

  pinMode(SLEEP, OUTPUT);       // PD4 : digital 4 = SLEEP
  digitalWrite(SLEEP, HIGH);

  pinMode(CN3_D5, OUTPUT);      // PD5 : digital 5 = not used
  digitalWrite(CN3_D5, LOW);

  pinMode(DISCN, OUTPUT);       // PD6 : digital 6 = BLE disconnect
  digitalWrite(DISCN, HIGH);

  pinMode(BLSLP, OUTPUT);       // PD7 : digital 7 = BLE sleep
  digitalWrite(BLSLP, HIGH);


  pinMode(SS, OUTPUT);          // PB2 : digital 10 = not used
  digitalWrite(SS, LOW);

  pinMode(MOSI, OUTPUT);        // PB3 : digital 11 = not used
  digitalWrite(MOSI, LOW);

  pinMode(MISO, OUTPUT);        // PB4 : digital 12 = not used
  digitalWrite(MISO, LOW);

  pinMode(SCK, OUTPUT);         // PB5 : digital 13 =LED on 8bit-Dev. Leaf
  digitalWrite(SCK, LOW);

  pinMode(CN2_D14, OUTPUT);      // PC0 : digital 14 = not used
  digitalWrite(CN2_D14, LOW);

  pinMode(CN2_D15, OUTPUT);      // PC1 : digital 15 = not used
  digitalWrite(CN2_D15, LOW);

  pinMode(WFTX, OUTPUT);         // PC2 : digital 16  = not used
  digitalWrite(WFTX, LOW);

  pinMode(WFRX, OUTPUT);         // PC3 : digital 17  = not used
  digitalWrite(WFRX, LOW);

}

//-----------------------------------------------
// external interrupt
//-----------------------------------------------
void setupExtInt(){

  detachInterrupt(1);           // BLE    INT0# = disabled
  detachInterrupt(1);           // sensor INT1# = disabled
}

//-----------------------------------------------
// timer2 interrupt (interval=125ms, int=overflow)
//-----------------------------------------------
void setupTC2Int(){

  MsTimer2::set(LOOP_INTERVAL, intTimer2);
}

//-----------------------------------------------
// watchdog tiner INT (interval=8s, int=overflow)
//-----------------------------------------------
void setupWdtInt(){

  MCUSR = 0;
  WDTCSR |= 0b00011000;               //WDCE WDE set
  WDTCSR =  0b01000000 | 0b000110;    //WDIE set,  scale 1 seconds
}

//=====================================================================
// interrupt
//=====================================================================
//----------------------------------------------
// Timer2 INT
//----------------------------------------------
void intTimer2(){

  bInterval = 1;
}

//---------------------------------------------
// Watchdog Timer INT
//---------------------------------------------
ISR(WDT_vect){

  if(bSleep == 1){

    countWDT += 1;

    if(countWDT >= wakeupWDT){

      countWDT = 0;

      bSleep = 0;
      eventWakeupWDT = 1;
    }
  }
}

//-----------------------------------------------
// sensor
//-----------------------------------------------
void setupSensor() {

  //-------------------------------------
  // LIS2DH (accelerometer)
  //-------------------------------------
  //-------------------
  // I2C address
  //------------------
  accel.begin(LIS2DH_ADDRESS);

  //-------------------
  // register
  //-------------------
  accel.setClick(0, 0);                      // Disable interrupt to save power
  accel.setRange(LIS3DH_RANGE_2_G);          // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_1_HZ);   // Data rate = 1Hz

  /*
    //-------------------
    // INT1 active
    //-------------------
    accel.writeRegister8(LIS3DH_REG_CTRL3, 0x40);    // INT1 = wakeup event
    accel.writeRegister8(LIS3DH_REG_CTRL5, 0x00);    // INT1 = no latch
    accel.writeRegister8(LIS3DH_REG_INT1THS, 0x37);  // Threshold = 866mG /2G x 128 =0x37
    accel.writeRegister8(LIS3DH_REG_INT1DUR, 0x01);  // INT1 active  occurs after 1000ms
    accel.writeRegister8(LIS3DH_REG_INT1CFG, 0x0A);  // Threshold = X or Y axis high
  */

  //-------------------------------------
  // HTS221 (temperature / humidity)
  //-------------------------------------
  smeHumidity.begin();

  //-------------------------------------
  // OPT3001 (light)
  //-------------------------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  //-------------------
  // I2C address
  //-------------------
  light.begin(OPT3001_ADDRESS);

  //-------------------
  // config register
  //-------------------
  newConfig.RangeNumber = B1100;               // automatic full scale
  newConfig.ConvertionTime = B1;               // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;   // continous conversion
  newConfig.Latch = B0;                        // hysteresis-style

  errorConfig = light.writeConfig(newConfig);

  if (errorConfig != NO_ERROR) {

    errorConfig = light.writeConfig(newConfig);   // retry
  }

  /*
    //-------------------
    // INT1 active = low
    //  1)over 2,000 lux
    //  2)high limit
    //    LE[3:0] = 0110b = 6
    //    T[11:0] = C35h = 3125
    //  ->  0.01 x 2^6 x 3125 = 2,000[lux]
    //-------------------
    Wire.beginTransmission(OPT3001_ADDRESS);
    Wire.write(HIGH_LIMIT);
    Wire.write(0x6C);
    Wire.write(0x35);
    Wire.endTransmission();
    OPT3001 result = light.readHighLimit();

    //-------------------
    // INT1 inactive = high
    //  1)under 1,500 lux
    //  2)low limit
    //    LE[3:0] = 0110b = 6
    //    T[11:0] = 927h = 2343
    //  ->  0.01 x 2^6 x 2343 = 1,500[lux]
    //-------------------
    Wire.beginTransmission(OPT3001_ADDRESS);
    Wire.write(LOW_LIMIT);
    Wire.write(0x69);
    Wire.write(0x27);
    Wire.endTransmission();
    result = light.readLowLimit();
  */
}

//-----------------------------------------------
// LoRa
//-----------------------------------------------
void setupLoRa() {
  lora.begin(9600);             // UART speed = 9600bps
}


//====================================================================
// functions
//====================================================================
//---------------------------------------
// sleep LoRa
//---------------------------------------
void sleepLoRa() {

  delay(100);
  digitalWrite(SLEEP, HIGH);
}

//---------------------------------------
// wakeup LoRa
//---------------------------------------
void wakeupLoRa() {

  digitalWrite(SLEEP, LOW);
  delay(100);
}

//--------------------------------------------------------------------
// sensor
//--------------------------------------------------------------------
//-----------------------------------------
// main loop
//-----------------------------------------
void sensor_read() {

  //-------------------------
  // LIS2DH
  //-------------------------
  accel.read();
  dataX_g = accel.x_g;
  dataY_g = accel.y_g;
  dataZ_g = accel.z_g;

  if (dataX_g >= 1.0) {

    dataX_g = 1.00;
  }
  else if (dataX_g <= -1.0) {

    dataX_g = -1.00;
  }

  if (dataY_g >= 1.0) {

    dataY_g = 1.00;
  }
  else if (dataY_g <= -1.0) {

    dataY_g = -1.00;
  }

  if (dataZ_g >= 1.0) {

    dataZ_g = 1.00;
  }
  else if (dataZ_g <= -1.0) {

    dataZ_g = -1.00;
  }

  dataTiltX = (float)(dataX_g * 180);
  dataTiltY = (float)(dataY_g * 180);
  dataTiltZ = (float)(dataZ_g * 180);

  //-------------------------
  // HTS221
  //-------------------------
  dataTemp = (float)smeHumidity.readTemperature();
  dataHumid = (float)smeHumidity.readHumidity();

    //-------------------------
    // Two-point correction for temperature and humidity
    //-------------------------
    dataTemp=TM0+(TM1-TM0)*(dataTemp-TL0)/(TL1-TL0);        // Temperature correction
    dataHumid=HM0+(HM1-HM0)*(dataHumid-HL0)/(HL1-HL0);      // Humidity correction

  //-------------------------
  // OPT3001
  //-------------------------
  OPT3001 result = light.readResult();

  if (result.error == NO_ERROR) {

    dataLight = result.lux;
  }
  else {

    dataLight = 0;
  }

    //-------------------------
    // Serial monitor display
    //-------------------------
  Serial.println("");
  Serial.println("--- sensor average data ---");
  Serial.println("  Temp[degC] = " + String(dataTemp));
  Serial.println("  Humid[RH%] = " + String(dataHumid));
  Serial.println("  Light[lux] = " + String(dataLight));
  Serial.println("  Tilt[deg]  = " + String(dataTiltX));
  Serial.println("");

  memset(trans, 0, 20);
  char *json = &trans[0];

  int dataT = (int)dataTemp;
  int dataH = (int)dataHumid;
  int dataL = (int)dataLight;
  int dataX = (int)dataTiltX;

  sprintf(json, "{\"t\":%d, \"h\":%d, \"l\":%d, \"x\":%d}", dataT, dataH, dataL, dataX);
  Serial.println(json);

  wakeupLoRa();

  // LoRa Data transmission
  lora.println(json);

  sleepLoRa();
}


//-----------------------------------------
// sleep sensor
//-----------------------------------------
void sleepSensor() {

  //-----------------------
  // OPT3001 sleep
  //-----------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  newConfig.ModeOfConversionOperation = B00;
  errorConfig = light.writeConfig(newConfig);
  if (errorConfig != NO_ERROR) {

    errorConfig = light.writeConfig(newConfig);
  }

  //-----------------------
  // LIS3DH sleep
  //-----------------------
  accel.setDataRate(LIS3DH_DATARATE_POWERDOWN);

  //-----------------------
  // HTS221 sleep
  //-----------------------
  smeHumidity.deactivate();
}

//-----------------------------------------
// wakeup sensor
//-----------------------------------------
void wakeupSensor() {

  //-----------------------
  // OPT3001 wakeup
  //-----------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  newConfig.RangeNumber = B1100;               //automatic full scale
  newConfig.ConvertionTime = B1;               //convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;   //continous conversion
  newConfig.Latch = B1;                        //latch window style

  errorConfig = light.writeConfig(newConfig);
  if (errorConfig != NO_ERROR) {

    errorConfig = light.writeConfig(newConfig);   //retry
  }
  delay(1000);

  //-----------------------
  // LIS2DH wakeup
  //-----------------------
  accel.setDataRate(LIS3DH_DATARATE_1_HZ);

  //-----------------------
  // HTS221 wakeup
  //-----------------------
  smeHumidity.activate();
}

//--------------------------------------------------------------------
// counter /event
//--------------------------------------------------------------------
//-----------------------------------------
// main loop
//-----------------------------------------
void loopCounter(){

  iLoop1s += 1;

  //--------------------
  // 1s period
  //--------------------
  if(iLoop1s == 8){               // 128ms x 8 = 1s

    iLoop1s = 0;

    eventSleepCheck = 1;
  }
}

//-----------------------------------------------
// sleep
//-----------------------------------------------
void loopSleep(){

  if(eventSleepCheck == 1){

    eventSleepCheck = 0;

    wdt_reset();
    bSleep = 1;
    countWDT = 0;

    Serial.println("");
    Serial.print("  >>> Go to sleep :  ");
    Serial.println("wakeup after = " + String(wakeupWDT) + " x 1s  >>>");
    Serial.println("");

    //-----------------------
    // flux buffer
    //-----------------------
    Serial.flush();

    //-----------------------
    // ATMega328 sleep
    //-----------------------
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);        // ATMega328 power down mode

    while(bSleep == 1){

      ADCSRA &= ~(1 << ADEN);                  // ADC = OFF in sleep
      noInterrupts();
      sleep_enable();
      MCUCR = bit (BODS) | bit (BODSE);        // BOD = OFF in sleep
      MCUCR = bit (BODS);
      interrupts();
      sleep_cpu();                             // Enter sleep

      sleep_disable();                         // Exit sleep
    }

    //-----------------------
    // ATMega328 wakeup
    // BOD is automatically enabled at wakeup
    //-----------------------
    ADCSRA |= (1 << ADEN);                     // ADC = ON

    //------------------------
    // resume
    //------------------------
    Serial.println("  <<< Wake up <<<");
    Serial.println("");

    //------------------------
    // wakeup sensor
    //------------------------
    wakeupSensor();

    //------------------------
    // sensor read
    //------------------------
    sensor_read();

    //------------------------
    // sleep sensor
    //------------------------
    sleepSensor();
  }
}

//====================================================================
// setup
//====================================================================
void setup() {

  Serial.begin(9600);       // UART 9600bps
  Wire.begin();              // I2C 100KHz

  Serial.println("=========================================");
  Serial.println("setup start");

  setupPort();
  delay(10);

  noInterrupts();
  setupExtInt();
  setupTC2Int();
  setupWdtInt();
  interrupts();

  setupSensor();
  setupLoRa();

  MsTimer2::start();      // Timer2 inverval start

  Serial.println("");
  Serial.println("=========================================");
  Serial.println("loop start");
  Serial.println("");
}

//====================================================================
// loop
//====================================================================
void loop() {

  //-----------------------------------------------------
  // Timer2 interval
  //-----------------------------------------------------
  if (bInterval == 1){

    bInterval = 0;

    //--------------------------------------------
    // loop counter /event
    //--------------------------------------------
    loopCounter();

    //--------------------------------------------
    // sleep/resume
    //--------------------------------------------
    loopSleep();
  }

}
