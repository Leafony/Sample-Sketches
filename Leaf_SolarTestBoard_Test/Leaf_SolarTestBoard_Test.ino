//-----------------------------------------------------------------
//  Solar Battery + Test board
//
//    2025-03-19
//    Koji Nakamura
//-----------------------------------------------------------------

//=====================================================================
// difinition
//=====================================================================
#include <Wire.h>

#define TEST_N       D2        // test pin (STM32_PC7) I/F=22pin

#define I2C_FREQ_HZ  400000   

char c;
const uint8_t KTD_ADDRESS = 0x30;
const uint8_t TCA_ADDRESS = 0x41;
const uint8_t ADC_ADDRESS = 0x50;

uint8_t regData;
uint8_t reg2Byte[2];

uint8_t advalue;

//====================================================================
// function
//====================================================================

//-----------------------------------------------
// KTD2026
//-----------------------------------------------
//--------------------------
// Start blinking
//--------------------------
void blinkStart(){

  //Serial.println("======================");
  Serial.println("KTD2026: LED start");
  //Serial.println("");

  i2c_write_byte(KTD_ADDRESS, 0x00, 0x07);    // Reset chip
  delay(2);

  i2c_write_byte(KTD_ADDRESS, 0x00, 0x18);    // chip = always ON
  i2c_write_byte(KTD_ADDRESS, 0x01, 0x0E);    // Flash period = 2.048s
  i2c_write_byte(KTD_ADDRESS, 0x02, 0x19);    // Flash ON Timer1 Duty=9.7602%
  i2c_write_byte(KTD_ADDRESS, 0x06, 0x9F);    // LED current = 20mA
  i2c_write_byte(KTD_ADDRESS, 0x04, 0x02);    // LED1 = PWM1
}

//--------------------------
// Stop blinking
//--------------------------
void blinkStop(){

  //Serial.println("======================");
  Serial.println("KTD2026: LED stop");
  //Serial.println("");

  i2c_write_byte(KTD_ADDRESS, 0x04, 0x00);    // LED1 = always off
  i2c_write_byte(KTD_ADDRESS, 0x00, 0x08);    // chip = shutdown
}

//-----------------------------------------------
// TCA9536
//-----------------------------------------------
//--------------------------
// Initialize
//--------------------------
void initTCA(){

  //Serial.println("======================");
  Serial.println("TCA: initialize");
  //Serial.println("");

  i2c_write_byte(TCA_ADDRESS, 0x02, 0x00);    // polarity
  i2c_write_byte(TCA_ADDRESS, 0x50, 0x40);    // P3 = GPIO
  i2c_write_byte(TCA_ADDRESS, 0x01, 0xF3);    // SHDN=Low, ONVCC=Low
  i2c_write_byte(TCA_ADDRESS, 0x03, 0xF3);    // GPIO
}

//--------------------------
// Read register
//--------------------------
void readTCA(){

  //Serial.println("======================");
  Serial.println("TCA : Read register");
  //Serial.println("");

  regData = i2c_read_byte(TCA_ADDRESS, 0x00);
  Serial.println("  Addr_0x00 = "+String(regData,HEX));

  regData = i2c_read_byte(TCA_ADDRESS, 0x01);
  Serial.println("  Addr_0x01 = "+String(regData,HEX));

  regData = i2c_read_byte(TCA_ADDRESS, 0x02);
  Serial.println("  Addr_0x02 = "+String(regData,HEX));

  regData = i2c_read_byte(TCA_ADDRESS, 0x03);
  Serial.println("  Addr_0x03 = "+String(regData,HEX));

  regData = i2c_read_byte(TCA_ADDRESS, 0x50);
  Serial.println("  Addr_0x50 = "+String(regData,HEX));
}

//--------------------------
// ONVCC = L -> H
//--------------------------
void onVCC(){

  //Serial.println("======================");
  Serial.println("TCA: ONVCC = High");
  //Serial.println("");

  i2c_write_byte(TCA_ADDRESS, 0x01, 0xF7);    // ONVCC=High
}

//--------------------------
// ONVCC = H -> L
//--------------------------
void offVCC(){

  //Serial.println("======================");
  Serial.println("TCA: ONVCC = Low");
  //Serial.println("");

  i2c_write_byte(TCA_ADDRESS, 0x01, 0xF3);    // ONVCC=Low
}

//--------------------------
// SHDN = L -> H
//--------------------------
void shutDown(){

  //Serial.println("======================");
  Serial.println("TCA: SHDN = High");
  //Serial.println("");

  i2c_write_byte(TCA_ADDRESS, 0x01, 0xFB);    // SHDN=High
}

//-----------------------------------------------
// ADC101C
//-----------------------------------------------
//--------------------------
// Initialize
//--------------------------
void initADC(){

  //Serial.println("======================");
  Serial.println("ADC: initialize");
  //Serial.println("");

  i2c_write_byte(ADC_ADDRESS, 0x02, 0x00);    // P3 = GPIO
}

//--------------------------
// AD convert
//--------------------------
void convADC(){

  uint16_t ad,ad0,ad1;
  float vbat;

  //Serial.println("======================");
  Serial.println("ADC: A/D convert");
  //Serial.println("");

  i2c_write_byte(ADC_ADDRESS, 0x02, 0xE0);    // ADC rate = 0.4ks/s
  delay(100);

  i2c_read(ADC_ADDRESS, 0x00, reg2Byte);      // Read address 0x00 (2byte)

  ad0 = reg2Byte[0];
  ad1 = reg2Byte[1];
  ad = (ad0<<8) | ad1;
  ad = ad>>2;

  vbat = ad*6.59/1023;

  Serial.println("  ADC = "+String(ad,HEX));
  Serial.println("  Vbat = "+String(vbat,3));
  Serial.println("");

  i2c_write_byte(ADC_ADDRESS, 0x02, 0x00);    // ADC stop
}

//--------------------------
// test board 
//--------------------------
void testBoard(){

  advalue = digitalRead(TEST_N);

  if(advalue == 0){

    Serial.println("TEST_N = Low");
  }
  else{

    Serial.println("TEST_N = High");
  }
}


//-----------------------------------------------
// Menu
//-----------------------------------------------
void putMenu(){
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("m = This Menu"));
  Serial.println(F("1 = KTD: LED start"));
  Serial.println(F("2 = KTD: LED stop"));
  Serial.println(F("3 = TCA: Initialize"));
  Serial.println(F("4 = TCA: Read register"));
  Serial.println(F("5 = TCA: ONVCC_High"));
  Serial.println(F("6 = TCA: ONVCC_Low"));
  Serial.println(F("7 = ADC: Initialize"));
  Serial.println(F("8 = ADC: AD convert"));
  Serial.println(F("9 = TestBoard: TEST_N signal"));
  Serial.println(F("--------------------------------------------------"));

  Serial.println("");
}

//-----------------------------------------------
// I2C
//-----------------------------------------------
//-----------------------
// I2C Write 1 byte
//-----------------------
bool i2c_write_byte(uint8_t dev_address, uint8_t reg_address, uint8_t write_data){
  Wire.beginTransmission(dev_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  if (Wire.endTransmission() != 0)
  {
    return false;
  }
  else
  {
    return true;
  }
}

//------------------------
// I2C Read 1 byte
//------------------------
unsigned char i2c_read_byte(uint8_t dev_address, uint8_t reg_address)
{
  Wire.beginTransmission(dev_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);
  //request 1 byte from slave
  if(Wire.requestFrom(dev_address, 1, 1))
  {
    return Wire.read();
  }
  else
  {
    return 0;
  }
}

//------------------------
// I2C Read 2 byte
//------------------------
void i2c_read(uint8_t dev_address, uint8_t reg_address, unsigned char* read_byte){

  Wire.beginTransmission(dev_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(dev_address, 2, (uint8_t)false);
  for (int i = 0; i < 2; i++){
    read_byte[i] = Wire.read();
  }

  Wire.endTransmission(true);
}

//====================================================================
// setup
//====================================================================
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(TEST_N, INPUT);

  Serial.println("*******************************");
  Serial.println("Solar Battery");
  Serial.println("");
  Serial.println("======================");
  Serial.println("Power Switch = ON");
  Serial.println("");
  Serial.println("======================");
  Serial.println("Initialization");
  Serial.println("");

  //-----------------------------------------------------
  // Initialize the i2c interface
  //-----------------------------------------------------
  Wire.begin();
  Wire.setClock(I2C_FREQ_HZ);

  //-----------------------------------------------------
  // KTD: LED blink start
  //-----------------------------------------------------
  blinkStart();

  //-----------------------------------------------------
  // TCA: initialize
  //-----------------------------------------------------
  initTCA();

  //-----------------------------------------------------
  // ADC: initialize
  //-----------------------------------------------------
   initADC();
   Serial.println("");
   Serial.println("finished");
   Serial.println("======================");
   Serial.println("");

}


//====================================================================
// loop
//====================================================================
void loop() {


  if (Serial.available()){

    c = Serial.read();

    //----------------------
    // KTD: LED blink start
    //----------------------
    if(c == '1'){

      blinkStart();
      Serial.println("");
    }
    //----------------------
    // KTD: LED blink stop
    //----------------------
    else if(c == '2'){

      blinkStop();
      Serial.println("");
    } 
    //----------------------
    // TCA: Initialize
    //----------------------
    else if(c == '3'){

      initTCA();
      Serial.println("");
    } 
    //----------------------
    // TCA: Read register
    //----------------------
    else if(c == '4'){

      readTCA();
      Serial.println("");
    } 
    //----------------------
    // TCA: ONVCC=High
    //----------------------
    else if(c == '5'){

      onVCC();
      Serial.println("");
    } 
    //----------------------
    // TCA: ONVCC=Low
    //----------------------
    else if(c == '6'){

      offVCC();
      Serial.println("");
    } 

    //----------------------
    // ADC: Initialize
    //----------------------
    else if(c == '7'){

      initADC();
      Serial.println("");
    }

    //----------------------
    // ADC: Initialize
    //----------------------
    else if(c == '8'){

      convADC();
      Serial.println("");
    }

    //----------------------
    // Test board
    //----------------------
    else if(c == '9'){

      testBoard();
      Serial.println("");
    }

    //----------------------
    // Menue
    //----------------------
    else if(c == 'm'){

      putMenu();
      Serial.println("");
    }
  }
}

