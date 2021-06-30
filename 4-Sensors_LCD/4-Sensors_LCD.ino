//=====================================================================
//  Leafony Platform sample sketch
//     Application  : 4-Sensors with LCD
//     Processor    : ATmega328P (3.3V /8MHz)
//     Arduino IDE  : 1.8.13
//
//     Leaf configuration
//       (1) AI01 4-Sensors
//       (2) AI04 LCD
//       (3) AP01 AVR MCU
//       (4) AZ01 USB
//
//		(c)2021 LEAFONY SYSTEMS Co., Ltd
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01 First release
//      
//=====================================================================
//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <MsTimer2.h>                       // Timer
#include <Wire.h>                           // I2C

#include <Adafruit_LIS3DH.h>                // 3-axis accelerometer
#include <HTS221.h>                         // humidity and temperature sensor
#include <ClosedCube_OPT3001.h>             // Ambient Light Sensor
#include <ST7032.h>                         // LCD

//=====================================================================

//===============================================
// Output to serial monitor
//      #define SERIAL_MONITOR = With output
//    //#define SERIAL_MONITOR = Without output (Comment out)
//===============================================
#define SERIAL_MONITOR

//-----------------------------------------------
// Define constants to be used in the program
//-----------------------------------------------
//------------------------------
// I2C address
//------------------------------
#define LIS2DH_ADDRESS          0x19        // Accelerometer (SD0/SA0 pin = VCC)
#define OPT3001_ADDRESS         0x45        // Ambient Light Sensor (ADDR pin = VCC)
#define LCD_I2C_EXPANDER_ADDR   0x1A        // LCD I2C Expander
#define BATT_ADC_ADDR           0x50        // Battery ADC

//-----------------------------------------------
// loop interval
// Timer interrupt interval (ms)
//-----------------------------------------------
#define LOOP_INTERVAL 125                   // 125ms interval

//---------------------------------------------------------------------
// object
//---------------------------------------------------------------------
//------------------------------
// LCD
//------------------------------
  ST7032 lcd;

//------------------------------
// Sensor
//------------------------------
Adafruit_LIS3DH accel = Adafruit_LIS3DH();
ClosedCube_OPT3001 light;      

//---------------------------------------------------------------------
// Define variables to be used in the program
//---------------------------------------------------------------------
//---------------------------
// LCD
//---------------------------
int8_t lcdSendCount = 0;

//------------------------------
// Loop counter
//------------------------------
uint8_t iLoop1s = 0;

//------------------------------
// Event
//------------------------------
bool event1s = false;

//------------------------------
// interval Timer interrupt
//------------------------------
volatile bool bInterval = false;

//------------------------------
// LIS2DH : Accelerometer
//------------------------------
float dataX_g, dataY_g, dataZ_g;
float dataTilt;

//------------------------------
// HTS221 : Humidity and Temperature sensor
//------------------------------
float dataTemp;
float dataHumid;

//--------------------
// Data for two-point correction
//--------------------
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

//------------------------------
// OPT3001 : Ambient Light Sensor
//------------------------------
float dataLight;

//---------------------------
// Battery
//---------------------------
float dataBatt = 0;

//=====================================================================
// setup
//=====================================================================
void setup(){
  Wire.begin();                 // I2C 100kHz
#ifdef SERIAL_MONITOR
  	Serial.begin(115200);       // UART 115200bps
    Serial.println("=========================================");
    Serial.println("setup start");
#endif

  i2c_write_byte(LCD_I2C_EXPANDER_ADDR, 0x03, 0xFE);
  i2c_write_byte(LCD_I2C_EXPANDER_ADDR, 0x01, 0x01);      // LCD power ON
  // LCD settings
  lcd.begin(8, 2);
  lcd.setContrast(30);
  lcd.clear();   

  lcd.print("NOW");
  lcd.setCursor(0, 1);
  lcd.print("BOOTING!");

  setupPort();
  delay(10);

  noInterrupts();
  setupTCInt();
  interrupts();

  setupSensor();
  MsTimer2::start();      // Timer inverval start

#ifdef SERIAL_MONITOR
    Serial.println("");
    Serial.println("=========================================");
    Serial.println("loop start");
    Serial.println("");
#endif
}

//-----------------------------------------------
// IO pin input/output settings
// Configure the settings according to the leaf to be connected.
//-----------------------------------------------
void setupPort(){
}

//---------------------------------------------------------------------
// Initial settings for each device
//---------------------------------------------------------------------
//------------------------------
// Sensor
//------------------------------
void setupSensor(){
  //-------------------------------------
  // LIS2DH (accelerometer)
  //-------------------------------------
  accel.begin(LIS2DH_ADDRESS);

  accel.setClick(0, 0);                             // Disable Interrupt
  accel.setRange(LIS3DH_RANGE_2_G);                 // Full scale +/- 2G
  accel.setDataRate(LIS3DH_DATARATE_10_HZ);         // Data rate = 10Hz

  //-------------------------------------
  // HTS221 (Humidity and Temperature sensor)
  //-------------------------------------
  smeHumidity.begin(); 

  //-------------------------------------
  // OPT3001 (Ambient Light Sensor)
  //-------------------------------------
  OPT3001_Config newConfig;
  OPT3001_ErrorCode errorConfig;

  light.begin(OPT3001_ADDRESS);                 // I2C address

  newConfig.RangeNumber = B1100;                // automatic full scale
  newConfig.ConvertionTime = B1;                // convertion time = 800ms
  newConfig.ModeOfConversionOperation = B11;    // continous conversion
  newConfig.Latch = B0;                         // hysteresis-style
  
  errorConfig = light.writeConfig(newConfig);
  
  if(errorConfig != NO_ERROR){
    errorConfig = light.writeConfig(newConfig);   // retry
  }
}

//=====================================================================
// Interrupt
//=====================================================================
//-----------------------------------------------
// Interrupt initialization
// Timer interrupt (interval=125ms, int=overflow)
// Timer interrupt setting for main loop
//-----------------------------------------------
void setupTCInt(){
  MsTimer2::set(LOOP_INTERVAL, intTimer);
}

//----------------------------------------------
// Timer INT
// Timer interrupt function
//----------------------------------------------
void intTimer(){
  bInterval = true;
}

//====================================================================
// loop
//====================================================================
//---------------------------------------------------------------------
// Main loop
//---------------------------------------------------------------------
void loop(){
  //-----------------------------------------------------
  // TTimer interval Loop once in 125ms
  //-----------------------------------------------------
  if (bInterval == true){
     bInterval = false; 
    //--------------------------------------------    
    loopCounter();                    // loop counter
    //--------------------------------------------
    // Run once in 1s
    //--------------------------------------------
    if (event1s == true){
      event1s = false;                  // initialize parameter
      loopSensor();                     // sensor read
      dispSencerData();                 // LCD
    }
  } 
}

//---------------------------------------------------------------------
// Counter
// Count the number of loops in the main loop and turn on sensor data acquisition
// at 1-second intervals
//---------------------------------------------------------------------
void loopCounter(){
  iLoop1s += 1;
  //--------------------
  // 1s period
  //--------------------
  if (iLoop1s >=  8){             // 125ms x 8 = 1s
    iLoop1s = 0;
    event1s = true;
  }
}

//---------------------------------------------------------------------
// Sensor
// When sensor data acquisition is ON, data is acquired from each sensor
// Serial output of measured values and calculation results when console output is ON
//---------------------------------------------------------------------
void loopSensor(){
  double temp_mv;
    //-------------------------
    // LIS2DH
    // Data acquisition for 3-axis sensors
    //-------------------------
    accel.read();
    dataX_g = accel.x_g;    // X-axis
    dataY_g = accel.y_g;    // Y-axis
    dataZ_g = accel.z_g;    // Z-axis

    if(dataZ_g >= 1.0){
      dataZ_g = 1.00;
    } else if (dataZ_g <= -1.0){
      dataZ_g = -1.00;
    }

    dataTilt = acos(dataZ_g)/PI*180;
    
    //-------------------------
    // HTS221
    // Temperature and humidity sensor data acquisition
    //-------------------------
    dataTemp = (float)smeHumidity.readTemperature();  // Temperature
    dataHumid = (float)smeHumidity.readHumidity();    // Humidity

    //-------------------------
    // Two-point correction for temperature and humidity
    //-------------------------
    dataTemp=TM0+(TM1-TM0)*(dataTemp-TL0)/(TL1-TL0);      // Temperature correction
    dataHumid=HM0+(HM1-HM0)*(dataHumid-HL0)/(HL1-HL0);    // Humidity correction

    //-------------------------
    // OPT3001
    // Illuminance sensor data acquisition
    //-------------------------
    OPT3001 result = light.readResult();

    if(result.error == NO_ERROR){
      dataLight = result.lux;
    }

  //-------------------------
  // ADC081C027（ADC)
  // Battery leaf battery voltage acquisition
  //-------------------------
  uint8_t adcVal1 = 0;
  uint8_t adcVal2 = 0;

  Wire.beginTransmission(BATT_ADC_ADDR);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(BATT_ADC_ADDR,2);
  adcVal1 = Wire.read();
  adcVal2 = Wire.read();

  if (adcVal1 == 0xff && adcVal2 == 0xff) { 
    // If the measured value is FF, the battery leaf is not connected.
    adcVal1 = adcVal2 = 0;
  }

  // Voltage calculation :　ADC　* ((Reference voltage(3.3V)/ ADC resolution(256)) * Divided voltage ratio(2)
  temp_mv = ((double)((adcVal1 << 4) | (adcVal2 >> 4)) * 3300 * 2) / 256;
  dataBatt = (float)(temp_mv / 1000);

    //-------------------------
    // Serial monitor display
    //-------------------------
#ifdef SERIAL_MONITOR
    Serial.println("--- sensor data ---");    
    Serial.println("  Tmp[degC]     = " + String(dataTemp));
    Serial.println("  Hum[%]        = " + String(dataHumid));
    Serial.println("  Lum[lx]       = " + String(dataLight));
    Serial.println("  Ang[arc deg]  = " + String(dataTilt));
    Serial.println("  Bat[V]        = " + String(dataBatt));
#endif
}

//---------------------------------------
//  Disp sensor data
// Convert sensor data into character string and display on LCD
//---------------------------------------
void dispSencerData(){
  float value;
  char temp[7], humid[7], light[7], tilt[7], battVolt[7];
  char sendData[40];

  //-----------------------------------
  // Convert sensor data to strings
  // dtostrf(Number to be converted, number of characters to be converted, number of decimal places, where to store the converted characters);
  // If the number of characters to be converted is set to -, the converted characters will be left-justified; if +, they will be right-justified.
  //-----------------------------------
  //-------------------------
  // Temperature (4Byte)
  //-------------------------
  value = dataTemp;
  if(value >= 100){
    value = 99.9;
  }
  else if(value <= -10){    
    value = -9.9;
  } 
  dtostrf(value,4,1,temp);

  //-------------------------
  // Humidity (4Byte)
  //-------------------------
  value = dataHumid;   
  dtostrf(value,4,1,humid);

  //-------------------------
  // Ambient Light (5Byte)
  //-------------------------
  value = dataLight;
  if(value >= 100000){
    value = 99999;
  }      
  dtostrf(value,5,0,light);
 
  //-------------------------
  // Tilt (4Byte)
  //-------------------------
  value = dataTilt;
  if(value < 3){
    value = 0;
  }
  dtostrf(value,4,0,tilt);

  //-------------------------
  // Battery Voltage (4Byte)
  //-------------------------
  value = dataBatt;
  if (value >= 10){
   value = 9.99;
  }
  dtostrf(value, 4, 2, battVolt);

  //-------------------------
  trim(temp);
  trim(humid);
  trim(light); 
  trim(tilt);
  trim(battVolt);

  lcd.clear();
  switch (lcdSendCount){
    case 0:                 // Tmp XX.X [degC]       
      lcd.print("Temp");
      lcd.setCursor(0, 1);
      lcd.print( String(temp) +" C");        
      break;
    case 1:                 // Hum xx.x [%]
      lcd.print("Humidity");
      lcd.setCursor(0, 1);
      lcd.print( String(humid) +" %");     
      break;
    case 2:                 // Lum XXXXX [lx]
      lcd.print("Luminous");
      lcd.setCursor(0, 1);
      lcd.print( String(light) +" lx"); 
      break;
    case 3:                 // Ang XXXX [arc deg]
      lcd.print("Angle");
      lcd.setCursor(0, 1);
      lcd.print( String(tilt) +" deg");
      break;
    case 4:                 // Bat X.XX [V]
      lcd.print("Battery");
      lcd.setCursor(0, 1);
      lcd.print( String(battVolt) +" V");
      break;
    default:
      break;
  }
  if (lcdSendCount < 4){
    lcdSendCount++;
  }
  else{
    lcdSendCount = 0;
  }
}

//---------------------------------------
// trim 
// Removing SP from a string array
//---------------------------------------
void trim(char * data){
  int i = 0, j = 0;

  while (*(data + i) != '\0'){
    if (*(data + i) != ' '){
      *(data + j) = *(data + i);
      j++;
    }
    i++;
  }
  *(data + j) = '\0';
}

//=====================================================================
// I2C control function
//=====================================================================
//-----------------------------------------------
// I2C Write 1 byte to the slave device
//-----------------------------------------------
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}

//-----------------------------------------------
// I2C Read 1 byte from the slave device
//-----------------------------------------------
unsigned char i2c_read_byte(int device_address, int reg_address){
  int read_data = 0;

  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, 1);
  read_data = Wire.read();

  return read_data;
}
