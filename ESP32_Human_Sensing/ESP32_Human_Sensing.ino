//=====================================================================
// Leafony Platform sample sketch
//     Application  : Human_Sensing (Beep)
//     Processor    : ESP32-WROOM-32 (ESP32 Dev Module)
//     Arduino IDE  : 1.8.13
//     Arduino ESP32: 1.0.4
//
//     Leaf configuration
//       (1) AI02 SP&PIR
//       (2) AP02 ESP32 MCU
//
//		(c) 2020  Trillion-Node Study Group
//		Released under the MIT license
//		https://opensource.org/licenses/MIT
//
//      Rev.00 2020/08/20  First release
//=====================================================================
//---------------------------------------------------------------------
// difinition
//---------------------------------------------------------------------
#include <Wire.h>                           // I2C

//-----------------------------------------------
// IOピン一覧
//-----------------------------------------------
//  D0  IO3   (RXD)
//  D1  IO1   (TXD)
//  D2  IO4   (INT0)
//  D3  IO27  (INT1)
//  D4  IO12  (H-MISO)
//  D5  IO13  (H-MOSI)
//  D6  IO14  (H-SCK)
//  D7  IO15  (H-SS)
//  D8  IO17  (UART_TX)
//  D9  IO16  (UART_RX)
//  D10 IO5   (SS)
//  D11 IO23  (MOSI)
//  D12 IO19  (MISO)
//  D13 IO18  (SCK)
//  D14 IO21  (SDA)
//  D15 IO22  (SCL)
//  A0  IO36
//  A1  IO25  (UART_TX)
//  A2  IO26  (UART_RX)
//  A3  IO39
//  A4  IO34
//  A5  IO35

//-----------------------------------------------
// IOピンの名前定義
// 接続するリーフに合わせて定義する
//-----------------------------------------------
#define PIR_INT             4                   // D2 IO4
#define BUZZER_OUT          13                  // D5 IO13 buzzer output

//------------------------------
// Tone設定
//------------------------------
#define LEDC_CHANNEL_0      0       // use first channel of 16 channels (started from zero)
#define LEDC_TIMER_13_BIT   13      // use 13 bit precission for LEDC timer
#define LEDC_BASE_FREQ      5000    // use 5000 Hz as a LEDC base frequency

//-----------------------------------------------
// プログラム内で使用する定数定義
//-----------------------------------------------
#define I2C_PIR_ADDR   0x65
#define I2C_SEND_BUF_LENGTH 10
#define I2C_RECV_BUF_LENGTH 10

unsigned char i2c_sendBuf[I2C_SEND_BUF_LENGTH];
unsigned char i2c_recvBuf[I2C_RECV_BUF_LENGTH];

double irData;
double tempData;

volatile int HumanDetected = 0; 

//=====================================================================
// setup
//=====================================================================
void setup(){
  // initialize serial communication at 115200 second per second:
  Serial.begin(115200);
  // initialize i2c communication:
  Wire.begin();

  delay(100);

  //人感センサ設定
  i2c_write_byte(I2C_PIR_ADDR, 0x20, 0xFF); //CNTL1  Resrt 
  i2c_write_byte(I2C_PIR_ADDR, 0x2A, 0xF2); //CNTL11 人感アルゴリズム有効/割り込み出力有効
  i2c_write_byte(I2C_PIR_ADDR, 0x25, 0x0F); //CNTL6  センサゲイン205%(最大)
  i2c_write_byte(I2C_PIR_ADDR, 0x2B, 0xFF); //CNTL12 Mode=1 start Meas(連続測定モード)
   delay(1000);

  // Beep
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
  ledcAttachPin(BUZZER_OUT, LEDC_CHANNEL_0);

  // 人接近検知割り込み
  attachInterrupt(PIR_INT,onHumanDetected , FALLING );

}

//---------------------------------------------------------------------
// 割り込み設定
//---------------------------------------------------------------------
//----------------------------------------------
// 人を検出したら呼び出される関数
//----------------------------------------------
void onHumanDetected(){
  HumanDetected = 1;
}

//=====================================================================
// Main loop
//=====================================================================
void loop(){

  // レジスタ読み出し
  i2c_read(I2C_PIR_ADDR, 0x04, 6, i2c_recvBuf);
  // IRセンサ
  irData = clacIR();
  Serial.print("IR   = ");
  Serial.print(irData,2);
  Serial.println(" pA");

  // センサ温度
  tempData = clacTemp();
  Serial.print("TSENS = ");
  Serial.print(tempData,2);
  Serial.println(" deg");
  Serial.println("===================================");

  if (HumanDetected == 1){          // 人の接近を検知
    Serial.println("Detect!");

    // Beep
    ledcWriteNote(LEDC_CHANNEL_0, NOTE_C, 5);
    int pauseBetweenNotes = 1000 / 4 * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone
    ledcWriteTone(LEDC_CHANNEL_0, 0);

    HumanDetected = 0;
  }
  delay(1000);  
}


//=====================================================================
double clacTemp(){  
  double ret;
  unsigned short val = (unsigned short)((i2c_recvBuf[4] << 8) |  i2c_recvBuf[3]);
  if ( (val & 0x8000) == 0x8000){
     val = ~val + 1;     
     ret = (double)((val) *   0.0019837 ) * -1;
  }
  else{
    ret = (double)val * 0.0019837;
  }
  return ret + 25;
}

//=====================================================================
double clacIR(){  
  double ret;
  unsigned short val = (unsigned short)((i2c_recvBuf[2] << 8) |  i2c_recvBuf[1]);
  if ((val & 0x8000) == 0x8000){
    val = ~val + 1;
    ret = (double)(val *   0.4578 ) * -1;
  }
  else{
    ret = (double)(val *  0.4578 );
  } 
  return ret;
}

//=====================================================================
// I2C　制御関数
//=====================================================================
//-----------------------------------------------
// I2C スレーブデバイスに1バイト書き込む
//-----------------------------------------------
void i2c_write_byte(int device_address, int reg_address, int write_data){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.write(write_data);
  Wire.endTransmission();
}

//-----------------------------------------------
// I2C スレーブデバイスから1バイト読み込む
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

//-----------------------------------------------
// I2C スレーブデバイスに複数バイト書き込む
//-----------------------------------------------
void i2c_write(int device_address, int reg_address, int lengrh, unsigned char* write_byte){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  for (int i = 0; i < lengrh; i++){
    Wire.write(write_byte[i]);
  }
  Wire.endTransmission();
}

//-----------------------------------------------
// I2C スレーブデバイスから複数バイト読み込む
//-----------------------------------------------
void i2c_read(int device_address, int reg_address, int lengrh, unsigned char* read_byte){
  Wire.beginTransmission(device_address);
  Wire.write(reg_address);
  Wire.endTransmission(false);

  Wire.requestFrom(device_address, lengrh);
  for (int i = 0; i < lengrh; i++){
    read_byte[i] = Wire.read();
  }
}
