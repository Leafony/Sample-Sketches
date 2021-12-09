//=====================================================================
//  Leafony Platform sample sketch
//     Application  : STM32 Simple BLE Beacon Example
//     Processor    : STM32L452RE (Nucleo-64/Nucleo L452RE)
//     Arduino IDE  : 1.8.12
//     STM32 Core   : Ver1.9.0
//
//     Leaf configuration
//       (1) AC02 BLE Sugar     :Bus-A or Bus-B
//       (2) AP03 STM32 MCU
//       (3) AZ01 USB           :Bus-A
//
//    (c) 2021 LEAFONY SYSTEMS Co., Ltd
//    Released under the MIT license
//    https://opensource.org/licenses/MIT
//
//      Rev.00 2021/04/01 First release
//=====================================================================
//  Required Libraries
//    https://github.com/Leafony/TBGLib
//    https://github.com/stm32duino/STM32LowPower
//    https://github.com/stm32duino/STM32RTC
//=====================================================================
#include "STM32LowPower.h"
#include "TBGLib.h"

//=====================================================================
// シリアルコンソールへのデバック出力
//      #define DEBUG = 出力あり
//　　//#define DEBUG = 出力なし（コメントアウトする）
//=====================================================================
#define DEBUG

//=====================================================================
// スリープ時間、送信時間の設定
//  SLEEP_INTERVAL : スリープ時間 (秒)
//  WAKE_INTERVAL　：パケット送信時間 (秒)
//=====================================================================
#define SLEEP_INTERVAL (8)
#define WAKE_INTERVAL  (1)

//=====================================================================
// IOピンの名前定義
//=====================================================================
#define BLE_WAKEUP PB12 // D7   PB12
#define BLE_RX PA0      // [A2] PA1
#define BLE_TX PA1      // [A1] PA0
#define INT_0 PC7       // INT0
#define INT_1 PB3       // INT1

//=====================================================================
// objects
//=====================================================================
// BLE
HardwareSerial Serialble(BLE_TX, BLE_RX);
BGLib ble112((HardwareSerial *)&Serialble, 0, 0);

//=====================================================================
// Variables
//=====================================================================
// BLE
volatile bool bSystemBootBle = false;

//=====================================================================
// IOピンの入出力設定
// 接続するリーフに合わせて設定する
//=====================================================================
void setupPort()
{
  pinMode(BLE_WAKEUP, OUTPUT);    // BLE Wakeup/Sleep
  digitalWrite(BLE_WAKEUP, HIGH); // BLE Wakeup
}

//-----------------------------------------------
// BLE
//-----------------------------------------------
void setupBLE()
{
  // set up internal status handlers
  ble112.onBusy = onBusy;
  ble112.onIdle = onIdle;
  ble112.onTimeout = onTimeout;

  // set up BGLib event handlers
  ble112.ble_evt_gatt_server_attribute_value = my_evt_gatt_server_attribute_value;
  ble112.ble_evt_le_connection_opend = my_evt_le_connection_opend;
  ble112.ble_evt_le_connection_closed = my_evt_le_connection_closed;
  ble112.ble_evt_system_boot = my_evt_system_boot;
  ble112.ble_evt_system_awake = my_evt_system_awake;
  ble112.ble_rsp_system_get_bt_address = my_rsp_system_get_bt_address;

  uint8_t tm = 0;
  Serialble.begin(9600);
  while (!Serialble && tm < 150)
  { // Serial起動待ち タイムアウト1.5s
    tm++;
    delay(10);
  }

  while (!bSystemBootBle)
  { // BLE起動待ち
    ble112.checkActivity(100);
  }

  ble112.ble_cmd_system_get_bt_address();
  while (ble112.checkActivity(1000));

  /* interval_min : 250ms( = 400 x 0.625ms ) */
  /* interval_max : 500ms( = 800 x 0.625ms ) */
  ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7); /* [BGLIB] <interval_min> <interval_max> <channel_map> */
  while (ble112.checkActivity(1000));
}

//-----------------------------------------------
// アドバタイズするデータの設定
//-----------------------------------------------
void StartAdvData()
{
  // Advertising data; 25byte MAX
  uint8_t adv_data[] = {
    // AD Structure 1: Flag
    (2),  //0: field length
    BGLIB_GAP_AD_TYPE_FLAGS,  //1: field type (0x01)
    (6),  //2: data
    // AD Structure 2: Complete local name
    (7),  //3: field length
    BGLIB_GAP_AD_TYPE_LOCALNAME_COMPLETE,  //4: field type (0x09)
    ('L'),  //5:
    ('e'),  //6:
    ('a'),  //7:
    ('f'),  //8:
    ('_'),  //9:
    ('A'),  //10:
    // AD Structure 3: Manufacture specific
    (13),   //11: field length
    (0xff), //12: field type (0xff)
    ('H'),  //13:
    ('E'),  //14:
    ('L'),  //15:
    ('L'),  //16:
    ('O'),  //17:
    (' '),  //18:
    ('B'),  //19:
    ('E'),  //20:
    ('A'),  //21:
    ('C'),  //22:
    ('O'),  //23:
    ('N'),  //24:
  };

  // Register advertising packet
  uint8_t stLen = sizeof(adv_data);
  ble112.ble_cmd_le_gap_set_adv_data(SCAN_RSP_ADVERTISING_PACKETS, stLen, adv_data);
  while (ble112.checkActivity(1000));

  // index = 0  LE_GAP_SCANNABLE_NON_CONNECTABLE / LE_GAP_UNDIRECTED_CONNECTABLE
  ble112.ble_cmd_le_gap_start_advertising(0, LE_GAP_USER_DATA, LE_GAP_SCANNABLE_NON_CONNECTABLE);
  while (ble112.checkActivity(1000));
}

//---------------------------------------
// sleep BLE
// BLE リーフをスリープさせる
//---------------------------------------
void sleepBLE()
{
  ble112.ble_cmd_le_gap_stop_advertising(0);
  while (ble112.checkActivity());

  ble112.ble_cmd_system_halt(1);
  while (ble112.checkActivity());

  digitalWrite(BLE_WAKEUP, LOW);
  delay(500);
}

//---------------------------------------
// wakeup BLE
// BLEリーフをスリープから復帰させる
//---------------------------------------
void wakeupBLE()
{
  digitalWrite(BLE_WAKEUP, HIGH);
  delay(500);

  ble112.ble_cmd_system_halt(0);
  while (ble112.checkActivity());

  ble112.ble_cmd_le_gap_set_adv_parameters(400, 800, 7);
  while (ble112.checkActivity(1000));
}

//---------------------------------------
// setup
//
//---------------------------------------
void setup()
{
  Serial.begin(115200);
  LowPower.begin(); // Configure low power

  setupPort();
  delay(10);

  setupBLE();
}

//---------------------------------------
// loop
//
//---------------------------------------
void loop()
{
  StartAdvData();
#ifdef DEBUG
  Serial.println(F("Start advertise"));
  Serial.flush();
#endif

  // Continue Advertising (during that STM32 sleeps.)
  LowPower.deepSleep(WAKE_INTERVAL * 1000);

#ifdef DEBUG
  Serial.println(F("Sleep BLE"));
#endif
  sleepBLE();

#ifdef DEBUG
  Serial.println(F("Sleep STM32"));
  Serial.println(F(">>> Sleep >>>"));
  Serial.flush();
#endif
  LowPower.deepSleep(SLEEP_INTERVAL * 1000);

#ifdef DEBUG
  Serial.println(F("Wakeup STM32"));
#endif

  wakeupBLE();
#ifdef DEBUG
  Serial.println(F("Wakeup BLE"));
#endif

#ifdef DEBUG
  Serial.println(F("<<< Wake up <<<"));
#endif
}


// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================

// called when the module begins sending a command
void onBusy()
{
}

// called when the module receives a complete response or "system_boot" event
void onIdle()
{
}

// called when the parser does not read the expected response in the specified time limit
void onTimeout()
{
}

// called immediately before beginning UART TX of a command
void onBeforeTXCommand()
{
}

// called immediately after finishing UART TX
void onTXCommandComplete()
{
}

// called when the attribute value changed
void my_evt_gatt_server_attribute_value(const struct ble_msg_gatt_server_attribute_value_evt_t *msg)
{
}

// called when the connection is opened
void my_evt_le_connection_opend(const ble_msg_le_connection_opend_evt_t *msg)
{
}

// called when connection is closed
void my_evt_le_connection_closed(const struct ble_msg_le_connection_closed_evt_t *msg)
{
}

// called when the system booted
void my_evt_system_boot(const ble_msg_system_boot_evt_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tsystem_boot: { ");
  Serial.print("major: ");
  Serial.print(msg->major, HEX);
  Serial.print(", minor: ");
  Serial.print(msg->minor, HEX);
  Serial.print(", patch: ");
  Serial.print(msg->patch, HEX);
  Serial.print(", build: ");
  Serial.print(msg->build, HEX);
  Serial.print(", bootloader_version: ");
  Serial.print(msg->bootloader, HEX);
  Serial.print(", hw: ");
  Serial.print(msg->hw, HEX);
  Serial.println(" }");
#endif

  bSystemBootBle = true;
}

// called when the system awake
void my_evt_system_awake(void)
{
#ifdef DEBUG
  Serial.println("###\tsystem_awake");
#endif
}

//
void my_rsp_system_get_bt_address(const struct ble_msg_system_get_bt_address_rsp_t *msg)
{
#ifdef DEBUG
  Serial.print("###\tsystem_get_bt_address: { ");
  Serial.print("address: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.print(msg->address.addr[i], HEX);
  }
  Serial.println(" }");
#endif
  unsigned short addr = 0;
  char cAddr[30];
  addr = msg->address.addr[0] + (msg->address.addr[1] * 0x100);
  sprintf(cAddr, "Device name is Leaf_A_#%05d ", addr);
  Serial.println(cAddr);
}
