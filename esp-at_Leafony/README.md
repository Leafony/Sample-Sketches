## ESP-AT for Leafony
ESP32 MCUをATコマンドで使用するためのファームウエア

espressifが提供している[ESP-AT](https://github.com/espressif/esp-at) V2.4.0.0(2022/05/20)のUARTピンアサインをに合わせたもの 

• TX (GPIO16)	  D9
• RX (GPIO17)	  D8
• CTS (GPIO15)	D7 (optional)
• RTS (GPIO14)	D6 (optional)


• Baudrate  : 115200
• Data Bits : 8
• Parity    : None
• Stop Bits : 1
• Flow Type : None
