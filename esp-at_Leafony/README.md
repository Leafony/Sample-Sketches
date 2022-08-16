## ESP-AT for Leafony
ESP32 MCUをATコマンドで使用するためのファームウエア

espressifが提供している[ESP-AT](https://github.com/espressif/esp-at) V2.4.0.0(2022/05/20)のUARTピンアサインをLeafonyに合わせたもの 

• TX (IO16)	:  D9<br>
• RX (IO17)	:  D8<br>
• CTS (IO15):	 D7 (optional)<br>
• RTS (IO14):	 D6 (optional)<br>


• Baudrate  : 115200<br>
• Data Bits : 8<br>
• Parity    : None<br>
• Stop Bits : 1<br>
• Flow Type : None<br>
