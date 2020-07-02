
#define DEBUG_BUILD // comment out when non-debugging mode

#ifdef DEBUG_BUILD
#define SERIAL_BEGIN(num) Serial.begin(num)
#define SERIAL_PRINTLN(str) Serial.println(str)
#define SERIAL_PRINT(str) Serial.print(str)
#else
#define SERIAL_BEGIN(num)
#define SERIAL_PRINTLN(str)
#define SERIAL_PRINT(str)
#endif
