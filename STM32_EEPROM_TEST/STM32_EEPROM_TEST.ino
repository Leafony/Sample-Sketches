#include <EEPROM.h>

int addr = 0;
byte value;
uint16_t err_cnt = 0;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("EEPROM Test");

  // Write Test
  Serial.println("EEPROM Write Start.");

  for (addr = 0; addr < EEPROM.length(); addr++) {
    EEPROM.write(addr, (addr & 0xFF));

    if (addr % 10 == 0 || addr == 2047) {
      Serial.print("Write: ");
      Serial.print(addr);
      Serial.println("/2047");
    }
  }
  
  Serial.println("EEPROM Write Finished!");

  // Read Test
  Serial.println("EEPROM Read Start.");
  
  for (addr=0; addr<EEPROM.length(); addr++){
    value = EEPROM.read(addr);

    if (value != (addr & 0xFF)) {
      err_cnt++;
      Serial.print("Error: Data Invalid!!: addr=");
      Serial.print(addr);
      Serial.print(" write=");
      Serial.print(addr & 0xFF);
      Serial.print(" read=");
      Serial.println(value, DEC);
    }

    if (addr % 100 == 0 || addr == 2047) {
      Serial.print("Read: ");
      Serial.print(addr);
      Serial.println("/2047");
    }
  }

  Serial.println("EEPROM Read Finished!");

  Serial.println("--------------------------------------");
  if (err_cnt != 0) {
    Serial.println("     Test Result: Failed     ");
  } else {
    Serial.println("     Test Result: Success!     ");
  }
  Serial.println("--------------------------------------");
}

void loop() {
  // put your main code here, to run repeatedly:

}
