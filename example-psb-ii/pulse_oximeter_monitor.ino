/* 01_i2c_scan.ino  --  STEP 2: is the MAX30100 answering on the bus?
   Serial Monitor @ 115200.

   Expected:  "FOUND device at 0x57  <-- MAX30100"
   0 devices  =  wiring / pull-up / power problem
   BUS LOW    =  SDA or SCL held low, or the module's 1.8V pull-ups are
                 dragging the bus under the 5V logic HIGH threshold      */

#include <Wire.h>

#define PIN_SDA A4     // Uno/Nano: A4.  Mega: 20.  Leonardo: 2.
#define PIN_SCL A5     // Uno/Nano: A5.  Mega: 21.  Leonardo: 3.

void busCheck() {
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delay(5);
  int sda = digitalRead(PIN_SDA);
  int scl = digitalRead(PIN_SCL);
  Serial.print(F("Idle line levels -> SDA=")); Serial.print(sda);
  Serial.print(F("  SCL="));                   Serial.println(scl);
  if (!sda || !scl) {
    Serial.println(F("*** BUS LOW. Both should read 1 when idle. ***"));
    Serial.println(F("    Unplug the sensor and reset: if they go to 1, the"));
    Serial.println(F("    module is the cause (1.8V pull-ups or a short)."));
  } else {
    Serial.println(F("Lines idle HIGH - bus looks electrically sane."));
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println(F("=== I2C SCANNER ==="));
  busCheck();

  Wire.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);          // needs AVR core >= 1.8.4
  Serial.println(F("Wire timeout: ENABLED"));
#else
  Serial.println(F("Wire timeout: NOT AVAILABLE - update your AVR core."));
  Serial.println(F("A stuck bus will freeze the scan instead of reporting."));
#endif
}

void loop() {
  byte found = 0;
  Serial.println(F("Scanning 0x01..0x7E ..."));
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  FOUND device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == 0x57) Serial.print(F("   <-- MAX30100"));
      Serial.println();
      found++;
    }
  }
  if (!found) Serial.println(F("  no devices -> check VIN, GND, SDA/SCL, pull-ups"));
  Serial.println();
  delay(2000);
}
