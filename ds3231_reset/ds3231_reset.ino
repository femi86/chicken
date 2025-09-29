#include <Wire.h>

#define DS3231_ADDR 0x68

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(DS3231_ADDR, (uint8_t)1);
  return Wire.read();
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Wire.begin(); // use Wire.begin(SDA, SCL) on ESP32 with your pins

  uint8_t ctrl = readReg(0x0E);
  uint8_t stat = readReg(0x0F);

  bool eosc = ctrl & 0x80;        // bit 7
  bool osf  = stat & 0x80;        // bit 7

  // Ensure oscillator is enabled
  if (eosc) {
    ctrl &= ~0x80;
    writeReg(0x0E, ctrl);
  }

  // Clear sticky flags
  if (osf) {
    stat &= ~(0x80 | 0x01 | 0x02);
    writeReg(0x0F, stat);
  }
}

void loop() {
  // nothing
}
