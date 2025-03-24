#include "hmc5883l.h"

#include <Wire.h>

#define HMC5883L_ADDR 0x1E // I2C address of the device

// Address
#define A_MODE_REG (0x02) // moderegister address
// Value
#define V_CONT_MSR (0x00) // continuous measurement mode

// Address
#define A_DATA_OUT (0x03) // start data output register

void hmc5883_init() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(A_MODE_REG);
  Wire.write(V_CONT_MSR);
  Wire.endTransmission();
}

void hmc5883_getValues(int16_t data[3]) {
    // Request 6 bytes of magnetometer data
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(A_DATA_OUT);  // Start at data output register
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);
  
  for (uint8_t i = 0; i < 3; i++) {
    data[i] = Wire.read() << 8 | Wire.read();
  }
}
