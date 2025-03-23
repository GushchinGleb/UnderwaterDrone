#include "mpu6050.h"

#include "Wire.h"

#define MPU_ADDR 0x68 // sensor I2C address

// Address
#define A_POWER_MANAGEMENT 0x6B // power configuration
// Values
#define V_WAKEUP 0x00

// Address
#define A_ACCELERATOR 0x1C // accelerator configuration
// Value
#define V_ACCEL_2G 0x00
#define V_ACCEL_4G 0x08
#define V_ACCEL_8G 0x10
#define V_ACCEL_16G 0x18

// Address
#define A_GYRO 0x1B // gyroscope configuration
// Value
#define V_GYRO_250DS 0x00  // degrees per second
#define V_GYRO_500DS 0x08  // degrees per second
#define V_GYRO_1000DS 0x10  // degrees per second
#define V_GYRO_2000DS 0x18  // degrees per second

// Address
#define A_ACCELERATION 0x3B // Value of the current acceleration

void mpu6050_init() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(A_POWER_MANAGEMENT);
    Wire.write(V_WAKEUP);  // Wake up MPU6050
    Wire.endTransmission();
  
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(A_ACCELERATOR);
    Wire.write(V_ACCEL_4G);
    Wire.endTransmission();
  
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(A_GYRO);
    Wire.write(V_GYRO_250DS);
    Wire.endTransmission();
}

void mpu6050_getData(int16_t data[7]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(A_ACCELERATION);
  Wire.endTransmission(false);

/** @brief data for the accelerometer
 * - [accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
 * - acc - acceleration
 * - gyr - angular velocity
 * - temp - temperature (raw)
 */
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers
  for (uint8_t i = 0; i < 7; i++) {
    data[i] = Wire.read() << 8 | Wire.read();
  }
}