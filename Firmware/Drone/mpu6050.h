#pragma once

#include <Arduino.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <stdint.h>

#define MPU_D_ACC_X 0
#define MPU_D_ACC_Y 1
#define MPU_D_ACC_Z 2
#define MPU_D_ACC_T 3
#define MPU_D_ANG_X 4
#define MPU_D_ANG_y 5
#define MPU_D_ANG_Z 6

void mpu6050_calibrate(MPU6050& mpu);

/**
 * @param direction[OUT] Q14
 */
uint8_t mpu6050_event(MPU6050& mpu, int32_t positioning[9], int16_t direction[3]);

void mpu6050_init(MPU6050& mpu);

/**
 * @param data[OUT] data from the accelerometer
 * - [accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
 * - acc - acceleration
 * - gyr - angular velocity
 * - temp - temperature (raw)
 */
void mpu6050_getData(int16_t data[7]);

void mpu6050_measure(MPU6050& mpu, int16_t m[6 * 3]);
