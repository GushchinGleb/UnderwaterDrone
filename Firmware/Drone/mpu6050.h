#pragma once

#include <stdint.h>

#define MPU_D_ACC_X 0
#define MPU_D_ACC_Y 1
#define MPU_D_ACC_Z 2
#define MPU_D_ACC_T 3
#define MPU_D_ANG_X 4
#define MPU_D_ANG_y 5
#define MPU_D_ANG_Z 6

void mpu6050_init();

/**
 * @param data[OUT] data from the accelerometer
 */
void mpu6050_getData(int16_t data[7]);
