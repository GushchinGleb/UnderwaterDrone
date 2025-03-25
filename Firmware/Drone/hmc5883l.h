#pragma once

#include <stdint.h>

#define HMC_D_X 0
#define HMC_D_Y 1
#define HMC_D_Z 2

/**
 * @brief THe function initialize the sensor.A_DATA_OUT
 * Required Wire been initialised before call.
 */
void hmc5883_init();

/**
 * @brief The fuction gets 3 values from the sensor.
 * @param data[OUT] parameters
 * - 0 X direction
 * - 1 Y direction
 * - 2 Z direction
 */
void hmc5883_getValues(int16_t data[3]);
