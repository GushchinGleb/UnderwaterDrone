#pragma once

#include <stdint.h>

void MHSD_init();

uint8_t MHSD_read_offsets(int16_t offsets[6]);

uint8_t MHSD_write_measures(const int16_t measures[3]);

void MHSD_write_offsets(const int16_t offsets[6]);

void MHSD_write_mpu();
