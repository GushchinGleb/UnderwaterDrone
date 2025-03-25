#pragma once

#include <stdint.h>

#define GY_SERIAL (Serial1) // gy-neo6mv2 serial instance

enum GY_PS { // gy-neo6mv2 parser stare
  WS, // Waiting for "$GPGGA"
  PF, // Parsing fields
  VD  // Data is valid
};

typedef struct gy_neo6mv2_data {
  uint8_t time[3]; // UTS time: 0 - hours, 1 - minutes, 2 - seconds

  float lat; // latitude [radians]
  float lon; // longitude [radians]

  uint8_t fix; // fix quality: 0 - no fix, 1 - GPS, 2 - DGPS

  float alt; // altitude [meters]
  float sea; // height of the sea level [meters]

  uint8_t state; // Current parser state
} gy_neo6mv2_data_t;

/**
 * @brief The function initializes periphery for GY-NEO6MV2 GPS sensor.
 */
void gy_neo6mv2_init(gy_neo6mv2_data_t& data);

void gy_neo6mv2_getData(gy_neo6mv2_data_t& data);
