#include "gy-neo6mv2.h"

#include <Arduino.h>
#include <stdint.h>

#define GY_SPEED (9600) // gy serial speed

/**
 * @brief Updates the GPS parser with one character at a time.
 */
static void parsChar(char c, gy_neo6mv2_data_t& data);

/**
 * @brief Converts degrees to radians.
 */
static float convertToRadians(float degrees);

static float parseCoordinate(const char* coord, char direction);


float convertToRadians(float degrees) {
  return degrees * (3.14159265358979f / 180.0f);
}

/**
 * @brief Initializes the Serial1 for GPS communication.
 */
void gy_neo6mv2_init(gy_neo6mv2_data_t& data) {
  GY_SERIAL.begin(GY_SPEED); // GPS baud rate

  data.state = GY_PS::WS; // resset parser
}

void gy_neo6mv2_getData(gy_neo6mv2_data_t& data) {
  while (GY_SERIAL.available()) {
    parsChar(GY_SERIAL.read(), data);
  }
}

static void parsChar(char c, gy_neo6mv2_data_t& data) {
  // TODO: parser using a state machine
}

