#pragma once

#include <Arduino.h>
#include <stdint.h>

enum BES { // ballast's edges
  LE = 0, // left  empty
  LF = 1, // left  full
  RE = 2, // right empty
  RF = 3, // right full

  LE_B = 1 << LE, // left  empty bit
  LF_B = 1 << LF, // left  full  bit
  RE_B = 1 << RE, // right empty bit
  RF_B = 1 << RF  // right full  bit
};

/**
 * @brief The function initializes the ballasts' pins.
 */
void ballast_init();

/**
 * @brief The function checks all ballasts' edges.
 * @return bit mask. Check BES above.
 */
uint8_t ballast_check_all();

/**
 * @brief The function checks empty edge of the left ballast.
 * @return true - edge detected, false - edge NOT detected.
 */
inline bool ballast_check_left_empty();

/**
 * @brief The function checks full edge of the left ballast.
 * @return true - edge detected, false - edge NOT detected.
 */
inline bool ballast_check_left_full();

/**
 * @brief The function checks empty edge of the right ballast.
 * @return true - edge detected, false - edge NOT detected.
 */
inline bool ballast_check_right_empty();

/**
 * @brief The function checks full edge of the right ballast.
 * @return true - edge detected, false - edge NOT detected.
 */
inline bool ballast_check_right_full();
