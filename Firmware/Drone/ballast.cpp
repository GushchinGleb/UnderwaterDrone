#include "ballast.h"

#include <Arduino.h>

#define PIN_LE (255) // left  empty
#define PIN_LF (255) // left  full
#define PIN_RE (255) // right empty
#define PIN_RF (255) // right full

void ballast_init() {
  pinMode(PIN_LE, INPUT_PULLUP);
  pinMode(PIN_LF, INPUT_PULLUP);
  pinMode(PIN_RE, INPUT_PULLUP);
  pinMode(PIN_RF, INPUT_PULLUP);

  return;
}

uint8_t ballast_check_all() {
  return
    digitalRead(PIN_LE) << BES::LE |
    digitalRead(PIN_LF) << BES::LF |
    digitalRead(PIN_RE) << BES::RE |
    digitalRead(PIN_RF) << BES::RF;
}

inline bool ballast_check_left_empty() {
  return digitalRead(PIN_LE);
}

inline bool ballast_check_left_full() {
  return digitalRead(PIN_LF);
}

inline bool ballast_check_right_empty() {
  return digitalRead(PIN_RE);
}

inline bool ballast_check_right_full() {
  return digitalRead(PIN_RF);
}
