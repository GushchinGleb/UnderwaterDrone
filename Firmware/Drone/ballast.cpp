#include "ballast.h"

#include <Arduino.h>
#include <avr/io.h>

#define PIN_LE PD5
#define PIN_LF PD4
#define PIN_RE PD6
#define PIN_RF PD7

void ballast_init() {
    // Clear bits 4–7 in DDRD to set PD4–PD7 as input
  DDRD &= ~((1 << PIN_LE) | (1 << PIN_LF) | (1 << PIN_RE) | (1 << PIN_RF));

  // Set bits 4–7 in PORTD to enable internal pull-up resistors
  PORTD |= (1 << PIN_LE) | (1 << PIN_LF) | (1 << PIN_RE) | (1 << PIN_RF);

  return;
}

inline bool ballast_check_left_empty() {
  return PIND & (1 << PIN_LE);
}

inline bool ballast_check_left_full() {
  return PIND & (1 << PIN_LF);
}

inline bool ballast_check_right_empty() {
  return PIND & (1 << PIN_RE);
}

inline bool ballast_check_right_full() {
  return PIND & (1 << PIN_RF);
}
