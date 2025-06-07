#include <Arduino.h>

#include "motor.h"

#include "chanels.h"
#include "RF24-rec.h"

// Pins

// Motors
#define MOTOR_E_L_F (PH2)   // engine left  forward
#define MOTOR_E_L_B (PH1)   // engine left  backward
#define MOTOR_E_L_P (OCR4A) // engine left  power
#define MOTOR_E_R_F (PH6)   // engine right forward
#define MOTOR_E_R_B (PH5)   // engine right backward
#define MOTOR_E_R_P (OCR4B) // engine right power

// Ballast
#define MOTOR_B_L_F (PL6)   // ballast left  forward
#define MOTOR_B_L_B (PL7)   // ballast left  backward
#define MOTOR_B_L_P (OCR5C) // ballast left  power
#define MOTOR_B_R_F (PL2)   // ballast right forward
#define MOTOR_B_R_B (PL3)   // ballast right backward
#define MOTOR_B_R_P (OCR5B) // ballast right power

#define MAX_VAL (255)

void motor_init() {
  /*** CONFIGURE PWM ***/

  // Set PH3 and PH4 as output
  DDRH |= (1 << PH3) | (1 << PH4);

  // Set Fast PWM mode using ICR4 as TOP (Mode 14)
  TCCR4A = (1 << COM4A1) | (1 << COM4B1);             // Non-inverting mode on OC4A/OC4B
  TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41); // Prescaler 8
  TCCR4A |= (1 << WGM41);  // Combine for mode 14

  ICR4 = 255; // TOP value (PWM resolution)

  MOTOR_E_R_P = 0;  // Duty cycle for PH3 (PWM out)
  MOTOR_E_L_P = 0;  // Duty cycle for PH4 (PWM out)

  // Set PL4 and PL5 as output
  DDRL |= (1 << PL4) | (1 << PL5);

  // Set Fast PWM mode using ICR5 as TOP (Mode 14)
  TCCR5A = (1 << COM5B1) | (1 << COM5C1);             // Non-inverting for OC5B (PL4) and OC5C (PL5)
  TCCR5B = (1 << WGM53) | (1 << WGM52) | (1 << CS51); // WGM mode 14, prescaler = 8
  TCCR5A |= (1 << WGM51);

  ICR5 = 255; // TOP value (8-bit resolution)
  OCR5B = 0;  // Duty cycle for PL4 (OC5B)
  OCR5C = 0;  // Duty cycle for PL5 (OC5C)

  /*** CONFIGURE CONTROLS ***/

  // Set PH1, PH2, PH5, PH6 as outputs
  DDRH |= (1 << MOTOR_E_L_F) | (1 << MOTOR_E_L_B) | (1 << MOTOR_E_R_F) | (1 << MOTOR_E_R_B);
  // Set PH1, PH2, PH5, PH6 to LOW
  PORTH &= ~((1 << MOTOR_E_L_F) | (1 << MOTOR_E_L_B) | (1 << MOTOR_E_R_F) | (1 << MOTOR_E_R_B));

  // Set PL2, PL3, PL6, PL7 as outputs
  DDRL |= (1 << MOTOR_B_L_F) | (1 << MOTOR_B_L_B) | (1 << MOTOR_B_R_F) | (1 << MOTOR_B_R_B);
  // Set PL2, PL3, PL6, PL7 to LOW
  PORTL &= ~((1 << MOTOR_B_L_F) | (1 << MOTOR_B_L_B) | (1 << MOTOR_B_R_F) | (1 << MOTOR_B_R_B));
}

void motor_move(float x, float a) {
  float r = x - a; // right motor
  float l = x + a; // left motor

  int16_t ar = (int16_t)(r * (float)MAX_VAL); // analog output to the right motor
  int16_t al = (int16_t)(l * (float)MAX_VAL); // analog output to the left motor

  ar = max(min(MAX_VAL, ar), -MAX_VAL);
  al = max(min(MAX_VAL, al), -MAX_VAL);

  // TODO: controle motors
}

#define CONTROL_MOTOR(port, front_pin, back_pin, power_ocr) \
  port = port & (~((1 << front_pin) | (1 << back_pin))) | ((1 << front_pin) & sign | (1 << back_pin) & ~sign); \
  power_ocr = bin_val; \
  break

void motor_chanel(uint8_t chanel, int16_t value) {
  if (value > 255) value = 255;
  else if (value < -255) value = -255;

  uint8_t bin_val = (uint8_t)(uint16_t)(value > 0 ? value : -value);
  uint8_t sign = (value & (1 << 15)) ? 0x00 : 0xFF;

  switch(chanel) {
  case M_CS::M_L: CONTROL_MOTOR(PORTH, MOTOR_E_L_F, MOTOR_E_L_B, MOTOR_E_L_P); // left engine
  case M_CS::M_R: CONTROL_MOTOR(PORTH, MOTOR_E_R_F, MOTOR_E_R_B, MOTOR_E_R_P); // right engine
  case M_CS::M_l: CONTROL_MOTOR(PORTL, MOTOR_B_L_F, MOTOR_B_L_B, MOTOR_B_L_P); // left ballast
  case M_CS::M_r: CONTROL_MOTOR(PORTL, MOTOR_B_R_F, MOTOR_B_R_B, MOTOR_B_R_P); // ring ballast
  default: break;
  }

  return;
}

void motor_control(int16_t x, int16_t y) {
  int16_t r = x + y;
  int16_t l = x - y;

  if (r > 255) r = 255;
  else if (r < -255) r = -255;
  if (l > 255) l = 255;
  else if (l < -255) l = -255;

  const uint8_t bin_val_r = (uint8_t)(uint16_t)(r > 0 ? r : -r);
  const uint8_t bin_val_l = (uint8_t)(uint16_t)(l > 0 ? l : -l);

  const uint8_t sign_r = (r & (1 << 15)) ? 0x00 : 0xFF;
  const uint8_t sign_l = (l & (1 << 15)) ? 0x00 : 0xFF;

  PORTH = PORTH & (~((1 << MOTOR_E_L_F) | (1 << MOTOR_E_L_B))) | ((1 << MOTOR_E_L_F) & sign_l | (1 << MOTOR_E_L_B) & ~sign_l);
  MOTOR_E_L_P = bin_val_l;
  PORTH = PORTH & (~((1 << MOTOR_E_R_F) | (1 << MOTOR_E_R_B))) | ((1 << MOTOR_E_R_F) & sign_r | (1 << MOTOR_E_R_B) & ~sign_r);
  MOTOR_E_R_P = bin_val_r;
}
