#include <Arduino.h>

#include "motor.h"

#include "RF24-rec.h"

// Pins

// Motors
#define MOTOR_E_L_F (255) // engine left  forward
#define MOTOR_E_L_B (255) // engine left  backward
#define MOTOR_E_L_P (255) // engine left  power
#define MOTOR_E_R_F (255) // engine right forward
#define MOTOR_E_R_B (255) // engine right backward
#define MOTOR_E_R_P (255) // engine right power

// Ballast
#define MOTOR_B_L_F (255) // ballast left  forward
#define MOTOR_B_L_B (255) // ballast left  backward
#define MOTOR_B_L_P (255) // ballast left  power
#define MOTOR_B_R_F (255) // ballast right forward
#define MOTOR_B_R_B (255) // ballast right backward
#define MOTOR_B_R_P (255) // ballast right power

#define MAX_VAL (255)

void motor_init() {
  pinMode(MOTOR_E_L_P, OUTPUT);
  pinMode(MOTOR_E_R_P, OUTPUT);
  pinMode(MOTOR_B_L_P, OUTPUT);
  pinMode(MOTOR_B_R_P, OUTPUT);
  analogWrite(MOTOR_E_L_P, 0);
  analogWrite(MOTOR_E_R_P, 0);
  analogWrite(MOTOR_B_L_P, 0);
  analogWrite(MOTOR_B_R_P, 0);

  pinMode(MOTOR_E_L_F, OUTPUT);
  pinMode(MOTOR_E_L_B, OUTPUT);
  pinMode(MOTOR_E_R_F, OUTPUT);
  pinMode(MOTOR_E_R_B, OUTPUT);
  pinMode(MOTOR_E_L_F, OUTPUT);
  pinMode(MOTOR_E_L_B, OUTPUT);
  pinMode(MOTOR_E_R_F, OUTPUT);
  pinMode(MOTOR_E_R_B, OUTPUT);
  digitalWrite(MOTOR_B_L_F, LOW);
  digitalWrite(MOTOR_B_L_B, LOW);
  digitalWrite(MOTOR_B_R_F, LOW);
  digitalWrite(MOTOR_B_R_B, LOW);
  digitalWrite(MOTOR_B_L_F, LOW);
  digitalWrite(MOTOR_B_L_B, LOW);
  digitalWrite(MOTOR_B_R_F, LOW);
  digitalWrite(MOTOR_B_R_B, LOW);
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


#define CONTROL_MOTOR(front_pin, back_pin, power_pin) \
  digitalWrite(front_pin, !sign);   \
  digitalWrite(back_pin, !!sign);   \
  analogWrite(power_pin, bin_val);  \
  break

void motor_chanel(uint8_t chanel, int16_t value) {
#warning motor_chanel disabled
  return;

  if (value > 255) value = 255;
  else if (value < -255) value = -255;

  uint8_t bin_val = (uint8_t)(uint16_t)(value > 0 ? value : -value);
  uint8_t sign = !!(value & (1 << 15));

  switch(chanel) {
  case CHS::L: CONTROL_MOTOR(MOTOR_E_L_F, MOTOR_E_L_B, MOTOR_E_L_P); // left engine
  case CHS::R: CONTROL_MOTOR(MOTOR_E_R_F, MOTOR_E_R_B, MOTOR_E_R_P); // right engine
  case CHS::l: CONTROL_MOTOR(MOTOR_B_L_F, MOTOR_B_L_B, MOTOR_B_L_P); // left ballast
  case CHS::r: CONTROL_MOTOR(MOTOR_B_R_F, MOTOR_B_R_B, MOTOR_B_R_P); // ring ballast
  default: break;
  }

  return;
}
