#include <Arduino.h>

#include "motor.h"

// Pins
#define MOTOR_L_F (23) // left forward
#define MOTOR_L_B (25) // left backward
#define MOTOR_L_P  (7) // left power
#define MOTOR_R_F (27) // right forward
#define MOTOR_R_B (29) // right backward
#define MOTOR_R_P  (8) // right power

#define MAX_VAL (255)

void motor_init() {
    pinMode(MOTOR_L_P, OUTPUT);
    pinMode(MOTOR_R_P, OUTPUT);
    analogWrite(MOTOR_L_P, 0);
    analogWrite(MOTOR_R_P, 0);

    pinMode(MOTOR_L_F, OUTPUT);
    pinMode(MOTOR_L_B, OUTPUT);
    pinMode(MOTOR_R_F, OUTPUT);
    pinMode(MOTOR_R_B, OUTPUT);
    digitalWrite(MOTOR_L_F, LOW);
    digitalWrite(MOTOR_L_B, LOW);
    digitalWrite(MOTOR_R_F, LOW);
    digitalWrite(MOTOR_R_B, LOW);
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
