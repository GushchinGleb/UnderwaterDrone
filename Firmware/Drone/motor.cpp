#include <Arduino.h>

#include "motor.h"

// Pins
#define MOTOR_L_F  (7) // left forward
#define MOTOR_L_B  (8) // left backward
#define MOTOR_R_F  (9) // right forward
#define MOTOR_R_B (10) // right backward

#define MAX_VAL (255)

void motor_init() {
    pinMode(MOTOR_L_F, OUTPUT);
    pinMode(MOTOR_L_B, OUTPUT);
    pinMode(MOTOR_R_F, OUTPUT);
    pinMode(MOTOR_R_B, OUTPUT);

    analogWrite(MOTOR_L_F, 0);
    analogWrite(MOTOR_L_B, 0);
    analogWrite(MOTOR_R_F, 0);
    analogWrite(MOTOR_R_B, 0);
}

void motor_move(float x, float a) {
    float r = x - a; // right motor
    float l = x + a; // left motor

    int16_t ar = (int16_t)(r * (float)MAX_VAL); // analog output to the right motor
    int16_t al = (int16_t)(l * (float)MAX_VAL); // analog output to the left motor

    ar = max(min(MAX_VAL, ar), -MAX_VAL);
    al = max(min(MAX_VAL, al), -MAX_VAL);

    if (ar >= 0) {
        analogWrite(MOTOR_R_B, 0);
        analogWrite(MOTOR_R_F, ar);
    } else {
        analogWrite(MOTOR_R_F, 0);
        analogWrite(MOTOR_R_B, -ar);
    }

    if (al >= 0) {
        analogWrite(MOTOR_L_B, 0);
        analogWrite(MOTOR_L_F, al);
    } else {
        analogWrite(MOTOR_L_F, 0);
        analogWrite(MOTOR_L_B, -al);
    }

    Serial.print(al);
    Serial.print(' ');
    Serial.println(ar);
}
