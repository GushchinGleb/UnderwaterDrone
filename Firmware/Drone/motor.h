#pragma once

/**
 * @brief The function initializes motors pins and test them
 */
void motor_init();

/** @brief
 * - The function controle motors for moving the drone.
 * - The motors will rotate until nex function call.
 * @param x[IN] move along x axis [-1; 1].
 * @param a[IN] rotate [-1, 1]. Positive -- right.
 */
void motor_move(float x, float a);

/**
 * @brief
 * - The function send value to the chanel.
 * - The motors will rotate until nex function call.
 * @param chanel[IN] see CHS from RF24-rec.h
 * @param value[IN] value [-255, 255]. Positive -- front / up / out.
 */
void motor_chanel(uint8_t chanel, int16_t value);
