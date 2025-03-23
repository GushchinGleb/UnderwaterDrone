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
