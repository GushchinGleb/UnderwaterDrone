#pragma once

#define F_PI (3.141592653589793f)
#define F_1PI (1.f / F_PI)
#define F_2PI (F_PI * 2.f)
#define F_DEG_RAD (F_PI / 180.0f)
#define F_RAD_DEG (1.f / F_DEG_RAD)

/**
 * @brief Fast calculation of sin
 * @param value[IN] range: [0; PI / 2]
 */
float f_sin_f(float value);