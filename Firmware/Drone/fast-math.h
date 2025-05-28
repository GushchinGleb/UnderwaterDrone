#pragma once

#define F_PI (3.141592653589793f)
#define F_1PI (1.f / F_PI)
#define F_2PI (F_PI * 2.f)
#define F_DEG_RAD (F_PI / 180.0f)
#define F_RAD_DEG (1.f / F_DEG_RAD)

struct f_vector2 {
  float x;
  float y;
};

/**
 * @brief fast sin of float
 * @param x[IN] range: [0; PI]
 */
float f_cos_f(float value);

/**
 * @brief fast sin of float
 * @param x[IN] range: [-PI/2; PI/2]
 */
float f_sin_f(float value);

/**
 * @brief The function calculates an required speed to reach and value with constant acceleration.
 * @param t[IN] target value (units)
 * @param a[IN] actual value (units)
 * @param d[in] target delta value (units per second^2)
 * @return required speed (units per second)
 */
float f_speed(float t, float a, float d);

/**
 * @brief The function calculates square root using lookup table.
 * @param v input value
 * @return sqrt(v)
 */
float f_sqrt(float v);

float f_rsqrt(float v);

/**
 * @brief The function calculates the target point on the segment for drone to lead it on the segment.
 * @param position[IN] drone position
 * @param start[IN] segment start
 * @param end[in] segment end
 * @param shift[IN] distance between target point and intersection of the segment and normal vector
 * @return target point
 */
struct f_vector2 f_point_on_segment(const struct f_vector2& position, const struct f_vector2& start, const struct f_vector2& end, const float& shift);

/**
 * @brief Length of a vector.
 * @param a[IN] vector
 * @return length
 */
float f_v_length(const struct f_vector2& a);
/**
 * @brief 1 divided on length of a vector.
 * @param a[IN] vector
 * @return 1 divide length
 */
float f_v_rlength(const struct f_vector2& a);

/**
 * @brief Scalar multiplication of 2 vectors.
 * @param a[IN] 1-st vector
 * @param b[IN] 2-nd vector
 * @return scalar multiplication
 */
float f_v_scalar(const struct f_vector2& a, const struct f_vector2& b);
