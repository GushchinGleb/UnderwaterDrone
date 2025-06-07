#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "tables.h"

#define N 193  // Number of lookup table elements
#define PI 3.14159265358979323846

void calculate_sin_table();
void calculate_sqrt_table();

float calculate_sqrt(float number);

double interpolation_sin(double x1, double x2, double* y2);
double interpolation_sqrt(double x1, double x2, double* y2);

double P_sin(double x, double x1, double x2);
double P_sqrt(double x, double x1, double x2);

int main() {
  float result = calculate_sqrt(2.f);
  printf("result: %f\n", result);
  return 0;
}

void calculate_sin_table() {
  printf("static const float lookup_table_%d[] = {\n", N);
  printf(" %.8ff,", 0.0f);
  for (int i = 1; i < N - 2; i++) {
    const double x1 = (double)i * (PI / 2) / (N - 1);  // Uniform sampling over [0, PI/2]
    const double x2 = (double)(i + 1) * (PI / 2) / (N - 1);
    double y2 = 0.0;
    const double y1 = interpolation_sin(x1, x2, &y2);
    printf(" %.8ff%s", (float)y1, (i < N - 1) ? "," : "");
    if (i % 16 == 15) printf("\n");

    if (i == N - 3) {
      printf(" %.8ff%s", (float)y2, (i < N - 1) ? "," : "");
    }
  }
  printf(" %.8ff\n", 1.0f);
  printf("};\n");
}

float calculate_sqrt(float v) {
  const uint32_t val = *(uint32_t*)&v; // set bin representation

  int8_t exp = (val >> 23 & 0xFF); // exponent

  // check edge cases (nan, inf)
  if ((uint8_t)exp == 0xFF) {
    return v;
  }

  exp -= 127; // remove shift
  uint8_t sign = val >> 31;
  uint32_t mantissa = val & 0x7FFFFFL;

  mantissa |= 0x800000L; // add 1. ... that ignores in IEEE. New range is [1, 2)

  mantissa <<= exp & 1; // if exponent is odd number increase mantissa 2 times because the lowest bit will be removed. New range is [2, 4)
  mantissa -= 0x800000L; // remove 1 from increased mantissa. New range is [1, 3)

  exp >>= 1;  // get "sqrt" from exp
  exp += 127; // shift exp back to IEEE format

  uint8_t addr = mantissa >> (25 - 8); // address of the cell in the lookup table (25 - 17 bits of mantissa)
  if (addr >= 192) { // some how overflow
    uint32_t result = sign << 31 | (uint8_t)exp << 23 | 0x7FFFFFL;

    return *(float*)&result;
  }

  const float value_low = sqrt_lookup_table_193[addr];
  const float value_high = sqrt_lookup_table_193[addr + 1];

  const float shift = (float)(mantissa & 0x1FFFFL) / (float)0x20000L; // get shift on linear approximation (last 17 bits of mantissa)
  const float value = value_low + (value_high - value_low) * shift; // get value on linear approximation

  uint32_t result = sign << 31 | (uint8_t)exp << 23 | ((*(uint32_t*)&value) & 0x7FFFFFL); // generate new IEEE754 float
  printf("%u\n", result);
  return *(float*)&result;
}

void calculate_sqrt_table() {
  printf("static const float sqrt_lookup_table_%d[] = {\n", N);
  printf(" %.8ff,", 1.0f);
  for (int i = 1; i < N - 1; i++) {
    const double x1 = (double)i * 3.0 / (N - 1) + 1.0;  // Uniform sampling over [0, 4]
    const double x2 = (double)(i + 1) * 3.0 / (N - 1) + 1.0;
    double y2 = 0.0;
    const double y1 = interpolation_sqrt(x1, x2, &y2);
    printf(" %.8ff%s", (float)y1, (i < N - 1) ? "," : "");
    if (i % 16 == 15) printf("\n");
  }
  printf(" %.8ff\n", 2.0f);
  printf("};\n");
}

double interpolation_sin(double x1, double x2, double* y2) {
  double y1 = sin(x1);

  double diff = (P_sin(x2, x1, x2) - P_sin(x1, x1, x2)) / (x2 - x1);

  *y2 = sin(x2) + diff;
  return y1 + diff;
}

double interpolation_sqrt(double x1, double x2, double* y2) {
  double y1 = sqrt(x1);

  double diff = (P_sqrt(x2, x1, x2) - P_sqrt(x1, x1, x2)) / (x2 - x1);

  *y2 = sqrt(x2) + diff;
  return y1 + diff;
}

double P_sin(double x, double x1, double x2) {
  const double y1 = sin(x1);
  const double y2 = sin(x2);
  const double dy = y2 - y1;
  const double dx = x2 - x1;

  const double a = -dy / dx * 0.5;
  const double b = -y1 + dy / dx * x1;

  return -cos(x) + a * x * x + b * x;
}

double P_sqrt(double x, double x1, double x2) {
  const double y1 = sqrt(x1);
  const double y2 = sqrt(x2);
  const double dy = y2 - y1;
  const double dx = x2 - x1;

  const double a = -dy / dx * 0.5;
  const double b = -y1 + dy / dx * x1;

  return (x * sqrt(x) / 1.5) + a * x * x + b * x;
}
