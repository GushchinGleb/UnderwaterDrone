#include <stdio.h>
#include <math.h>

#define N 256  // Number of lookup table elements
#define PI 3.14159265358979323846

double interpolation_sin(double x1, double x2, double* y2);

double linear_diff(double x1, double x2);

int main() {
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
  return 0;
}

double P(double x, double x1, double x2) {
  const double y1 = sin(x1);
  const double y2 = sin(x2);
  const double dy = y2 - y1;
  const double dx = x2 - x1;
  
  const double a = -dy / dx * 0.5;
  const double b = -y1 + dy / dx * x1;

  return -cos(x) + a * x * x + b * x;
}

double interpolation_sin(double x1, double x2, double* y2) {
  double y1 = sin(x1);

  double diff = (P(x2, x1, x2) - P(x1, x1, x2)) / (x2 - x1);

  *y2 = sin(x2) + diff;
  return y1 + diff;
}
