#include <stdio.h>
#include <math.h>

#define N 6           // Number of known points
#define MAX_ITER 100000
#define LEARNING_RATE 0.001
#define TOLERANCE 1e-6

typedef struct {
    double x, y, z;
} Point3D;

// Known positions and radius
Point3D anchors[N] = {
  {    8,  1224, -3840},
  { 1188,    38,    18},
  {-2020,   578,    20},
  {  -16,   668,   142},
  { 3420, -1696,    18},
  { 4916,  1190, -3758}
};

const double g_mss = (1<<12)*1.0; // constant radius
const double g = g_mss / g_mss; // constant radius

// Compute cost function: sum of squared residuals
double cost(Point3D guess) {
    double sum = 0.0;
    for (int i = 0; i < N; i++) {
        double dx = guess.x - anchors[i].x;
        double dy = guess.y - anchors[i].y;
        double dz = guess.z - anchors[i].z;
        double dist2 = dx*dx + dy*dy + dz*dz;
        double res = dist2 - g*g;
        sum += res * res;
    }
    return sum;
}

// Compute gradient of the cost function
Point3D gradient(Point3D guess) {
    Point3D grad = {0};
    for (int i = 0; i < N; i++) {
        double dx = guess.x - anchors[i].x;
        double dy = guess.y - anchors[i].y;
        double dz = guess.z - anchors[i].z;
        double dist2 = dx*dx + dy*dy + dz*dz;
        double res = dist2 - g*g;
        grad.x += 4 * dx * res;
        grad.y += 4 * dy * res;
        grad.z += 4 * dz * res;
    }
    printf("x: %lf y: %lf z: %lf\n", grad.x, grad.y, grad.z);
    return grad;
}

// Gradient Descent Solver
Point3D solve_trilateration() {
    Point3D guess = {0.0, 0.0, 0.0}; // Initial guess
    for (int iter = 0; iter < MAX_ITER; iter++) {
        Point3D grad = gradient(guess);
        double norm = sqrt(grad.x * grad.x + grad.y * grad.y + grad.z * grad.z);
        if (norm != norm)
          break;
        if (norm < TOLERANCE)
            break;
        guess.x -= LEARNING_RATE * grad.x;
        guess.y -= LEARNING_RATE * grad.y;
        guess.z -= LEARNING_RATE * grad.z;
    }
    return guess;
}

int main() {
    for (int i = 0; i < N; ++i) {
      anchors[i].x /= g_mss;
      anchors[i].y /= g_mss;
      anchors[i].z /= g_mss;
    }

    Point3D result = solve_trilateration();
    printf("Estimated position: (%.6f, %.6f, %.6f)\n", result.x * g_mss, result.y * g_mss, result.z * g_mss);
    printf("Final error: %.8f\n", cost(result));
    
    for (int l = 0; l < N; ++l) {
      printf("l:%d %lf\n",
        l,
        sqrt(anchors[l].x * anchors[l].x + anchors[l].y * anchors[l].y + anchors[l].z * anchors[l].z));
    }
    return 0;
}

