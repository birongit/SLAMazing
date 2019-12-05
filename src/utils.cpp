#include "utils.h"

#include <math.h>

double normalize_angle(double angle) {
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0)
    angle += 2 * M_PI;
  return angle - M_PI;
}

void addJacobians(int i, int j, const std::vector<std::vector<double>> &A,
                  const std::vector<std::vector<double>> &B,
                  std::vector<std::vector<double>> &H) {

  int size = A.size();

  int idx_i = size * i;
  int idx_j = size * j;

  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      for (int s = 0; s < size; ++s) {
        H[idx_i + r][idx_i + c] += A[s][r] * A[s][c];
        H[idx_i + r][idx_j + c] += A[s][r] * B[s][c];
        H[idx_j + r][idx_i + c] += B[s][r] * A[s][c];
        H[idx_j + r][idx_j + c] += B[s][r] * B[s][c];
      }
    }
  }
}

void addJacobians(int i, int j, const std::vector<std::vector<double>> &A,
                  const std::vector<std::vector<double>> &B,
                  const std::vector<double> &e, std::vector<double> &b) {
  int size = A.size();

  int idx_i = size * i;
  int idx_j = size * j;

  for (int r = 0; r < size; ++r) {
    for (int s = 0; s < size; ++s) {
      b[idx_i + r] += A[s][r] * e[s];
      b[idx_j + r] += B[s][r] * e[s];
    }
  }
}
