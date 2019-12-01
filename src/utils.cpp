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

  std::vector<std::vector<double>> At(size, std::vector<double>(size, 0));
  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      At[r][c] = A[c][r];
    }
  }
  std::vector<std::vector<double>> Bt(size, std::vector<double>(size, 0));
  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      Bt[r][c] = B[c][r];
    }
  }

  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      for (int s = 0; s < size; ++s) {
        H[idx_i + r][idx_i + c] += At[r][s] * A[s][c];
        H[idx_i + r][idx_j + c] += At[r][s] * B[s][c];
        H[idx_j + r][idx_i + c] += Bt[r][s] * A[s][c];
        H[idx_j + r][idx_j + c] += Bt[r][s] * B[s][c];
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

  std::vector<std::vector<double>> At(size, std::vector<double>(size, 0));
  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      At[r][c] = A[c][r];
    }
  }
  std::vector<std::vector<double>> Bt(size, std::vector<double>(size, 0));
  for (int r = 0; r < size; ++r) {
    for (int c = 0; c < size; ++c) {
      Bt[r][c] = B[c][r];
    }
  }

  for (int r = 0; r < size; ++r) {
    for (int s = 0; s < size; ++s) {
      b[idx_i + r] += At[r][s] * e[s];
      b[idx_j + r] += Bt[r][s] * e[s];
    }
  }
}
