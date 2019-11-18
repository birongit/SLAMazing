#include <iostream>

#include "src/solver.h"

// Test solver
int main() {
  int num_landmarks = 2;
  int num_poses = 3;

  int size = num_landmarks + num_poses;
  std::vector<std::vector<double>> A(size, std::vector<double>(size, 0));
  std::vector<double> b(size, 0);
  std::vector<double> sol(size, 0);

  // Equations
  // x0 - x1 = -5
  // x1 - x2 = -4
  // x0 - l0 = -4
  // x1 - l0 = 1
  // x1 - l1 = -2
  // x2 - l1 = 2

  A[0][0] += 1;

  A[0][0] += 1;
  A[0][1] += -1;
  b[0] += -5;

  A[1][0] += -1;
  A[1][1] += 1;
  b[1] += 5;

  A[1][1] += 1;
  A[1][2] += -1;
  b[1] += -4;

  A[2][1] += -1;
  A[2][2] += 1;
  b[2] += 4;

  A[0][0] += 1;
  A[0][3] += -1;
  b[0] += -4;

  A[3][0] += -1;
  A[3][3] += 1;
  b[3] += 4;

  A[1][1] += 1;
  A[1][3] += -1;
  b[1] += 1;

  A[3][1] += -1;
  A[3][3] += 1;
  b[3] += -1;

  A[1][1] += 1;
  A[1][4] += -1;
  b[1] += -2;

  A[4][1] += -1;
  A[4][4] += 1;
  b[4] += 2;

  A[2][2] += 1;
  A[2][4] += -1;
  b[2] += 2;

  A[4][2] += -1;
  A[4][4] += 1;
  b[4] += -2;

  std::cout << "b" << std::endl;
  for (int i = 0; i < b.size(); ++i) {
    std::cout << b[i] << " " << std::endl;
  }

  std::cout << "A" << std::endl;
  for (int row = 0; row < A.size(); ++row) {
    for (int col = 0; col < A[row].size(); ++col) {
      std::cout << A[row][col] << " ";
    }
    std::cout << std::endl;
  }

  solve(A, b, sol);

  std::cout << "result" << std::endl;
  for (int i = 0; i < sol.size(); ++i) {
    std::cout << sol[i] << " " << std::endl;
  }

  return 0;
}