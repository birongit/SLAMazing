#include <gtest/gtest.h>

#include "src/solver.h"

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(SolverTests, SimpleSolver) {
  int num_landmarks = 2;
  int num_poses = 3;

  int size = num_landmarks + num_poses;
  std::vector<std::vector<double>> A(size, std::vector<double>(size, 0));
  std::vector<double> b(size, 0);
  std::vector<double> x(size, 0);

  // Equations
  // x0 - x1 = -5
  // x1 - x2 = -4
  // x0 - l0 = -4
  // x1 - l0 = 1
  // x1 - l1 = -2
  // x2 - l1 = 2

  A = {{3, -1, 0, -1, 0},
       {-1, 4, -1, -1, -1},
       {0, -1, 2, 0, -1},
       {-1, -1, 0, 2, 0},
       {0, -1, -1, 0, 2}};

  b = {-9, 0, 6, 3, 0};

  solve(A, b, x);

  std::vector<double> solution = {0, 5, 9, 4, 7};

  ASSERT_EQ(x.size(), solution.size());
  for (int i = 0; i < solution.size(); ++i) {
    EXPECT_NEAR(x[i], solution[i], 10e-14);
  }
}
