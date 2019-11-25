#include <iostream>

#include "src/solver.h"
#include <math.h>

struct SE2 {
  double x;
  double y;
  double t;
};

struct landmark_SE2 {
  int id;
  SE2 pose;
};

void solve_2D() {
  int num_landmarks = 2;
  int num_poses = 4;

  int size = num_landmarks + num_poses;

  // Measurements
  std::vector<std::pair<SE2, std::vector<landmark_SE2>>> measurements(
      num_poses, std::pair<SE2, std::vector<landmark_SE2>>{});

  measurements[0].first = {6.0, 0, M_PI / 2.0};
  measurements[0].second.push_back({1, {1.0, 1.0, M_PI / 2.0}});

  measurements[1].first = {4.0, 0, M_PI / 2.0};
  measurements[1].second.push_back({1, {1.0, 3.0, 0.0}});

  measurements[2].first = {6.0, 0, M_PI / 2.0};
  measurements[2].second.push_back({0, {2.0, -1.0, -M_PI / 2.0}});

  measurements[3].first = {4.0, 0, M_PI / 2.0};
  measurements[3].second.push_back({0, {-1.0, 2.0, M_PI}});
  measurements[3].second.push_back({1, {5.0, 1.0, M_PI}});

  bool converged = false;
  while (!converged) {
    std::vector<std::vector<double>> A(size, std::vector<double>(3 * size, 0));
    std::vector<double> b(3 * size, 0);
    std::vector<double> sol(3 * size, 0);

    // Fix initial pose
    A[0][0] += 1;
    A[1][1] += 1;
    A[2][2] += 1;
    b[0] += 1;
    b[1] += 1;
    b[2] += 1;

    SE2 prev_p{0, 0, 0};
    for (const auto &m : measurements) {
      // pose
      SE2 pose{};
      SE2 odo = m.first;
      pose.x = prev_p.x + cos(prev_p.t) * odo.x - sin(prev_p.t) * odo.y;
      pose.y = prev_p.y + sin(prev_p.t) * odo.x + cos(prev_p.t) * odo.y;
      pose.t = prev_p.t + odo.t;

      std::cout << "p " << pose.x << " " << pose.y << " " << pose.t
                << std::endl;

      prev_p = pose;

      for (const auto &l : m.second) {
        SE2 landmark{};
        landmark.x = pose.x + cos(pose.t) * l.pose.x - sin(pose.t) * l.pose.y;
        landmark.y = pose.y + sin(pose.t) * l.pose.x + cos(pose.t) * l.pose.y;
        landmark.t = pose.t + l.pose.t;

        std::cout << l.id << " " << landmark.x << " " << landmark.y << " "
                  << landmark.t << std::endl;
      }
    }

    converged = true;
  }
}

void solve_1D() {
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
}

int main() {
  solve_1D();

  solve_2D();

  return 0;
}
