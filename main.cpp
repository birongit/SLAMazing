#include <iostream>

#include "src/solver.h"
#include <math.h>

struct landmark_SE2 {
  int id;
  SE2 pose;
};

void solve_2D() {
  int num_landmarks = 2;
  int num_poses = 5;

  int size = num_landmarks + num_poses;

  // Measurements
  std::vector<std::pair<SE2, std::vector<landmark_SE2>>> measurements(
      num_poses - 1, std::pair<SE2, std::vector<landmark_SE2>>{});

  measurements[0].first = {6.0, 0, M_PI / 2.0};
  measurements[0].second.push_back({1, {1.0, 1.0, M_PI / 2.0}});

  measurements[1].first = {4.0, 0, M_PI / 2.0};
  measurements[1].second.push_back({1, {1.0, 3.0, 0.0}});

  measurements[2].first = {6.0, 0, M_PI / 2.0};
  measurements[2].second.push_back({0, {2.0, -1.0, -M_PI / 2.0}});

  measurements[3].first = {4.0, 0, M_PI / 2.0};
  measurements[3].second.push_back({0, {-1.0, 2.0, M_PI}});
  measurements[3].second.push_back({1, {5.0, 1.0, M_PI}});

  // Building the graph
  Graph graph(num_poses);

  for (const auto &m : measurements) {
    // Add pose to graph
    graph.add_motion(m.first);

    // Add landmarks to graph
    for (const auto &l : m.second) {
      graph.add_measurement(l.id, l.pose);
    }
  }

  // Test with arbitrary perturbations
  graph.pose_vertices[4].t -= 0.1;

  // Optimize graph
  solve(graph);

  std::cout << "Result poses:" << std::endl;
  for (int i = 0; i < num_poses; ++i) {
    std::cout << i << " " << graph.pose_vertices[i].x << " "
              << graph.pose_vertices[i].y << " " << graph.pose_vertices[i].t
              << std::endl;
  }
  std::cout << "Result landmarks:" << std::endl;
  for (int i = 0; i < num_landmarks; ++i) {
    std::cout << i << " " << graph.landmark_vertices[i].x << " "
              << graph.landmark_vertices[i].y << " "
              << graph.landmark_vertices[i].t << std::endl;
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
