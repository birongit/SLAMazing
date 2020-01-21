#include <gtest/gtest.h>

#include "src/solver.h"

#include "test_graph.h"

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

  b = {0, 0, 0, 0, 0};
  solve(A, b, x);

  for (int i = 0; i < solution.size(); ++i) {
    EXPECT_NEAR(x[i], 0, 10e-14);
  }
}

TEST(SolverTests, SolveSimpleGraph3DOF) {
  constexpr double THRESH = 10e-14;

  // Construct the test graph
  TestGraph graph{};

  // Optimize graph
  solve(graph);

  // Check graph for correct solution
  graph.check_graph(THRESH);
}

TEST(SolverTests, SolveNoisyGraph3DOF) {
  constexpr double THRESH = 10e-10;
  constexpr double NOISE[] = {-0.1, 0.1};

  // Test noisy poses
  // Cannot apply noise to 0th vertex because initial vertex is fixed
  for (int i = 1; i < TestGraph::get_correct_poses().size(); ++i) {
    for (const auto &noise : NOISE) {
      // Construct the test graph
      TestGraph graph{};

      // Add noise
      graph.pose_vertices[i].x += noise;
      graph.pose_vertices[i].y += noise;
      graph.pose_vertices[i].t += noise;

      // Optimize graph
      solve(graph);

      // Check graph for correct solution
      graph.check_graph(THRESH);
    }
  }

  // Test noisy landmarks
  for (int i = 0; i < TestGraph::get_correct_landmarks().size(); ++i) {
    for (const auto &noise : NOISE) {
      // Construct the test graph
      TestGraph graph{};

      // Add noise
      graph.landmark_vertices[i].x += noise;
      graph.landmark_vertices[i].y += noise;
      graph.landmark_vertices[i].t += noise;

      // Optimize graph
      solve(graph);

      // Check graph for correct solution
      graph.check_graph(THRESH);
    }
  }
}
