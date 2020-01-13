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

  b = {0, 0, 0, 0, 0};
  solve(A, b, x);

  for (int i = 0; i < solution.size(); ++i) {
    EXPECT_NEAR(x[i], 0, 10e-14);
  }
}

TEST(SolverTests, SolveSimpleGraph3DOF) {
  int num_poses = 3;
  int num_landmarks = 2;
  // Construct the graph
  Graph graph(num_poses);

  std::vector<SE2> pose_vertices = {{0.0, 0.0, 0.0},
                                    {6.0, 0.0, M_PI / 2.0},
                                    {6.0, 4.0, -M_PI},
                                    {0.0, 4.0, -M_PI / 2.0},
                                    {0.0, 0.0, 0.0}};
  graph.pose_vertices = pose_vertices;

  std::unordered_map<int, SE2> landmark_vertices = {{0, {-1.0, 2.0, -M_PI}},
                                                    {1, {5.0, 1.0, -M_PI}}};
  graph.landmark_vertices = landmark_vertices;

  auto &pv = graph.pose_vertices;
  auto &lv = graph.landmark_vertices;
  graph.edges = {{&pv[0], 0, &pv[1], 1, {6.0, 0, M_PI / 2.0}},
                 {&pv[1], 1, &lv[1], num_poses + 1, {1.0, 1.0, M_PI / 2.0}},
                 {&pv[1], 1, &pv[2], 2, {4.0, 0, M_PI / 2.0}},
                 {&pv[2], 2, &lv[1], num_poses + 1, {1.0, 3.0, 0.0}},
                 {&pv[2], 2, &pv[3], 3, {6.0, 0, M_PI / 2.0}},
                 {&pv[3], 3, &lv[0], num_poses + 0, {2.0, -1.0, -M_PI / 2.0}},
                 {&pv[3], 3, &pv[4], 4, {4.0, 0, M_PI / 2.0}},
                 {&pv[4], 4, &lv[0], num_poses + 0, {-1.0, 2.0, M_PI}},
                 {&pv[4], 4, &lv[1], num_poses + 1, {5.0, 1.0, M_PI}}};

  // Optimize graph
  solve(graph);

  for (int i = 0; i < num_poses; ++i) {
    EXPECT_NEAR(graph.pose_vertices[i].x, pose_vertices[i].x, 10e-14);
    EXPECT_NEAR(graph.pose_vertices[i].y, pose_vertices[i].y, 10e-14);
    EXPECT_NEAR(graph.pose_vertices[i].t, pose_vertices[i].t, 10e-14);
  }

  for (int i = 0; i < num_landmarks; ++i) {
    EXPECT_NEAR(graph.landmark_vertices[i].x, landmark_vertices[i].x, 10e-14);
    EXPECT_NEAR(graph.landmark_vertices[i].y, landmark_vertices[i].y, 10e-14);
    EXPECT_NEAR(graph.landmark_vertices[i].t, landmark_vertices[i].t, 10e-14);
  }
}
