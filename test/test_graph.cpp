#include "test_graph.h"

#include <gtest/gtest.h>

const std::vector<SE2> TestGraph::correct_pose_vertices = {
    {0.0, 0.0, 0.0},
    {6.0, 0.0, M_PI / 2.0},
    {6.0, 4.0, -M_PI},
    {0.0, 4.0, -M_PI / 2.0},
    {0.0, 0.0, 0.0}};

const std::unordered_map<int, SE2> TestGraph::correct_landmark_vertices{
    {0, {-1.0, 2.0, -M_PI}}, {1, {5.0, 1.0, -M_PI}}};

TestGraph::TestGraph() : Graph(correct_pose_vertices.size()) {

  pose_vertices = correct_pose_vertices;
  landmark_vertices = correct_landmark_vertices;

  auto &pv = pose_vertices;
  auto &lv = landmark_vertices;
  int num_poses = correct_pose_vertices.size();
  edges = {{&pv[0], 0, &pv[1], 1, {6.0, 0, M_PI / 2.0}},
           {&pv[1], 1, &lv[1], num_poses + 1, {1.0, 1.0, M_PI / 2.0}},
           {&pv[1], 1, &pv[2], 2, {4.0, 0, M_PI / 2.0}},
           {&pv[2], 2, &lv[1], num_poses + 1, {1.0, 3.0, 0.0}},
           {&pv[2], 2, &pv[3], 3, {6.0, 0, M_PI / 2.0}},
           {&pv[3], 3, &lv[0], num_poses + 0, {2.0, -1.0, -M_PI / 2.0}},
           {&pv[3], 3, &pv[4], 4, {4.0, 0, M_PI / 2.0}},
           {&pv[4], 4, &lv[0], num_poses + 0, {-1.0, 2.0, M_PI}},
           {&pv[4], 4, &lv[1], num_poses + 1, {5.0, 1.0, M_PI}}};
}

void TestGraph::check_graph(double tolerance) {
  auto correct_poses = get_correct_poses();
  ASSERT_EQ(correct_poses.size(), pose_vertices.size());
  for (int i = 0; i < correct_poses.size(); ++i) {
    ASSERT_NEAR(pose_vertices[i].x, correct_poses[i].x, tolerance);
    ASSERT_NEAR(pose_vertices[i].y, correct_poses[i].y, tolerance);
    ASSERT_NEAR(pose_vertices[i].t, correct_poses[i].t, tolerance);
  }

  auto correct_landmarks = get_correct_landmarks();
  ASSERT_EQ(correct_landmarks.size(), landmark_vertices.size());
  for (int i = 0; i < correct_landmarks.size(); ++i) {
    ASSERT_NEAR(landmark_vertices[i].x, correct_landmarks[i].x, tolerance);
    ASSERT_NEAR(landmark_vertices[i].y, correct_landmarks[i].y, tolerance);
    ASSERT_NEAR(landmark_vertices[i].t, correct_landmarks[i].t, tolerance);
  }
}

const std::vector<SE2> &TestGraph::get_correct_poses() {
  return correct_pose_vertices;
}

const std::unordered_map<int, SE2> &TestGraph::get_correct_landmarks() {
  return correct_landmark_vertices;
}
