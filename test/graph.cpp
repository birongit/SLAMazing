#include <gtest/gtest.h>

#include <src/graph.h>

#include <iostream>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(GraphTests, BuildSimpleGraph) {
  int num_poses = 5;

  struct landmark_SE2 {
    int id;
    SE2 pose;
  };

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

  // Check poses
  std::vector<SE2> correct_poses = {{0.0, 0.0, 0.0},
                                    {6.0, 0.0, M_PI / 2.0},
                                    {6.0, 4.0, -M_PI},
                                    {0.0, 4.0, -M_PI / 2.0},
                                    {0.0, 0.0, 0.0}};

  ASSERT_EQ(graph.pose_vertices.size(), correct_poses.size());
  for (int i = 0; i < correct_poses.size(); ++i) {
    EXPECT_NEAR(graph.pose_vertices[i].x, correct_poses[i].x, 10e-14);
    EXPECT_NEAR(graph.pose_vertices[i].y, correct_poses[i].y, 10e-14);
    EXPECT_NEAR(graph.pose_vertices[i].t, correct_poses[i].t, 10e-14);
  }

  // Check landmarks
  std::unordered_map<int, SE2> correct_landmarks = {{0, {-1.0, 2.0, -M_PI}},
                                                    {1, {5.0, 1.0, -M_PI}}};
  std::vector<Edge> edges;

  ASSERT_EQ(graph.landmark_vertices.size(), correct_landmarks.size());
  for (int i = 0; i < correct_landmarks.size(); ++i) {
    EXPECT_NEAR(graph.landmark_vertices[i].x, correct_landmarks[i].x, 10e-14);
    EXPECT_NEAR(graph.landmark_vertices[i].y, correct_landmarks[i].y, 10e-14);
    EXPECT_NEAR(graph.landmark_vertices[i].t, correct_landmarks[i].t, 10e-14);
  }

  // Check edges
  std::vector<std::pair<SE2 *, SE2 *>> correct_edges = {
      {&graph.pose_vertices[0], &graph.pose_vertices[1]},
      {&graph.pose_vertices[1], &graph.pose_vertices[2]},
      {&graph.pose_vertices[2], &graph.pose_vertices[3]},
      {&graph.pose_vertices[3], &graph.pose_vertices[4]},
      {&graph.pose_vertices[1], &graph.landmark_vertices[1]},
      {&graph.pose_vertices[2], &graph.landmark_vertices[1]},
      {&graph.pose_vertices[3], &graph.landmark_vertices[0]},
      {&graph.pose_vertices[4], &graph.landmark_vertices[0]},
      {&graph.pose_vertices[4], &graph.landmark_vertices[1]}};

  auto find_edge = [](const std::pair<SE2 *, SE2 *> &correct_edge,
                      const std::vector<Edge> &edges) {
    return (std::find_if(edges.begin(), edges.end(),
                         [correct_edge](const Edge &edge) {
                           return ((correct_edge.first == edge.vertex_0) ||
                                   (correct_edge.first == edge.vertex_1)) &&
                                  ((correct_edge.second == edge.vertex_0) ||
                                   (correct_edge.second == edge.vertex_1));
                         }) != edges.end());
  };

  for (const auto &correct_edge : correct_edges) {
    EXPECT_TRUE(find_edge(correct_edge, graph.edges));
  }
}
