#ifndef TEST_GRAPH_H
#define TEST_GRAPH_H

#include "src/graph.h"

class TestGraph : public Graph {
public:
  TestGraph();
  void check_graph(double tolerance);
  static const std::vector<SE2> &get_correct_poses();
  static const std::unordered_map<int, SE2> &get_correct_landmarks();

private:
  static const std::vector<SE2> correct_pose_vertices;
  static const std::unordered_map<int, SE2> correct_landmark_vertices;
};

#endif // TEST_GRAPH_H
