#ifndef GRAPH_H
#define GRAPH_H

#include "utils.h"

#include <unordered_map>
#include <vector>

struct Edge {
  SE2 *vertex_0;
  int idx_0;
  SE2 *vertex_1;
  int idx_1;
  SE2 connection;
};

class Graph {
public:
  Graph(int num_poses);
  void add_motion(const SE2 &odo);
  void add_measurement(int id, const SE2 &measurement);

  std::vector<SE2> pose_vertices;
  std::unordered_map<int, SE2> landmark_vertices;
  std::vector<Edge> edges;

protected:
  int num_poses;
};

#endif // GRAPH_H
