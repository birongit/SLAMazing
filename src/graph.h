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
  std::vector<SE2> pose_vertices;
  std::unordered_map<int, SE2> landmark_vertices;
  std::vector<Edge> edges;
};
