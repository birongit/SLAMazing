#include "graph.h"

#include <iostream>

// TODO: Make size dynamic
Graph::Graph(int num_poses) : num_poses(num_poses) {
  // Allocate enough space so that no reallocation happens
  pose_vertices.reserve(num_poses);
}

void Graph::add_motion(const SE2 &odo) {
  if (pose_vertices.empty()) {
    pose_vertices.push_back(SE2{0.0, 0.0, 0.0});
  }
  SE2 prev_p = pose_vertices.back();

  SE2 pose{};
  pose.x = prev_p.x + cos(prev_p.t) * odo.x - sin(prev_p.t) * odo.y;
  pose.y = prev_p.y + sin(prev_p.t) * odo.x + cos(prev_p.t) * odo.y;
  pose.t = normalize_angle(prev_p.t + odo.t);

  std::cout << "p " << pose.x << " " << pose.y << " " << pose.t << std::endl;

  // Add poses to graph
  Edge edge{};
  edge.vertex_0 = &pose_vertices.back();
  edge.idx_0 = pose_vertices.size() - 1;
  pose_vertices.push_back(pose);
  edge.vertex_1 = &pose_vertices.back();
  edge.idx_1 = pose_vertices.size() - 1;
  edge.connection = odo;
  edges.push_back(edge);
}

void Graph::add_measurement(int id, const SE2 &measurement) {
  SE2 landmark{};
  SE2 pose = pose_vertices.back();
  landmark.x =
      pose.x + cos(pose.t) * measurement.x - sin(pose.t) * measurement.y;
  landmark.y =
      pose.y + sin(pose.t) * measurement.x + cos(pose.t) * measurement.y;
  landmark.t = normalize_angle(pose.t + measurement.t);

  std::cout << id << " " << landmark.x << " " << landmark.y << " " << landmark.t
            << std::endl;

  // Inserts only first time visited
  landmark_vertices.insert(std::make_pair(id, landmark));

  Edge edge{};
  edge.vertex_0 = &pose_vertices.back();
  edge.idx_0 = pose_vertices.size() - 1;
  edge.vertex_1 = &landmark_vertices[id];
  edge.idx_1 = num_poses + id;
  edge.connection = measurement;
  edges.push_back(edge);
}
