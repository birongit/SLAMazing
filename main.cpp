#include <iostream>

#include "src/graph.h"
#include "src/solver.h"
#include <math.h>

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
      num_poses, std::pair<SE2, std::vector<landmark_SE2>>{}); // num_poses

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
  Graph graph{};
  graph.pose_vertices.reserve(
      num_poses + 1); // Allocate enough space so that no reallocation happens

  SE2 prev_p{0, 0, 0};
  graph.pose_vertices.push_back(prev_p);

  for (const auto &m : measurements) {
    // pose
    SE2 pose{};
    SE2 odo = m.first;
    pose.x = prev_p.x + cos(prev_p.t) * odo.x - sin(prev_p.t) * odo.y;
    pose.y = prev_p.y + sin(prev_p.t) * odo.x + cos(prev_p.t) * odo.y;
    pose.t = normalize_angle(prev_p.t + odo.t);

    std::cout << "p " << pose.x << " " << pose.y << " " << pose.t << std::endl;

    // Add poses to graph
    Edge edge{};
    edge.vertex_0 = &graph.pose_vertices.back();
    edge.idx_0 = graph.pose_vertices.size() - 1;
    graph.pose_vertices.push_back(pose);
    edge.vertex_1 = &graph.pose_vertices.back();
    edge.idx_1 = graph.pose_vertices.size() - 1;
    edge.connection = odo;
    graph.edges.push_back(edge);

    prev_p = pose;

    // landmarks
    for (const auto &l : m.second) {
      SE2 landmark{};
      landmark.x = pose.x + cos(pose.t) * l.pose.x - sin(pose.t) * l.pose.y;
      landmark.y = pose.y + sin(pose.t) * l.pose.x + cos(pose.t) * l.pose.y;
      landmark.t = normalize_angle(pose.t + l.pose.t);

      std::cout << l.id << " " << landmark.x << " " << landmark.y << " "
                << landmark.t << std::endl;

      // Inserts only first time visited
      graph.landmark_vertices.insert(std::make_pair(l.id, landmark));

      Edge edge{};
      edge.vertex_0 = &graph.pose_vertices.back();
      edge.idx_0 = graph.pose_vertices.size() - 1;
      edge.vertex_1 = &graph.landmark_vertices[l.id];
      edge.idx_1 = num_poses + l.id;
      edge.connection = l.pose;
      graph.edges.push_back(edge);
    }
  }

  bool converged = false;
  while (!converged) {
    std::vector<std::vector<double>> H(3 * size,
                                       std::vector<double>(3 * size, 0));
    std::vector<double> b(3 * size, 0);
    std::vector<double> sol(3 * size, 0);

    // Fix initial pose
    H[0][0] += 1;
    H[1][1] += 1;
    H[2][2] += 1;

    std::cout << "Edges: " << graph.edges.size() << std::endl;

    for (const auto &e : graph.edges) {
      std::vector<std::vector<double>> A(3, std::vector<double>(3, 0));
      std::vector<std::vector<double>> B(3, std::vector<double>(3, 0));
      SE2 err;

      // Calculate error
      double x_tmp = cos(e.vertex_0->t) * (e.vertex_1->x - e.vertex_0->x) +
                     sin(e.vertex_0->t) * (e.vertex_1->y - e.vertex_0->y) -
                     e.connection.x;
      double y_tmp = -sin(e.vertex_0->t) * (e.vertex_1->x - e.vertex_0->x) +
                     cos(e.vertex_0->t) * (e.vertex_1->y - e.vertex_0->y) -
                     e.connection.y;

      err.x = cos(e.connection.t) * x_tmp + sin(e.connection.t) * y_tmp;
      err.y = -sin(e.connection.t) * x_tmp + cos(e.connection.t) * y_tmp;
      err.t = normalize_angle(e.vertex_1->t - e.vertex_0->t - e.connection.t);

      std::cout << "Error: "; // Must be ~0
      std::cout << err.x << " " << err.y << " " << err.t << std::endl;

      // Calculate Jacobians
      std::vector<std::vector<double>> RiT(2, std::vector<double>(2, 0));
      RiT[0][0] = cos(e.vertex_0->t);
      RiT[0][1] = sin(e.vertex_0->t);
      RiT[1][0] = -sin(e.vertex_0->t);
      RiT[1][1] = cos(e.vertex_0->t);

      std::vector<std::vector<double>> RijT(2, std::vector<double>(2, 0));
      RijT[0][0] = cos(e.connection.t);
      RijT[0][1] = sin(e.connection.t);
      RijT[1][0] = -sin(e.connection.t);
      RijT[1][1] = cos(e.connection.t);

      std::vector<std::vector<double>> dRiT(2, std::vector<double>(2, 0));
      dRiT[0][0] = -sin(e.vertex_0->t);
      dRiT[0][1] = cos(e.vertex_0->t);
      dRiT[1][0] = -cos(e.vertex_0->t);
      dRiT[1][1] = -sin(e.vertex_0->t);

      A[0][0] = -RijT[0][0] * RiT[0][0] - RijT[0][1] * RiT[1][0];
      A[0][1] = -RijT[0][0] * RiT[0][1] - RijT[0][1] * RiT[1][1];
      A[1][0] = -RijT[1][0] * RiT[0][0] - RijT[1][1] * RiT[1][0];
      A[1][1] = -RijT[1][0] * RiT[0][1] - RijT[1][1] * RiT[1][1];

      std::vector<std::vector<double>> R_tmp(2, std::vector<double>(2, 0));
      R_tmp[0][0] = RijT[0][0] * dRiT[0][0] + RijT[0][1] * dRiT[1][0];
      R_tmp[0][1] = RijT[0][0] * dRiT[0][1] + RijT[0][1] * dRiT[1][1];
      R_tmp[1][0] = RijT[1][0] * dRiT[0][0] + RijT[1][1] * dRiT[1][0];
      R_tmp[1][1] = RijT[1][0] * dRiT[0][1] + RijT[1][1] * dRiT[1][1];

      A[0][2] = R_tmp[0][0] * (e.vertex_1->x - e.vertex_0->x) +
                R_tmp[0][1] * (e.vertex_1->y - e.vertex_0->y);
      A[1][2] = R_tmp[1][0] * (e.vertex_1->x - e.vertex_0->x) +
                R_tmp[1][1] * (e.vertex_1->y - e.vertex_0->y);

      A[2][0] = 0;
      A[2][1] = 0;
      A[2][2] = -1;

      B[0][0] = RijT[0][0] * RiT[0][0] + RijT[0][1] * RiT[1][0];
      B[0][1] = RijT[0][0] * RiT[0][1] + RijT[0][1] * RiT[1][1];
      B[1][0] = RijT[1][0] * RiT[0][0] + RijT[1][1] * RiT[1][0];
      B[1][1] = RijT[1][0] * RiT[0][1] + RijT[1][1] * RiT[1][1];

      B[0][2] = 0;
      B[1][2] = 0;

      B[2][0] = 0;
      B[2][1] = 0;
      B[2][2] = 1;

      // Add to Information matrix
      // Assume Omega = I
      addJacobians(e.idx_0, e.idx_1, A, B, H);
      addJacobians(e.idx_0, e.idx_1, A, B,
                   std::vector<double>{err.x, err.y, err.t}, b);
    }

    std::cout << "b" << std::endl;
    for (int i = 0; i < b.size(); ++i) {
      std::cout << b[i] << " " << std::endl;
    }

    std::cout << "H" << std::endl;
    for (int row = 0; row < H.size(); ++row) {
      for (int col = 0; col < H[row].size(); ++col) {
        std::cout << H[row][col] << " ";
      }
      std::cout << std::endl;
    }

    solve(H, b, sol);

    std::cout << "result" << std::endl;
    for (int i = 0; i < sol.size(); ++i) {
      std::cout << sol[i] << " " << std::endl;
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
