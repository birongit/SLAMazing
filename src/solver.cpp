#include "solver.h"

#include <iostream>

extern "C" {
#include "csparse/cs.h"
}

void solve(const std::vector<std::vector<double>> &A,
           const std::vector<double> &B, std::vector<double> &x) {
  cs *a = cs_spalloc(0, 0, 1, 1, 1);
  for (int row = 0; row < A.size(); ++row) {
    for (int col = 0; col < A.size(); ++col) {
      if (A[row][col] != 0) {
        cs_entry(a, row, col, A[row][col]);
      }
    }
  }

  cs *a_comp = cs_compress(a);

  double *b =
      reinterpret_cast<double *>(cs_calloc(A[0].size(), sizeof(double)));
  std::memcpy(b, B.data(), B.size() * sizeof(double));

  if (cs_cholsol(1, a_comp, b)) {
    std::memcpy(x.data(), b, B.size() * sizeof(double));
  } else {
    printf("Error: cs_cholsol not successful!\n");
  }

  cs_free(b);
  cs_spfree(a_comp);
  cs_spfree(a);
}

void solve(Graph &graph) {
  uint num_poses = graph.pose_vertices.size();
  uint num_landmarks = graph.landmark_vertices.size();
  uint size = num_poses + num_landmarks;
  std::cout << "size = " << size << std::endl;

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

    // Apply result to Graph
    for (int i = 0; i < num_poses; ++i) {
      graph.pose_vertices[i].x -= sol[i * 3 + 0];
      graph.pose_vertices[i].y -= sol[i * 3 + 1];
      graph.pose_vertices[i].t -= sol[i * 3 + 2];
    }
    for (int i = 0; i < num_landmarks; ++i) {
      graph.landmark_vertices[i].x -= sol[(num_poses + i) * 3 + 0];
      graph.landmark_vertices[i].y -= sol[(num_poses + i) * 3 + 1];
      graph.landmark_vertices[i].t -= sol[(num_poses + i) * 3 + 2];
    }

    double max = 0.0;
    std::for_each(sol.begin(), sol.end(),
                  [&](double s) { max = std::max(std::abs(s), max); });

    // Test for convergence
    if (max < 0.00001) {
      converged = true;
      return;
    }
  }
}
