#include "solver.h"

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
