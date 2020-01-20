#ifndef SOLVER_H
#define SOLVER_H

#include "graph.h"

#include <vector>

void solve(const std::vector<std::vector<double>> &A,
           const std::vector<double> &b, std::vector<double> &x);

void solve(Graph &graph);

#endif // SOLVER_H
