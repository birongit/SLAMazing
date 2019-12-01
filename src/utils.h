#include <vector>

struct SE2 {
  double x;
  double y;
  double t;
};

double normalize_angle(double angle);

void addJacobians(int i, int j, const std::vector<std::vector<double>> &A,
                  const std::vector<std::vector<double>> &B,
                  std::vector<std::vector<double>> &H);

void addJacobians(int i, int j, const std::vector<std::vector<double>> &A,
                  const std::vector<std::vector<double>> &B,
                  const std::vector<double> &e, std::vector<double> &b);
