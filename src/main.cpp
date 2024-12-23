#include <Eigen/Eigen>
#include <iostream>
int main() {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix(3, 3);
  matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  std::cout << matrix.inverse();
}