#include <iostream>
#include "eigen-3.4.0/Eigen/Dense"

int main() {
    Eigen::MatrixXf A = Eigen::MatrixXf::Random(3, 2);
    Eigen::VectorXf b = Eigen::VectorXf::Random(3);
    std::cout << "The solution using the QR decomposition is:\n"
        << A.fullPivHouseholderQr().solve(b) << std::endl;
}