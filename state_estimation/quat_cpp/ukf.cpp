#include "ukf.hpp"
#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd, Eigen::VectorXd;

ukf::ukf(Eigen::VectorXd x_, Eigen::MatrixXd P_, Eigen::MatrixXd Q0_, double alpha,
    double kappa, double beta, std::function<Eigen::VectorXd(Eigen::VectorXd)> fx, std::function<Eigen::VectorXd(Eigen::VectorXd)> hx)
    : x_(x_), P_(P_), Q0_(Q0_), alpha(alpha), kappa(kappa), beta(beta), fx(fx), hx(hx) {
        n_x = x_.size();
        generate_weights();
    };


VectorXd full(int size, double c) {
    // Create VectorXd of size size with all values c
    VectorXd result(size);
    for (int i = 0; i < size; i++) {
        result(i) = c;
        std::cout << result(i) << std::endl;
    } 
    return result;
}

void ukf::generate_weights(){
    // Generate covariance and mean weights
    double lambda = alpha * alpha * (n_x + kappa) - n_x;
    double c = 0.5/(double(n_x)+lambda);

    // Generate rows all with the value c
    Wc = full(2 * n_x + 1, c);
    Wm = full(2 * n_x + 1, c);
    Wc(0) = lambda / (n_x + lambda) + (1 - alpha * alpha + beta);
    Wm(0) = lambda / (n_x + lambda);
    // std::cout << "Wm: " << Wm << std::endl;
};







