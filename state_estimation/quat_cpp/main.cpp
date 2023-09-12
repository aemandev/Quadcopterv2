#include "Quat.hpp"
#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include "ukf.hpp"

using std::count, std::vector, std::function, std::cout, std::endl, std::ostream, std::copy, Eigen::VectorXd, Eigen::MatrixXd;

int main(){
    Quat q2(1, 2, 3, 4);
    std::cout << q2 << std::endl;

    Quat &q3 = q2.conjugate();
    std::cout << q3 << std::endl;

    Quat q4 = q2.norm();
    std::cout << q4 << std::endl;

    // print q2
    std::cout << q2 << std::endl;

    //print q3
    std::cout << q3 << std::endl;

    // Test vector constructor
    std::vector<float> v = {1, 2, 3};
    Quat q5(v, 0.01);
    std::cout << q5 << std::endl;
    
    std::vector<float> v2 = {1,2,3};
    Quat q6(v2);
    std::cout << q6 << std::endl;

    // Test multiplication
    Quat q7 = q5 * q6;

    // Test Rotate Vector
    // Initialize a unit quaternion
    Quat q8(-.713, .054, .696, 0.054);
    std::vector<float> v3 = {1, 2, 3};
    std::vector<float> v4 = q8.rotate_vector(v3);
    std::cout << v4[0] << " " << v4[1] << " " << v4[2] << std::endl;

    // Need to test the initialization of the ukf class
    // Make an eigen vector test
    VectorXd x(7);
    x << 1, 0, 0, 0, 0, 0, 0;
    std::cout << x << std::endl;

    // Make an eigen matrix test
    MatrixXd P(3, 3);
    P << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;
    std::cout << P << std::endl;

    // Make a function test
    function<VectorXd(VectorXd)> fx = [](VectorXd x) {
        VectorXd y(4);
        y << 1*2, 2*2, 3*2, 4*2;
        return y;
    };
    
    std::cout << fx(x) << std::endl;


    // Make a ukf test
    double alpha = 0.1;
    double kappa = -1;
    double beta = 2;
    ukf ukf_test(x, P, P, alpha, kappa, beta, fx, fx);
    // std::cout << "x: " << ukf_test.x_ << std::endl;
    // std::cout << "P_: " << ukf_test.P_ << std::endl;
    // std::cout << "Q0:" << ukf_test.Q0_ << std::endl;
    // std::cout << "alpha: " << ukf_test.alpha << std::endl;
    // std::cout << "kappa: " << ukf_test.kappa << std::endl;
    // std::cout << "beta: " << ukf_test.beta << std::endl;
    // std::cout << "fx: " << ukf_test.fx(x) << std::endl;
    // std::cout << "hx: " << ukf_test.hx(x) << std::endl;

    // Test the generate weights function
    std::cout << "Wc: " << ukf_test.Wc << std::endl;
    std::cout << "Wm: " << ukf_test.Wm << std::endl;
        
    return 0;
}