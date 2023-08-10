#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <eigen3/Eigen/Dense>



class ukf
{
public:
    // State dimensions
    int n_x;
    // Measurement dimensions
    int n_R;
    double alpha;
    double kappa;
    double beta;
    Eigen::VectorXd Wc;
    Eigen::VectorXd Wm;
    // Define state vector
    Eigen::VectorXd x_;
    // Define state covariance matrix
    Eigen::MatrixXd P_;
    // Define sigma point matrix
    Eigen::MatrixXd Xsig_;

    // Define Q0
    Eigen::MatrixXd Q0_;

    // Define R0
    Eigen::MatrixXd R0_;

    // Eigen::MatrixXd weights = generate_weights();

    // Define the Prediction function
    std::function<Eigen::VectorXd(Eigen::VectorXd)> fx;
    // Define the measurement function
    std::function<Eigen::VectorXd(Eigen::VectorXd)> hx;

    ukf(Eigen::VectorXd x_, Eigen::MatrixXd P_, Eigen::MatrixXd Q0_, double alpha,
        double kappa, double beta, std::function<Eigen::VectorXd(Eigen::VectorXd)> fx, std::function<Eigen::VectorXd(Eigen::VectorXd)> hx);

    // float x0, float P0[sizeof(x0)][sizeof(x0)],
    // float Q0[sizeof(x0)][sizeof(x0)], float R0, float alpha,
    // float kappa, float beta, function fx, function hx)
    void generate_weights();
    Eigen::MatrixXd generate_sigma_points(Eigen::VectorXd x, Eigen::MatrixXd P);
    Eigen::VectorXd vec2quat(const std::vector<double> &vec, const std::vector<double> &dt = {});
    // Quat quat2vec(const Quat &q);
    Eigen::MatrixXd kraftSigmaPnts();
    Eigen::VectorXd predict();
    Eigen::VectorXd update();
    Eigen::MatrixXd computer_process_sigmas(float dt, std::function<Eigen::VectorXd(Eigen::VectorXd)> fx = {},
                                            std::function<Eigen::VectorXd(Eigen::VectorXd)> hx = {});
    Eigen::MatrixXd markleyAverage(Eigen::MatrixXd Qmat);
};