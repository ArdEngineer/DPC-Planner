# pragma once

# include <string>
# include <nlopt.hpp>
# include <ceres/ceres.h>
# include <Eigen/Core>
# include <vector>
# include <cmath>

# include "common_utils.h"

class TrajOptimizer{
public:
    double dt;
    double sf;
    double lambda_s;
    double lambda_d;
    double lambda_c;
    double cj, cmv, cma, cmj;
    double lamda;
    double omega_v, omega_a, omega_j;

    double xtol_rel;
    int maxeval;

    int num_of_control_points;
    int bspline_order;
    double average_v;

    void set_optimizer_params(PublicParams const&);

    void solve_optimize(
        std::vector<std::pair<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>>>&,
        std::vector<Eigen::Vector3d>&
    );

    std::vector<std::pair<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> optimize_content_;

    double penalty_function_jc(
        const std::vector<Eigen::Vector3d>&, 
        const std::vector<std::pair<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>>>&
    );;
    double penalty_function_jd(const std::vector<Eigen::Vector3d>&);
    double penalty_function_js(const std::vector<Eigen::Vector3d>&);

    std::vector<Eigen::Vector3d> penalty_function_Jacobi_jc(
        const std::vector<Eigen::Vector3d>&,
        const std::vector<std::pair<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>>>&
    );
    std::vector<Eigen::Vector3d> penalty_function_Jacobi_jd(const std::vector<Eigen::Vector3d>&);
    std::vector<Eigen::Vector3d> penalty_function_Jacobi_js(const std::vector<Eigen::Vector3d>&);

    double object_function(const std::vector<Eigen::Vector3d>&);
    std::vector<Eigen::Vector3d> object_function_Jacobi(const std::vector<Eigen::Vector3d>&);

    double f(double x, double cj, double cm, double lamda);
    double f_deriv(double x, double cj, double cm, double lamda);

    Eigen::Vector3d evaluate_bspline(
        double t, 
        int k, 
        const Eigen::VectorXd& knots, 
        const std::vector<Eigen::Vector3d>& control_points
    );
    std::vector<Eigen::Vector3d> bspline_optimize(const std::vector<Eigen::Vector3d>&);
};