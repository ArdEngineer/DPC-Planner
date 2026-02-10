# pragma once

# include <string>
# include <ceres/solver.h>
# include <Eigen/Core>
# include <map>

# include "common_utils.h"

class OptimizeParams{
public:
    double delta_t;
    double sf;
    double lambda_s;
    double lambda_d;
    double lambda_c;
    double cj, cmv, cma, cmj;
    double lamda;
    double omega_v, omega_a, omega_j;

    int num_of_control_points;
    int bspline_order;
    std::string knot_type;
};

class TrajOptimizer{
public:
    double delta_t;
    double sf;
    double lambda_s;
    double lambda_d;
    double lambda_c;
    double cj, cmv, cma, cmj;
    double lamda;
    double omega_v, omega_a, omega_j;

    int num_of_control_points;
    int bspline_order;
    std::string knot_type;

    std::map<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>, Vec3Less>* problem_content;

    void init_optimizer_params(OptimizeParams const&);

    void construct_optimize_problem();

    void solve_optimize(std::vector<Eigen::Vector3d>*);

private:
    auto penalty_function();
    auto penalty_function_Jacobi();
    void bspline_optimize();
};