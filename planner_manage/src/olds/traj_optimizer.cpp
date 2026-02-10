# include "../include/traj_optimizer.h"

void TrajOptimizer::init_optimizer_params(OptimizeParams const& params){
    delta_t = params.delta_t;
    sf = params.sf;
    lambda_s = params.lambda_s;
    lambda_d = params.lambda_d;
    lambda_c = params.lambda_c;
    cj = params.cj;
    cmv = params.cmv;
    cma = params.cma;
    cmj = params.cmj;
    lamda = params.lamda;
    omega_v = params.omega_v;
    omega_a = params.omega_a;
    omega_j = params.omega_j;
    num_of_control_points = params.num_of_control_points;
    bspline_order = params.bspline_order;
    knot_type = params.knot_type;
}

void TrajOptimizer::construct_optimize_problem(){

}

auto TrajOptimizer::penalty_function(){

}
auto TrajOptimizer::penalty_function_Jacobi(){

}

void TrajOptimizer::solve_optimize(std::vector<Eigen::Vector3d>*){

}

void TrajOptimizer::bspline_optimize(){
    
}