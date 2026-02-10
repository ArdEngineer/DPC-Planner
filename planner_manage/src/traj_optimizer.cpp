# include "../include/traj_optimizer.h"
# include <cmath>
# include "ros/ros.h"

void TrajOptimizer::set_optimizer_params(PublicParams const& params){
    dt = params.dt;
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
    xtol_rel = params.xtol_rel;
    maxeval = params.maxeval;
    average_v = params.average_v;
}

// 全局静态指针用于NLOpt回调
static TrajOptimizer* g_optimizer = nullptr;

// NLOpt目标函数包装器
double nlopt_objective(unsigned n, const double* x, double* grad, void* data) {
    g_optimizer = static_cast<TrajOptimizer*>(data);
    
    // 将输入x转换为Eigen格式
    Eigen::Map<const Eigen::VectorXd> flat_pts(x, n);
    std::vector<Eigen::Vector3d> control_points(n / 3);
    for (int i = 0; i < n / 3; ++i) {
        control_points[i] = flat_pts.segment<3>(i * 3);
    }
    
    // 计算目标函数值
    double f_val = g_optimizer->object_function(control_points);
    
    // 如果需要梯度
    if (grad) {
        std::vector<Eigen::Vector3d> gradient = g_optimizer->object_function_Jacobi(control_points);
        for (int i = 0; i < n; ++i) {
            grad[i] = gradient[i / 3](i % 3);
        }
    }
    
    return f_val;
}

void TrajOptimizer::solve_optimize(
    std::vector<std::pair<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>>>& opt_content,
    std::vector<Eigen::Vector3d>& opt_res
) { 
    // 存储优化内容供回调函数使用
    optimize_content_ = opt_content;
    
    // 提取有效控制点（跳过首尾）
    std::vector<Eigen::Vector3d> initial_control_points;
    for (size_t i = 0; i < opt_content.size(); ++i) {
        if (i == 0 || i == opt_content.size() - 1) continue;
        initial_control_points.push_back(opt_content[i].first);
    }
    
    if (initial_control_points.empty()) {
        ROS_ERROR("No valid control points for optimization, returning original path");
        opt_res.reserve(opt_content.size());
        for (const auto& pt : opt_content) {
            opt_res.push_back(pt.first);
        }
        return;
    }
    
    // 准备初始向量
    int n_vars = initial_control_points.size() * 3;
    Eigen::VectorXd initial_vec(n_vars);
    for (size_t i = 0; i < initial_control_points.size(); ++i) {
        initial_vec.segment<3>(i * 3) = initial_control_points[i];
    }
    
    // 创建NLOpt优化器
    nlopt::opt opt(nlopt::LD_LBFGS, n_vars);
    
    // 设置目标函数
    g_optimizer = this;
    opt.set_min_objective(nlopt_objective, this);
    
    // 设置停止条件
    opt.set_xtol_rel(xtol_rel);
    opt.set_maxeval(maxeval);
    
    // 执行优化
    std::vector<double> x(initial_vec.data(), initial_vec.data() + n_vars);
    double min_f;
    nlopt::result result = opt.optimize(x, min_f);
    
    // 构建完整控制点（包含固定的首尾）
    std::vector<Eigen::Vector3d> optimized_control_pts(opt_content.size());
    optimized_control_pts[0] = opt_content[0].first; // 首点固定
    optimized_control_pts.back() = opt_content.back().first; // 尾点固定
    
    // 填充优化后的内部点
    int idx = 1;
    for (size_t i = 0; i < initial_control_points.size(); ++i) {
        optimized_control_pts[idx++] = Eigen::Vector3d(x[i * 3], x[i * 3 + 1], x[i * 3 + 2]);
    }

    // ROS_WARN("Control points after optimize:");
    // for(auto c : optimized_control_pts){
    //     std::cout << "q = ("<< c(0) << ", " << c(1) << ", " << c(2) << ")\n"; 
    // }
    // std::cout << "\n";
    
    // 生成B样条并采样
    opt_res = bspline_optimize(optimized_control_pts);
    
    ROS_WARN("Optimization finished successfully");
}

/*------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------惩罚函数部分--------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------*/
double TrajOptimizer::penalty_function_jc(
    const std::vector<Eigen::Vector3d>& control_points, 
    const std::vector<std::pair<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>>>& opt_content
) {
    double jc = 0.0;

    for (size_t i = 0; i < opt_content.size(); ++i) {
    const auto& pt = opt_content[i];
    const Eigen::Vector3d& p = pt.second.first;
    const Eigen::Vector3d& v = pt.second.second;
    const Eigen::Vector3d& ctrl_pt = control_points[i];

    // 跳过无效点对
    if (v.norm() < 1e-6) continue;

    Eigen::Vector3d diff = p - ctrl_pt;
    double d_ij = diff.dot(v);
    double c_ij = sf - d_ij;

    if (c_ij <= 0) {
        jc += 0.0;
    } else if (0 < c_ij <= sf) {
        jc += pow(c_ij, 3);
    } else {
        jc += 3 * sf * c_ij * c_ij - 3 * sf * sf * c_ij + sf * sf * sf;
    }
    }

    return jc;
}

double TrajOptimizer::penalty_function_jd(const std::vector<Eigen::Vector3d>& control_points) {
    double fv = 0.0, fa = 0.0, fj = 0.0;
    int n = control_points.size();
    
    // 速度惩罚
    for (int i = 0; i < n - 1; ++i) {
        Eigen::Vector3d v = (control_points[i + 1] - control_points[i]) / dt;
        for (int dim = 0; dim < 3; ++dim) {
            fv += f(v(dim), cj, cmv, lamda);
        }
    }
    
    // 加速度惩罚
    for (int i = 0; i < n - 2; ++i) {
        Eigen::Vector3d a = (control_points[i + 2] - 2 * control_points[i + 1] + control_points[i]) / (dt * dt);
        for (int dim = 0; dim < 3; ++dim) {
            fa += f(a(dim), cj, cma, lamda);
        }
    }
    
    // Jerk惩罚
    for (int i = 0; i < n - 3; ++i) {
        Eigen::Vector3d j = (control_points[i + 3] - 3 * control_points[i + 2] + 3 * control_points[i + 1] - control_points[i]) / (dt * dt * dt);
        for (int dim = 0; dim < 3; ++dim) {
            fj += f(j(dim), cj, cmj, lamda);
        }
    }
    
    return omega_v * fv + omega_a * fa + omega_j * fj;
}

double TrajOptimizer::penalty_function_js(const std::vector<Eigen::Vector3d>& control_points) {
    double a_cost = 0.0;
    double j_cost = 0.0;
    int n = control_points.size();
    
    // 加速度成本
    for (int i = 0; i < n - 2; ++i) {
        Eigen::Vector3d acc = (control_points[i + 2] - 2 * control_points[i + 1] + control_points[i]) / (dt * dt);
        a_cost += acc.squaredNorm();
    }
    
    // Jerk成本
    for (int i = 0; i < n - 3; ++i) {
        Eigen::Vector3d jerk = (control_points[i + 3] - 3 * control_points[i + 2] + 3 * control_points[i + 1] - control_points[i]) / (dt * dt * dt);
        j_cost += jerk.squaredNorm();
    }
    
    return a_cost + j_cost;
}

double TrajOptimizer::f(double x, double cj, double cm, double lamda) {
    double a1 = 3 * (cj - lamda * cm);
    double b1 = 3 * (cj * cj - pow(lamda * cm, 2));
    double c1 = pow(cj, 3) - pow(lamda * cm, 3);
    double a2 = 3 * (cj - lamda * cm);
    double b2 = -3 * (cj * cj - pow(lamda * cm, 2));
    double c2 = pow(cj, 3) - pow(lamda * cm, 3);
    
    if (x <= -cj) {
        return a1 * x * x + b1 * x + c1;
    } else if (-cj < x < -lamda * cm) {
        return pow(-lamda * cm - x, 3);
    } else if (-lamda * cm <= x <= lamda * cm) {
        return 0.0;
    } else if (lamda * cm < x < cj) {
        return pow(x - lamda * cm, 3);
    } else {
        return a2 * x * x + b2 * x + c2;
    }
}

double TrajOptimizer::f_deriv(double x, double cj, double cm, double lamda) {
    double a1 = 3 * (cj - lamda * cm);
    double b1 = 3 * (cj * cj - pow(lamda * cm, 2));
    double a2 = 3 * (cj - lamda * cm);
    double b2 = -3 * (cj * cj - pow(lamda * cm, 2));
    
    if (x <= -cj) {
        return 2 * a1 * x + b1;
    } else if (-cj < x < -lamda * cm) {
        return -3 * pow(-lamda * cm - x, 2);
    } else if (-lamda * cm <= x <= lamda * cm) {
        return 0.0;
    } else if (lamda * cm < x < cj) {
        return 3 * pow(x - lamda * cm, 2);
    } else {
        return 2 * a2 * x + b2;
    }
}

/*------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------惩罚函数梯度--------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------*/
std::vector<Eigen::Vector3d> TrajOptimizer::penalty_function_Jacobi_jc(const std::vector<Eigen::Vector3d>& control_points,
    const std::vector<std::pair<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>>>& opt_content
) {
    int n = control_points.size();
    std::vector<Eigen::Vector3d> grad(n, Eigen::Vector3d::Zero());

    for (int i = 0; i < opt_content.size(); ++i) {
    const auto& pt = opt_content[i];
    const Eigen::Vector3d& p = pt.second.first;
    const Eigen::Vector3d& v = pt.second.second;
    const Eigen::Vector3d& ctrl_pt = control_points[i];

    if (v.norm() < 1e-6) continue;

    Eigen::Vector3d diff = p - ctrl_pt;
    double d_ij = diff.dot(v);
    double c_ij = sf - d_ij;

    if (c_ij <= 0) {
        continue;
    } else if (0 < c_ij <= sf) {
        grad[i] = 3 * c_ij * c_ij * v;
    } else {
        grad[i] = (6 * sf * c_ij - 3 * sf * sf) * v;
    }
    }

    return grad;
}

std::vector<Eigen::Vector3d> TrajOptimizer::penalty_function_Jacobi_jd(const std::vector<Eigen::Vector3d>& control_points) {
    int n = control_points.size();
    std::vector<Eigen::Vector3d> grad(n, Eigen::Vector3d::Zero());
    double dt_sq = dt * dt;
    double dt_cu = dt * dt * dt;
    
    // 速度梯度
    for (int i = 0; i < n - 1; ++i) {
        Eigen::Vector3d v_vec = (control_points[i + 1] - control_points[i]) / dt;
        double dv_dx = 1.0 / dt;
        
        for (int dim = 0; dim < 3; ++dim) {
            double df_dv = f_deriv(v_vec(dim), cj, cmv, lamda);
            grad[i](dim) += -omega_v * df_dv * dv_dx;
            grad[i + 1](dim) += omega_v * df_dv * dv_dx;
        }
    }
    
    // 加速度梯度
    for (int i = 0; i < n - 2; ++i) {
        Eigen::Vector3d a_vec = (control_points[i + 2] - 2 * control_points[i + 1] + control_points[i]) / dt_sq;
        double da_dx = 1.0 / dt_sq;
        
        for (int dim = 0; dim < 3; ++dim) {
            double df_da = f_deriv(a_vec(dim), cj, cma, lamda);
            grad[i](dim) += omega_a * df_da * da_dx;
            grad[i + 1](dim) += -2 * omega_a * df_da * da_dx;
            grad[i + 2](dim) += omega_a * df_da * da_dx;
        }
    }
    
    // Jerk梯度
    for (int i = 0; i < n - 3; ++i) {
        Eigen::Vector3d j_vec = (control_points[i + 3] - 3 * control_points[i + 2] + 3 * control_points[i + 1] - control_points[i]) / dt_cu;
        double dj_dx = 1.0 / dt_cu;
        
        for (int dim = 0; dim < 3; ++dim) {
            double df_dj = f_deriv(j_vec(dim), cj, cmj, lamda);
            grad[i](dim) += -omega_j * df_dj * dj_dx;
            grad[i + 1](dim) += 3 * omega_j * df_dj * dj_dx;
            grad[i + 2](dim) += -3 * omega_j * df_dj * dj_dx;
            grad[i + 3](dim) += omega_j * df_dj * dj_dx;
        }
    }
    
    return grad;
}
std::vector<Eigen::Vector3d> TrajOptimizer::penalty_function_Jacobi_js(const std::vector<Eigen::Vector3d>& control_points) {
    int n = control_points.size();
    std::vector<Eigen::Vector3d> grad(n, Eigen::Vector3d::Zero());
    double dt_sq = dt * dt;
    double dt_cu = dt * dt * dt;
    
    // 加速度梯度
    for (int i = 0; i < n - 2; ++i) {
        Eigen::Vector3d acc = (control_points[i + 2] - 2 * control_points[i + 1] + control_points[i]) / dt_sq;
        grad[i] += 2 * acc / dt_sq;
        grad[i + 1] += -4 * acc / dt_sq;
        grad[i + 2] += 2 * acc / dt_sq;
    }
    
    // Jerk梯度
    for (int i = 0; i < n - 3; ++i) {
        Eigen::Vector3d jerk = (control_points[i + 3] - 3 * control_points[i + 2] + 3 * control_points[i + 1] - control_points[i]) / dt_cu;
        grad[i] += -2 * jerk / dt_cu;
        grad[i + 1] += 6 * jerk / dt_cu;
        grad[i + 2] += -6 * jerk / dt_cu;
        grad[i + 3] += 2 * jerk / dt_cu;
    }
    
    return grad;
}

/*------------------------------------------------------------------------------------------------------------*/
/*-----------------------------------------------目标函数部分--------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------*/
double TrajOptimizer::object_function(const std::vector<Eigen::Vector3d>& control_points) {
    double js = penalty_function_js(control_points);
    double jc = penalty_function_jc(control_points, optimize_content_);
    double jd = penalty_function_jd(control_points);
    
    return lambda_s * js + lambda_c * jc + lambda_d * jd;
}

std::vector<Eigen::Vector3d> TrajOptimizer::object_function_Jacobi(const std::vector<Eigen::Vector3d>& control_points) {
    std::vector<Eigen::Vector3d> js_grad = penalty_function_Jacobi_js(control_points);
    std::vector<Eigen::Vector3d> jc_grad = penalty_function_Jacobi_jc(control_points, optimize_content_);
    std::vector<Eigen::Vector3d> jd_grad = penalty_function_Jacobi_jd(control_points);
    
    std::vector<Eigen::Vector3d> total_grad(control_points.size());
    for (size_t i = 0; i < control_points.size(); ++i) {
        total_grad[i] = lambda_s * js_grad[i] + lambda_c * jc_grad[i] + lambda_d * jd_grad[i];
    }
    
    return total_grad;
}

/*------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------样条优化器--------------------------------------------------*/
/*-----------------------------------------------------------------------------------------------------------*/
double basis_function(int i, int k, double t, const Eigen::VectorXd& knots, int n_ctrl) {
    // 0次基函数（递归终止）
    if (k == 0) {
        // return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
        if (i == n_ctrl - 1) {
            // 最后一个控制点：t ∈ [knots[i], knots[i+1]] （双闭）
            return (t >= knots[i] && t <= knots[i + 1] + 1e-9) ? 1.0 : 0.0;
        } else {
            // 普通控制点：t ∈ [knots[i], knots[i+1]) （左闭右开）
            return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
        }
    }
    
    // 递推公式计算k次基函数
    double left = 0.0, right = 0.0;
    
    // 左项
    double left_num = t - knots[i];
    double left_denom = knots[i + k] - knots[i];
    if (left_denom > 1e-6) {
        left = left_num / left_denom * basis_function(i, k - 1, t, knots, n_ctrl);
    }
    
    // 右项
    double right_num = knots[i + k + 1] - t;
    double right_denom = knots[i + k + 1] - knots[i + 1];
    if (right_denom > 1e-6) {
        right = right_num / right_denom * basis_function(i + 1, k - 1, t, knots, n_ctrl);
    }
    
    return left + right;
}

Eigen::Vector3d TrajOptimizer::evaluate_bspline(
    double t, 
    int k, 
    const Eigen::VectorXd& knots, 
    const std::vector<Eigen::Vector3d>& control_points
) {
    int n = control_points.size();

    if (t <= 0.0) return control_points.front();
    if (t >= 1.0) return control_points.back();
    
    // 找到t所在的节点区间
    // int span = k;
    // for (int i = k; i < knots.size() - k - 1; ++i) {
    //     if (t >= knots[i] && t < knots[i + 1]) {
    //         span = i;
    //         break;
    //     }
    // }
    int span = k;
    for (int i = k; i < n; ++i) {
        if (t >= knots[i] && t < knots[i + 1]) {
            span = i;
            break;
        }
    }
    
    // 计算所有基函数N_i,k(t)
    std::vector<double> basis_values(n);
    for (int i = 0; i < n; ++i) {
        basis_values[i] = basis_function(i, k, t, knots, n);
    }
    
    // 加权求和：C(t) = Σ N_i,k(t) * P_i
    Eigen::Vector3d result = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; ++i) {
        result += basis_values[i] * control_points[i];
    }
    
    return result;
}

std::vector<Eigen::Vector3d> TrajOptimizer::bspline_optimize(const std::vector<Eigen::Vector3d>& control_points) {
    int n = control_points.size();
    if (n <= 5) return control_points;
    
    // 计算轨迹长度
    double length = 0.0;
    for (int i = 1; i < n; ++i) {
        length += (control_points[i] - control_points[i - 1]).norm();
    }
    double total_time = length / average_v;
    
    // 采样点数
    int num_samples = ceil(total_time / dt) + 1;
    ROS_WARN("BSpline sampling number of points = %d", num_samples);
    if (num_samples < 2) num_samples = 2;
    
    // B样条阶数
    int k = std::min(bspline_order, n - 1);
    if (k < 1) k = 1;
    
    // 构建clamped节点向量（保留原逻辑）
    int num_knots = n + k + 1;
    Eigen::VectorXd knots(num_knots);
    for (int i = 0; i <= k; ++i) {
        knots(i) = 0.0;
    }
    for (int i = k + 1; i < n; ++i) {
        knots(i) = double(i - k) / (n - k);
    }
    for (int i = n; i < num_knots; ++i) {
        knots(i) = 1.0;
    }
    
    // 均匀采样（使用Cox-de Boor公式）
    std::vector<Eigen::Vector3d> result;
    for (int i = 0; i < num_samples; ++i) {
        double t = double(i) / (num_samples - 1);
        result.push_back(evaluate_bspline(t, k, knots, control_points));
    }
    
    return result;
}