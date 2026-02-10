# pragma once

# include <Eigen/Core>

// 输出字体和背景颜色控制
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* 黑色 */
#define RED     "\033[31m"      /* 红色 */
#define GREEN   "\033[32m"      /* 绿色 */
#define YELLOW  "\033[33m"      /* 黄色 */
#define BLUE    "\033[34m"      /* 蓝色 */
#define MAGENTA "\033[35m"      /* 洋红 */
#define CYAN    "\033[36m"      /* 青色 */
#define WHITE   "\033[37m"      /* 白色 */
#define BOLD    "\033[1m"       /* 加粗 */

// 背景色
#define BG_BLACK        "\033[40m"
#define BG_RED          "\033[41m"
#define BG_GREEN        "\033[42m"
#define BG_YELLOW       "\033[43m"
#define BG_BLUE         "\033[44m"
#define BG_WHITE        "\033[47m"
#define BG_BRIGHT_WHITE "\033[107m"

class PublicParams{
public:
    // 存放整个规划器内部都会使用到的公共参数
    bool test_mode;

    double planner_freq;
    double reached_threshold;

    int gd_path_mode;

    double ts;
    double dec_ts;
    double max_v;
    double sample_max_ds;

    int sensor_type;
    double FOV_ANGLE;
    double FOV_DEPTH;
    double map_resolution;
    double pro_hit;
    double pro_miss;
    double clamping_thresmin;
    double clamping_thresmax;
    double map_z_min;
    double map_z_max;
    double map_r;

    double theta_v_max;
    double delta_theta;
    double plan_len;
    double average_v;
    double replan_threshold_dist;
    double spline_sampling_ds;

    int astar_search_mode;
    double astar_step_size;
    double astar_max_iter;
    double astar_reached_threshold;
    double astar_max_time;
    double astar_hfunc_factor;
    
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
};

struct Vec3Less {
    bool operator()(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
        return (a.x() != b.x()) ? (a.x() < b.x()) :
               (a.y() != b.y()) ? (a.y() < b.y()) :
               (a.z() < b.z());
    }
};