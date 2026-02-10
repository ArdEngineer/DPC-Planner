# pragma once

# include <ros/ros.h>
# include <geometry_msgs/Pose.h>
# include <geometry_msgs/PoseArray.h>
# include <geometry_msgs/Vector3.h>
# include <geometry_msgs/Twist.h>
# include <sensor_msgs/PointCloud2.h>
# include <visualization_msgs/Marker.h>
# include <std_msgs/Char.h>
# include <Eigen/Core>
# include <Eigen/QR>

# include <string>
# include <vector>
# include <cmath>

# include "dyn_a_star.h"
# include "map_utils.h"
# include "common_utils.h"
# include "traj_optimizer.h"

class Flags{
public:
    /* flags */
    bool receive_new_goal;
    bool receive_new_map;
    bool update_tmp_goal;
    bool exec_optimize;
    bool map_buf_lock;
    bool goal_empty;
    bool reached_goal;
    bool update_mission_points;
    bool tpg_hit_g; // tmp_goal第一次设置为goal，如果该标志位触发，则后续不再更新guidance_path
    char plan_state; // 1->first_time_plan, 0->replan

    std_msgs::Char interrupt_signal;
};

class PlannerTopic{
public:
    std::string drone_pose_topic;
    std::string set_goal_topic;
    std::string map_topic;

    std::string planned_path_visual_topic;
    std::string guidance_path_visual_topic;
    std::string astar_path_visual_topic;
    std::string planned_path_mission_topic;
    std::string flight_interrupt_signal_topic;
    std::string goal_receive_feedback_topic;
    std::string emergency_hover_cmd_topic;
};

class Planner{
public:
    // 规划器地图容器
    MapUtils* map_utils;

    // 规划器动作信号
    Flags fs;

    // 规划器参数容器
    PublicParams pp;

    // 规划中使用的位置、速度、加速度约束信息
    Eigen::Vector3d start_p;
    Eigen::Vector3d goal;
    Eigen::Vector3d cur_pos;
    Eigen::Vector3d tmp_goal;

    Eigen::Vector3d start_v;
    Eigen::Vector3d start_a;

    // 规划器辅助插件，包括无碰撞路径搜索器，优化器，以及参数容器
    AStarSearch* astar;
    TrajOptimizer* optimizer;

    // 规划器规划信息存放容器
    std::vector<Eigen::Vector3d> guidance_path;
    std::vector<Eigen::Vector3d> control_points;
    std::vector<geometry_msgs::Point> bspline_points;

    std::vector<std::pair<size_t, size_t>> collision_segs;
    std::vector<std::vector<Eigen::Vector3d>> collision_free_segs;

    // 优化信息存储容器
    std::vector<std::pair<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> optimize_content;

    // ROS话题订阅器
    ros::Subscriber drone_pose_sub;
    ros::Subscriber map_sub;
    ros::Subscriber set_goal_sub;

    // ROS话题发布器
    ros::Publisher planned_path_visual_pub;
    ros::Publisher guidance_path_visual_pub;
    ros::Publisher astar_path_visual_pub;

    ros::Publisher planned_path_mission_pub;
    ros::Publisher emergency_hover_cmd_pub;

    ros::Publisher flight_interrupt_signal_pub;

    // ROS消息回调函数
    void set_goal_callback(const geometry_msgs::PoseStamped::ConstPtr&);
    void map_callback(const sensor_msgs::PointCloud2::ConstPtr&);
    void drone_pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);

    // 规划器信息ROS发布函数
    void publish_mission_points();
    void publish_emergency_hover_cmd();
    void publish_flight_interrupt_signal();

    // 规划器主要功能函数
    void init_planner(PublicParams const&, PlannerTopic const&, ros::NodeHandle&);

    void set_replan_start(double);

    void find_safty_tmp_goal(Eigen::Vector3d&);
    void generate_guidance_path(int);
    void check_collision();
    void find_collision_free_path();

    void construct_optimize_content();
    void path_optimize();

    void update_guidance_path(int);

    // 规划器RViz可视化组件
    void visualize_guidance_path();
    void visualize_searched_path();
    void visualize_planning_path();

    // 规划器状态检查与重置组件
    void stop_current_mission();
    bool check_mission_state();
    void reset(int);
};