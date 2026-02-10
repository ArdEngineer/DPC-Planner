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

class PlannerUtils{
public:
    // flags
    bool receive_new_goal;
    bool receive_map;
    bool update_tmp_goal;
    bool exec_optimize;
    char plan_state; // 1->first_time_plan, 0->replan

    std_msgs::Char interrupt_signal;

    Eigen::Vector3d start;
    Eigen::Vector3d goal;

    octomap::OcTree *Map;

    std::vector<Eigen::Vector3d> plan_traj;
    geometry_msgs::PoseArray mission_points;

    std::map<Eigen::Vector3d, std::pair<Eigen::Vector3d, Eigen::Vector3d>, Vec3Less> guidance_path;
    std::shared_ptr<sensor_msgs::PointCloud2> map;
    Eigen::Vector3d drone_pose;

    ros::Subscriber drone_pose_sub;
    ros::Subscriber map_sub;
    ros::Subscriber set_goal_sub;

    ros::Publisher planned_path_visual_pub;
    ros::Publisher guidance_path_visual_pub;
    ros::Publisher astar_path_visual_pub;

    ros::Publisher planned_path_mission_pub;
    ros::Publisher emergency_hover_cmd_pub;

    ros::Publisher goal_receive_feedback_pub;
    ros::Publisher flight_interrupt_signal_pub;

    AStarSearch* astar;
    TrajOptimizer* optimizer;
    OptimizeParams* params;

    void init_planner(OptimizeParams const&, PlannerTopic const&, ros::NodeHandle&);

    void set_goal_callback(const geometry_msgs::Pose::ConstPtr&);
    void map_callback(const sensor_msgs::PointCloud2::ConstPtr&);
    void drone_pose_callback(const geometry_msgs::Pose::ConstPtr&);
    
    void set_current_plan_start(double);
    void update_guidance_path();
    Eigen::Vector4d convert_polynominal_to_bspline(std::vector<Eigen::Vector3d> const&);
    void generate_guidance_path(int);
    bool check_collision();
    void find_collision_free_path();
    void path_optimize_and_adjust();
    
    void visualize_guidance_path();
    void visualize_astar_path();
    void visualize_planning_path();

    void publish_mission_points();
    void publish_emergency_hover_cmd();
    void publish_flight_interrupt_signal();

    bool reached_goal();
    void reset(int);
};