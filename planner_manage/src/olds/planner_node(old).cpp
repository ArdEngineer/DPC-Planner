# include "ros/ros.h"
# include "../include/traj_optimizer.h"
# include "../include/planner_utils.h"

# include <iostream>
# include <tf/transform_listener.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");

    double planner_freq;

    ros::param::get("planner_freq", planner_freq);
    ros::Rate rate(planner_freq);

    PlannerTopic topics;
    OptimizeParams params;

    ros::param::get("drone_pose_topic", topics.drone_pose_topic);
    ros::param::get("set_goal_topic", topics.set_goal_topic);
    ros::param::get("map_topic", topics.map_topic);

    ros::param::get("planned_path_visual_topic", topics.planned_path_visual_topic);
    ros::param::get("guidance_path_visual_topic", topics.guidance_path_visual_topic);
    ros::param::get("astar_path_visual_topic", topics.astar_path_visual_topic);
    ros::param::get("planned_path_mission_topic", topics.planned_path_mission_topic);
    ros::param::get("flight_interrupt_signal_topic", topics.flight_interrupt_signal_topic);
    ros::param::get("goal_receive_feedback_topic", topics.goal_receive_feedback_topic);

    ros::param::get("delta_t", params.delta_t);
    ros::param::get("sf", params.sf);
    ros::param::get("lambda_s", params.lambda_s);
    ros::param::get("lambda_d", params.lambda_d);
    ros::param::get("lambda_c", params.lambda_c);
    ros::param::get("cj", params.cj);
    ros::param::get("cmv", params.cmv);
    ros::param::get("cma", params.cma);
    ros::param::get("cmj", params.cmj);
    ros::param::get("lamda", params.lamda);
    ros::param::get("omega_v", params.omega_v);
    ros::param::get("omega_a", params.omega_a);
    ros::param::get("omega_j", params.omega_j);
    ros::param::get("num_of_control_points", params.num_of_control_points);
    ros::param::get("bspline_order", params.bspline_order);
    ros::param::get("knot_type", params.knot_type);
    
    tf::TransformListener *tf_listener = new tf::TransformListener;
    PlannerUtils *planner_utils = new PlannerUtils();

    ROS_INFO("Initialize planner and optimizer.");

    planner_utils->init_planner(params, topics, nh);

    ROS_WARN("Planner and Optimizer initialize completed!");

    double waiting_map_duration;
    double forward_predict_time;

    while(!ros::isShuttingDown()){
        planner_utils->visualize_guidance_path();
        planner_utils->visualize_astar_path();
        planner_utils->visualize_planning_path();

        if(planner_utils->receive_new_goal){
            ROS_WARN("Planner receive new goal, interrupt current mission and reset.");
            planner_utils->publish_flight_interrupt_signal();
            planner_utils->publish_emergency_hover_cmd();
            planner_utils->reset(1);
            planner_utils->generate_guidance_path(1);
        }else{
            if(planner_utils->reached_goal()){
                planner_utils->reset(0);
                continue;
            }
            auto t0 = std::chrono::high_resolution_clock::now();
            while(!planner_utils->receive_map){
                ROS_INFO("Waiting for new map...");
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            waiting_map_duration = std::chrono::duration<double, std::milli>(t1 - t0).count(); // ms
            forward_predict_time = 50.0f + waiting_map_duration; //ms

            planner_utils->set_current_plan_start(forward_predict_time);

            if(planner_utils->check_collision()){
                ROS_WARN("Find collision segment, start optimize.");
                planner_utils->find_collision_free_path();
                planner_utils->path_optimize_and_adjust();
                planner_utils->update_guidance_path();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}