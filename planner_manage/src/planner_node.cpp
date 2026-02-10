# include "ros/ros.h"
# include "../include/planner_utils.h"

# include <iostream>
# include <tf/transform_listener.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");

    // 规划器话题、公共参数读取部分
    PlannerTopic topics;
    PublicParams params;

    // 基本规划参数
    ros::param::get("test_mode", params.test_mode);
    ros::param::get("planner_freq", params.planner_freq);
    ros::param::get("plan_len", params.plan_len);
    ros::param::get("reached_threshold", params.reached_threshold);
    ros::param::get("spline_sampling_ds", params.spline_sampling_ds);

    // 传感器与地图参数
    ros::param::get("sensor_type", params.sensor_type);
    ros::param::get("FOV_ANGLE", params.FOV_ANGLE);
    ros::param::get("FOV_DEPTH", params.FOV_DEPTH);
    ros::param::get("map_resolution", params.map_resolution);
    ros::param::get("pro_hit", params.pro_hit);
    ros::param::get("pro_miss", params.pro_miss);
    ros::param::get("clamping_thresmin", params.clamping_thresmin);
    ros::param::get("clamping_thresmax", params.clamping_thresmax);
    ros::param::get("theta_v_max", params.theta_v_max);
    ros::param::get("delta_theta", params.delta_theta);
    ros::param::get("map_z_max", params.map_z_max);
    ros::param::get("map_z_min", params.map_z_min);
    ros::param::get("map_r", params.map_r);

    // 运动参数
    ros::param::get("average_v", params.average_v);
    ros::param::get("replan_threshold_dist", params.replan_threshold_dist);

    // 订阅的话题
    ros::param::get("drone_pose_topic", topics.drone_pose_topic);
    ros::param::get("set_goal_topic", topics.set_goal_topic);
    ros::param::get("map_topic", topics.map_topic);

    // 发布的话题
    ros::param::get("planned_path_visual_topic", topics.planned_path_visual_topic);
    ros::param::get("guidance_path_visual_topic", topics.guidance_path_visual_topic);
    ros::param::get("astar_path_visual_topic", topics.astar_path_visual_topic);
    ros::param::get("planned_path_mission_topic", topics.planned_path_mission_topic);
    ros::param::get("flight_interrupt_signal_topic", topics.flight_interrupt_signal_topic);
    ros::param::get("goal_receive_feedback_topic", topics.goal_receive_feedback_topic);
    ros::param::get("emergency_hover_cmd_topic", topics.emergency_hover_cmd_topic);  // 新增

    // A*搜索参数
    ros::param::get("astar_step_size", params.astar_step_size);
    ros::param::get("astar_max_iter", params.astar_max_iter);
    ros::param::get("astar_reached_threshold", params.astar_reached_threshold);
    ros::param::get("astar_max_time", params.astar_max_time);
    ros::param::get("astar_search_mode", params.astar_search_mode);
    ros::param::get("astar_hfunc_factor", params.astar_hfunc_factor);

    // 引导轨迹生成控制参数
    ros::param::get("ts", params.ts);
    ros::param::get("dec_ts", params.dec_ts);
    ros::param::get("max_v", params.max_v);
    ros::param::get("sample_max_ds", params.sample_max_ds);

    // 优化器参数
    ros::param::get("dt", params.dt);
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

    // B样条参数
    ros::param::get("num_of_control_points", params.num_of_control_points);
    ros::param::get("bspline_order", params.bspline_order);

    // 优化控制参数
    ros::param::get("opt_xtol_rel", params.xtol_rel);
    ros::param::get("opt_maxeval", params.maxeval);

    // 其他参数
    ros::param::get("gd_path_mode", params.gd_path_mode);
    
    // 坐标变换监听器
    tf::TransformListener *tf_listener = new tf::TransformListener;

    // 初始化规划器
    Planner *planner = new Planner();
    planner->init_planner(params, topics, nh);
    ros::Rate rate(params.planner_freq);

    ROS_INFO("%s%s%sPlanner initialize completed!%s", BOLD, BG_WHITE, BLACK, RESET);

    // 日志信息输出速度控制
    double waiting_map_log_duration = 500.0f; // ms
    double waiting_goal_log_duration = 1000.0f; // ms
    double forward_predict_time = 100.0f; // ms
    double mapinfo_log_t = ros::Time::now().toSec() * 1000.0f; // ms
    double goalinfo_log_t = ros::Time::now().toSec() * 1000.0f; // ms

    // 规划器工作逻辑部分
    int plan_epoch = 0;
    while(!ros::isShuttingDown()){
        if(planner->fs.receive_new_goal){
            // 重置规划器状态
            plan_epoch = 0;
            ROS_WARN("Planner receive new goal, interrupt current mission and reset.");

            planner->stop_current_mission();
            planner->fs.plan_state = 1;
            planner->reset(1);

            planner->fs.receive_new_goal = false;
            planner->fs.goal_empty = false;

            ROS_WARN("Receive goal = (%.3f, %.3f, %.3f)", planner->goal(0), planner->goal(1), planner->goal(2));
        }

        if(!planner->fs.goal_empty){
            // planner->update_guidance_path(1);

            /* Rviz Visualization funs */
            planner->visualize_guidance_path();
            // planner->visualize_searched_path();
            planner->visualize_planning_path();

            if(!planner->fs.reached_goal){
                if(planner->fs.receive_new_map){
                    std::cout << "\n";
                    ROS_INFO("%s%s====================== New Plan Epoch ======================%s", BOLD, CYAN, RESET);
                    planner->fs.map_buf_lock = true;

                    if(planner->fs.plan_state == 1){
                        // 首次规划状态
                        planner->start_p = planner->cur_pos; // 设置规划起点

                        planner->generate_guidance_path(planner->pp.gd_path_mode); // 0->Polynominal, 1->Straight

                        ROS_INFO("%s%s%splan_state = FIRST_PLAN : plan_epoch = %d%s", BOLD, BG_YELLOW, WHITE, ++plan_epoch, RESET);

                    }else{
                        // 重规划状态
                        planner->set_replan_start(forward_predict_time);

                        planner->fs.update_tmp_goal = planner->check_mission_state();
                        
                        if(!planner->pp.test_mode){
                            if(planner->fs.update_tmp_goal && !planner->fs.tpg_hit_g){
                                ROS_INFO("%s%s%sREPLAN TRIGGER ON! Find the new tmp_goal%s", BOLD, BG_BLUE, WHITE, RESET);
                                planner->generate_guidance_path(planner->pp.gd_path_mode);
                            }
                        }
                        
                        // For debug
                        // if(planner->fs.tpg_hit_g){
                        //     ROS_ERROR(
                        //         "TMP_GAOL NOW = (%.3f, %.3f, %.3f), GOAL = (%.3f, %.3f, %.3f)",
                        //         planner->tmp_goal(0), planner->tmp_goal(1), planner->tmp_goal(2),
                        //         planner->goal(0), planner->goal(1), planner->goal(2)
                        //     );
                        // }

                        ROS_INFO("%s%s%splan_state = REPLAN : plan_epoch = %d%s", BOLD, BG_YELLOW, WHITE, ++plan_epoch, RESET);
                    }

                    planner->check_collision();

                    /* Test log */
                    if(!planner->collision_segs.empty()) 
                        ROS_WARN("%s%s%sFind %ld Collision segment!%s", BOLD, BG_RED, WHITE, planner->collision_segs.size(), RESET);
                    else ROS_WARN("No collision found!");
                    /* End test log */

                    if(planner->collision_segs.empty()) {
                        planner->fs.exec_optimize = false;
                    }
                    else planner->fs.exec_optimize = true;
                    
                    // 更新了tmp_goal之后也需要进行一次新的优化，否则可能发生updance_guidance_path()
                    // 函数中通过对旧的B样条轨迹进行采样更新引导路径导致覆盖了新的引导路径
                    // 在现有逻辑下，因为B样条轨迹只会在发现有碰撞的时候才会进行更新
                    // 如果新加入的引导路径没有碰撞，那么B样条还是原来那条短的，对其采样会导致新的引导路径被覆盖发生错误
                    if(planner->fs.exec_optimize == 1 || planner->fs.plan_state == 1 || planner->fs.update_tmp_goal){
                        planner->find_collision_free_path();
                        planner->construct_optimize_content();
                        planner->path_optimize();

                        planner->fs.update_mission_points = true;
                    }

                    ROS_INFO("%s%s%sPlanner handle finished!%s", BOLD, BG_GREEN, WHITE, RESET);

                    if(planner->fs.update_mission_points) planner->publish_mission_points();

                    ROS_INFO("Publsih mission points finished!");

                    planner->update_guidance_path(0);

                    ROS_WARN("Update guidance point finished!");

                    planner->fs.map_buf_lock = false;
                    planner->fs.update_mission_points = false;
                    planner->fs.plan_state = 0;
                    planner->fs.update_tmp_goal = false;

                }else{
                    double t = ros::Time::now().toSec() * 1000.0f;
                    if(t - mapinfo_log_t >= waiting_map_log_duration){
                        ROS_WARN("Waiting for map...");
                        mapinfo_log_t = t;
                    }
                    planner->fs.map_buf_lock = false;
                }
            }else{
                ROS_WARN("Reached goal");

                planner->publish_flight_interrupt_signal();
                planner->publish_emergency_hover_cmd();
                planner->reset(0);
                planner->fs.goal_empty = true;
            }

        }else{
            double t = ros::Time::now().toSec() * 1000.0f;
            if(t - goalinfo_log_t >= waiting_goal_log_duration){
                ROS_WARN("Waiting for goal...");
                goalinfo_log_t = t;
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}