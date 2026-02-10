# include "../include/planner_utils.h"

void PlannerUtils::init_planner(OptimizeParams const& optimizer_params, PlannerTopic const& topics, ros::NodeHandle& nh){
    set_goal_sub = nh.subscribe(topics.set_goal_topic, 1, &PlannerUtils::set_goal_callback, this);
    drone_pose_sub = nh.subscribe(topics.drone_pose_topic, 1, &PlannerUtils::drone_pose_callback, this);
    map_sub = nh.subscribe(topics.map_topic, 1, &PlannerUtils::map_callback, this);

    astar_path_visual_pub = nh.advertise<visualization_msgs::Marker>(topics.astar_path_visual_topic, 1);
    guidance_path_visual_pub = nh.advertise<visualization_msgs::Marker>(topics.guidance_path_visual_topic, 1);
    planned_path_visual_pub = nh.advertise<visualization_msgs::Marker>(topics.planned_path_visual_topic, 1);

    interrupt_signal.data = 'q';

    astar = new AStarSearch();
    astar->init_searcher(0.5f);

    optimizer = new TrajOptimizer();
    optimizer->init_optimizer_params(optimizer_params);

    receive_map = false;
    receive_new_goal = false;
}

bool PlannerUtils::reached_goal(){
    double threshold = 0.1;
    return ((goal - drone_pose).norm() <= threshold);
}

void PlannerUtils::set_goal_callback(const geometry_msgs::Pose::ConstPtr& msg){
    goal(0) = msg->position.x;
    goal(1) = msg->position.y;
    goal(2) = msg->position.z;
    receive_new_goal = true;
}

void PlannerUtils::drone_pose_callback(const geometry_msgs::Pose::ConstPtr& msg){
    drone_pose(0) = msg->position.x;
    drone_pose(1) = msg->position.y;
    drone_pose(2) = msg->position.z;
}

void PlannerUtils::map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    receive_map = true;
}

void PlannerUtils::update_guidance_path(){
    if(!plan_traj.empty()){
        guidance_path.clear();

        for(const auto& p : plan_traj){
            guidance_path.emplace(p, std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
        }
    }
}

Eigen::Vector4d PlannerUtils::convert_polynominal_to_bspline(std::vector<Eigen::Vector3d> const& polynominal_pts){
    Eigen::Vector4d BSplineCoeffs = Eigen::Vector4d::Zero();

    // Calculate b-spline coeffs

    return BSplineCoeffs;
}

void PlannerUtils::generate_guidance_path(int mode){
    if(mode == 0){
        // Generate BSpline guidance path
        double dist = (start - goal).norm();
        double T = 0.0f;
        double max_v = 2.0f;

        double dist_threshold = max_v * max_v;
        T = 2 * max_v + ((dist - dist_threshold >= 1e-6) ? ((dist - dist_threshold) / max_v) : 0.0f);

        double T2 = std::pow(T, 2);
        double T3 = std::pow(T, 3);
        double T4 = std::pow(T, 4);
        double T5 = std::pow(T, 5);

        Eigen::Matrix<double, 6, 6> A;
        A << 1, 0, 0,   0,    0,     0,
            1, T, T2,  T3,   T4,    T5,
            0, 1, 0,   0,    0,     0,
            0, 1, 2*T, 3*T2, 4*T3,  5*T4,
            0, 2, 0,   0,    0,     0,
            0, 0, 2,   6*T,  12*T2, 20*T3;
        
        Eigen::VectorXd bx = (Eigen::VectorXd(6, 1) << 0, start(0), 0, 0, 0, 0).finished();
        Eigen::VectorXd by = (Eigen::VectorXd(6, 1) << 0, start(1), 0, 0, 0, 0).finished();
        Eigen::VectorXd bz = (Eigen::VectorXd(6, 1) << 0, start(2), 0, 0, 0, 0).finished();

        Eigen::VectorXd cx = A.colPivHouseholderQr().solve(bx);
        Eigen::VectorXd cy = A.colPivHouseholderQr().solve(by);
        Eigen::VectorXd cz = A.colPivHouseholderQr().solve(bz);

        double step_size = 0.3;
        double ts = (start - goal).norm() > 0.1 ? step_size / max_v * 1.2 : step_size / max_v * 5;

        ts *= 1.5f;
        double max_dist = 1.0;
        std::vector<Eigen::Vector3d> sampling_pts;
        double t = 0.0f;

        Eigen::Vector3d last_pt;
        last_pt(0) = cx(0) + cx(1)*t + cx(2)*std::pow(t,2) + cx(3)*std::pow(t,3) + cx(4)*std::pow(t,4) + cx(5)*std::pow(t,5);
        last_pt(1) = cy(0) + cy(1)*t + cy(2)*std::pow(t,2) + cy(3)*std::pow(t,3) + cy(4)*std::pow(t,4) + cy(5)*std::pow(t,5);
        last_pt(2) = cz(0) + cz(1)*t + cz(2)*std::pow(t,2) + cz(3)*std::pow(t,3) + cz(4)*std::pow(t,4) + cz(5)*std::pow(t,5);
        sampling_pts.push_back(last_pt);

        while(t <= T){
            while(1){
                t += ts;
                Eigen::Vector3d next_pt;
                next_pt(0) = cx(0) + cx(1)*t + cx(2)*std::pow(t,2) + cx(3)*std::pow(t,3) + cx(4)*std::pow(t,4) + cx(5)*std::pow(t,5);
                next_pt(1) = cy(0) + cy(1)*t + cy(2)*std::pow(t,2) + cy(3)*std::pow(t,3) + cy(4)*std::pow(t,4) + cy(5)*std::pow(t,5);
                next_pt(2) = cz(0) + cx(1)*t + cz(2)*std::pow(t,2) + cz(3)*std::pow(t,3) + cz(4)*std::pow(t,4) + cz(5)*std::pow(t,5);

                if((next_pt - last_pt).norm() > max_dist){
                    t -= ts;
                    ts /= 1.5;
                }else{
                    last_pt = next_pt;
                    sampling_pts.push_back(last_pt);
                    break;
                }
            }
        }

        Eigen::Vector4d bspline_coeffs = convert_polynominal_to_bspline(sampling_pts);

        guidance_path.clear();

        // sampling from b-spline trajectory

        ROS_INFO("[MODE 0 BSPLINE LINE] Guidance path generate finished!");
    }
    else if(mode == 1){
        // Generate straight line guidance path
        double dist = (start - goal).norm();

        double total_t, delta_t;
        double average_v;

        average_v = 1.5;
        delta_t = 0.5;
        total_t = dist / average_v;

        int segment_num = total_t / delta_t;

        Eigen::Vector3d pnext = start;
        Eigen::Vector3d u = (start - goal) / (start - goal).norm();
        guidance_path.emplace(std::make_pair(pnext, std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())));
        for(int i = 0; i < segment_num; i++){
            if(i == segment_num - 1 && (goal - pnext).norm() < average_v * delta_t) pnext = goal;
            else pnext = pnext + average_v * delta_t * u;
            guidance_path.emplace(
                std::make_pair(
                    pnext, std::make_pair(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())
                )
            );
        }

        ROS_INFO("[MODE 1 STRAIGHT LINE] Guidance path generate finished!");
    }
}

void PlannerUtils::set_current_plan_start(double forward_t){
    double delta_t = forward_t / 1000.0f;

    // Predict the start point
    // Then update guidance path
}

bool PlannerUtils::check_collision(){
    bool collision_flag = false;
    astar->segment_field.clear();
    std::pair<Eigen::Vector3d, Eigen::Vector3d> cur_seg;
    for(auto iter = guidance_path.begin(); iter != guidance_path.end(); ++iter){
        auto next_iter = std::next(iter);
        Eigen::Vector3d cur_pt = iter->first;
        Eigen::Vector3d next_pt = next_iter->first;
        if (!HitObstacles(cur_pt) && HitObstacles(next_pt)){
            cur_seg.first = cur_pt;
            collision_flag = true;
        }
        else if(HitObstacles(cur_pt) && !HitObstacles(next_pt)){
            cur_seg.second = next_pt;
            astar->segment_field.emplace_back(cur_seg);
        }
    }

    return collision_flag;
}

void PlannerUtils::find_collision_free_path(){
    astar->path_search();

    Vec3Less cmp;
    int index = 0;
    auto astar_path = astar->collision_free_paths.begin();
    for(auto seg = astar->segment_field.begin(); seg != astar->segment_field.end(); ++seg){
        index += 1;
        if (astar_path->empty()){
            ROS_WARN("Cann't not find collision free path in %d-th collision segment!", index);
        }else{
            std::vector<Eigen::Vector3d> pts;
            auto start_p = guidance_path.find(seg->first);
            auto end_p = guidance_path.find(seg->second);

            bool forward = cmp(start_p->first, end_p->first);
            auto cur_p = start_p;

            do{
                pts.emplace_back(cur_p->first);
                if(forward) ++cur_p;
                else --cur_p;
            }while(cur_p != end_p);

            for(int i = 1; i < pts.size(); i++){
                Eigen::Vector3d Ri = (pts[i + 1] - pts[i - 1]) / (pts[i + 1] - pts[i - 1]).norm();
                Eigen::Vector3d P;
                bool p_none = true;

                for(int k = 0; k < astar_path->size() - 1; k++){
                    auto ak = (*astar_path)[k];
                    auto ak1 = (*astar_path)[k+1];

                    auto seg_vec = ak1 - ak;
                    auto norm_seg = (seg_vec).norm();

                    if(norm_seg < 1e-6) continue;
                    auto numerator = Ri.dot((pts[i] - ak));
                    auto denominator = Ri.dot(seg_vec);

                    if (denominator < 1e-6 || denominator >= -1e-6) continue;

                    auto t = numerator / denominator;
                    if (t - 1.0f <= 1e-6 && t >= 1e-6){
                        P = ak + t * seg_vec;
                        p_none = false;
                        break;
                    }
                }

                if(!p_none){
                    Eigen::Vector3d vi = P - pts[i];
                    vi = vi / vi.norm();

                    auto qi = guidance_path.find(pts[i]);
                    (qi->second).first = P;
                    (qi->second).second = vi;
                }
            }

            ++astar_path;
        }
    }   
}

void PlannerUtils::path_optimize_and_adjust(){
    optimizer->problem_content = &guidance_path;
    optimizer->construct_optimize_problem();
    optimizer->solve_optimize(&plan_traj);
}

void PlannerUtils::visualize_guidance_path(){
    if(!guidance_path.empty()){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;

        for(const auto& p : guidance_path){
            geometry_msgs::Point pt;
            pt.x = p.first(0);
            pt.y = p.first(1);
            pt.z = p.first(2);
            marker.points.emplace_back(pt);
        }

        guidance_path_visual_pub.publish(marker);
    }
}

void PlannerUtils::visualize_astar_path(){
    if(!astar->collision_free_paths.empty()){
        int id = 0;
        for(const auto &traj : astar->collision_free_paths){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;

            for(const auto& p : traj){
                geometry_msgs::Point pt;
                pt.x = p(0);
                pt.y = p(1);
                pt.z = p(2);
                marker.points.emplace_back(pt);
            }

            astar_path_visual_pub.publish(marker);
        }
    }
}

void PlannerUtils::visualize_planning_path(){
    if(!plan_traj.empty()){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        for(const auto& p : plan_traj){
            geometry_msgs::Point pt;
            pt.x = p(0);
            pt.y = p(1);
            pt.z = p(2);
            marker.points.emplace_back(pt);
        }

        planned_path_visual_pub.publish(marker);
    }
}

void PlannerUtils::publish_mission_points(){
    mission_points.header.frame_id = "map";
    mission_points.header.stamp = ros::Time::now();

    mission_points.poses.clear();

    for(const auto& p : plan_traj){
        geometry_msgs::Pose set_point;
        set_point.position.x = p(0);
        set_point.position.y = p(1);
        set_point.position.z = p(2);
        mission_points.poses.emplace_back(set_point);
    }

    planned_path_mission_pub.publish(mission_points);
}

void PlannerUtils::publish_emergency_hover_cmd(){
    geometry_msgs::Twist hover;

    hover.linear.x = 0.0f;
    hover.linear.y = 0.0f;
    hover.linear.z = 0.0f;
    hover.angular.x = 0.0f;
    hover.angular.y = 0.0f;
    hover.angular.z = 0.0f;

    emergency_hover_cmd_pub.publish(hover);
}

void PlannerUtils::publish_flight_interrupt_signal(){
    flight_interrupt_signal_pub.publish(interrupt_signal);
}

void PlannerUtils::reset(int state)
{
    ROS_INFO("Reset planner in mode %d", state);

    switch (state)
    {
        case 0:
            ROS_WARN("State 0, reached goal!");
            guidance_path.clear();
            astar->collision_free_paths.clear();
            astar->segment_field.clear();
            receive_map = false;
            receive_new_goal = false;
            break;
        case 1:
            ROS_WARN("State 1, receive new goal!");
            guidance_path.clear();
            astar->collision_free_paths.clear();
            astar->segment_field.clear();
            receive_map = false;
            break;
        default:
            ROS_ERROR("Wrong state, check state set!");
            break;
    }
}