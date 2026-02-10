# include "../include/planner_utils.h"

void Planner::init_planner(PublicParams const& pparams, PlannerTopic const& topics, ros::NodeHandle& nh){
    set_goal_sub = nh.subscribe(topics.set_goal_topic, 1, &Planner::set_goal_callback, this);
    drone_pose_sub = nh.subscribe(topics.drone_pose_topic, 1, &Planner::drone_pose_callback, this);
    map_sub = nh.subscribe(topics.map_topic, 1, &Planner::map_callback, this);

    astar_path_visual_pub = nh.advertise<visualization_msgs::Marker>(topics.astar_path_visual_topic, 1);
    guidance_path_visual_pub = nh.advertise<visualization_msgs::Marker>(topics.guidance_path_visual_topic, 1);
    planned_path_visual_pub = nh.advertise<visualization_msgs::Marker>(topics.planned_path_visual_topic, 1);
    emergency_hover_cmd_pub = nh.advertise<geometry_msgs::Twist>(topics.emergency_hover_cmd_topic, 1);
    flight_interrupt_signal_pub = nh.advertise<std_msgs::Char>(topics.flight_interrupt_signal_topic, 1);
    planned_path_mission_pub = nh.advertise<geometry_msgs::PoseArray>(topics.planned_path_mission_topic, 1);

    // 初始化公共参数
    pp.test_mode = pparams.test_mode;
    pp.planner_freq = pparams.planner_freq;
    pp.reached_threshold = pparams.reached_threshold;

    pp.gd_path_mode = pparams.gd_path_mode;

    pp.sensor_type = pparams.sensor_type;
    pp.FOV_ANGLE = pparams.FOV_ANGLE;
    pp.FOV_DEPTH = pparams.FOV_DEPTH;
    pp.map_resolution = pparams.map_resolution;
    pp.pro_hit = pparams.pro_hit;
    pp.pro_miss = pparams.pro_miss;
    pp.clamping_thresmin = pparams.clamping_thresmin;
    pp.clamping_thresmax = pparams.clamping_thresmax;
    pp.map_z_min = pparams.map_z_min;
    pp.map_z_max = pparams.map_z_max;
    pp.map_r = pparams.map_r;

    pp.theta_v_max = pparams.theta_v_max;
    pp.delta_theta = pparams.delta_theta;
    pp.plan_len = pparams.plan_len;
    pp.average_v = pparams.average_v;
    pp.replan_threshold_dist = pparams.replan_threshold_dist;
    pp.spline_sampling_ds = pparams.spline_sampling_ds;

    pp.astar_step_size = pparams.astar_step_size;
    pp.astar_max_iter = pparams.astar_max_iter;
    pp.astar_reached_threshold = pparams.astar_reached_threshold;
    pp.astar_search_mode = pparams.astar_search_mode;
    pp.astar_max_time = pparams.astar_max_time;
    pp.astar_hfunc_factor = pparams.astar_hfunc_factor;

    pp.ts = pparams.ts;
    pp.dec_ts = pparams.dec_ts;
    pp.max_v = pparams.max_v;
    pp.sample_max_ds = pparams.sample_max_ds;

    pp.dt = pparams.dt;
    pp.sf = pparams.sf;
    pp.lambda_s = pparams.lambda_s;
    pp.lambda_d = pparams.lambda_d;
    pp.lambda_c = pparams.lambda_c;
    pp.cj = pparams.cj;
    pp.cmv = pparams.cmv;
    pp.cma = pparams.cma;
    pp.cmj = pparams.cmj;
    pp.lamda = pparams.lamda;
    pp.omega_v = pparams.omega_v;
    pp.omega_a = pparams.omega_a;
    pp.omega_j = pparams.omega_j;

    pp.xtol_rel = pparams.xtol_rel;
    pp.maxeval = pparams.maxeval;

    pp.num_of_control_points = pparams.num_of_control_points;
    pp.bspline_order = pparams.bspline_order;

    // 设置规划器动作初始标志位
    fs.receive_new_map = false;
    fs.receive_new_goal = false;
    fs.exec_optimize = true;
    fs.goal_empty = true;
    fs.plan_state = 1;
    fs.reached_goal = false;
    fs.update_tmp_goal = false;
    fs.update_mission_points = true;
    fs.interrupt_signal.data = 'q';
    fs.tpg_hit_g = false;
    
    // 初始化地图容器
    map_utils = new MapUtils();
    map_utils->InitMapUtils(
        pp.map_z_min,
        pp.map_z_max,
        pp.map_r,
        pp.map_resolution, 
        pp.pro_hit, 
        pp.pro_miss, 
        pp.clamping_thresmin, 
        pp.clamping_thresmax, 
        pp.FOV_ANGLE, 
        pp.FOV_DEPTH,
        nh
    );

    // 初始化优化器
    optimizer = new TrajOptimizer();
    optimizer->set_optimizer_params(pp);

    // 初始化前端搜索器
    astar = new AStarSearch();
    astar->init_searcher(
        pp.astar_search_mode, 
        pp.astar_step_size, 
        pp.astar_max_iter, 
        pp.astar_max_time, 
        pp.astar_reached_threshold, 
        pp.astar_hfunc_factor
    );
}

/*-------------------------------------------------------------------------------*/
/*-------------------------ROS话题订阅处理函数-------------------------------------*/
/*-------------------------------------------------------------------------------*/
void Planner::set_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal(0) = msg->pose.position.x;
    goal(1) = msg->pose.position.y;
    goal(2) = cur_pos(2);
    fs.receive_new_goal = true;
}

void Planner::drone_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    cur_pos(0) = msg->pose.position.x;
    cur_pos(1) = msg->pose.position.y;
    cur_pos(2) = msg->pose.position.z;

    if((cur_pos - goal).norm() <= pp.reached_threshold) fs.reached_goal = true;
}

void Planner::map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if(fs.map_buf_lock == false){
        // 局部栅格地图载入
        bool sgn = map_utils->InsertNewMap(msg);
        if(sgn) {
            ROS_INFO("Successfully receive new map!");
            fs.receive_new_map = true;
        }
        else {
            ROS_ERROR("Insert new map error, please check MapUtils::InsertNewMap()!");
            fs.receive_new_map = false;
        }
    }else{
        fs.receive_new_map = false;
    }
}

/*-------------------------------------------------------------------------------*/
/*------------------------------------END----------------------------------------*/
/*-------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------*/
/*------------------------------任务状态检查与重置---------------------------------*/
/*-------------------------------------------------------------------------------*/
bool Planner::check_mission_state(){
    if(!pp.test_mode){
        if(!fs.goal_empty){
            ROS_INFO("MISSION GOING, CHECKED MISSION AND REPORT SET_TMP_GOAL SIGNAL.");

            if(fs.tpg_hit_g){
                return false;
            }

            if((tmp_goal - cur_pos).norm() <= pp.replan_threshold_dist) return true;
            else return false;
        }
        else{
            ROS_ERROR("NO MISSION, CHECK FINISHED.");
            return false;
        }
    }
    return true;
}

void Planner::stop_current_mission(){
    publish_flight_interrupt_signal();
    publish_emergency_hover_cmd();
}

void Planner::reset(int state)
{
    ROS_INFO("Reset planner in mode %d", state);

    switch (state)
    {
        case 0:
            ROS_WARN("State 0, reached goal!");
            guidance_path.clear();
            control_points.clear();
            bspline_points.clear();
            optimize_content.clear();
            collision_segs.clear();
            collision_free_segs.clear();

            // 规划器动作标志位重置
            fs.receive_new_map = false;
            fs.receive_new_goal = false;
            fs.exec_optimize = true;
            fs.goal_empty = true;
            fs.plan_state = 1;
            fs.reached_goal = false;
            fs.update_tmp_goal = false;
            fs.update_mission_points = true;
            fs.interrupt_signal.data = 'q';
            fs.tpg_hit_g = false;
            break;
        case 1:
            ROS_WARN("State 1, receive new goal!");
            // guidance_path.clear();
            guidance_path.clear();
            control_points.clear();
            bspline_points.clear();
            optimize_content.clear();
            collision_segs.clear();
            collision_free_segs.clear();

            // 规划器动作标志位重置
            fs.receive_new_map = false;
            fs.receive_new_goal = false;
            fs.exec_optimize = true;
            fs.goal_empty = true;
            fs.plan_state = 1;
            fs.reached_goal = false;
            fs.update_tmp_goal = false;
            fs.update_mission_points = true;
            fs.interrupt_signal.data = 'q';
            fs.tpg_hit_g = false;
            break;
        default:
            ROS_ERROR("Wrong state, check state set!");
            break;
    }
}
/*-------------------------------------------------------------------------------*/
/*------------------------------------END----------------------------------------*/
/*-------------------------------------------------------------------------------*/

void Planner::update_guidance_path(int mode){
    if(mode == 0){
        // 用于在每次优化完成后用B样条轨迹取点更新guidance_path
        size_t nc = guidance_path.size();
        size_t nb = bspline_points.size();

        if(nb == 0){
            ROS_ERROR("Bspline points empty, skip guidance path update");
            return;
        }

        guidance_path.clear();

        if(nb <= nc){
            // // 计算控制轨迹长度
            // double Lb = 0.0;
            // for(size_t i = 0; i < bspline_points.size() - 1; i++){
            //     Lb += sqrt(
            //       (bspline_points[i + 1].x - bspline_points[i].x) * (bspline_points[i + 1].x - bspline_points[i].x) +
            //       (bspline_points[i + 1].y - bspline_points[i].y) * (bspline_points[i + 1].y - bspline_points[i].y) +
            //       (bspline_points[i + 1].z - bspline_points[i].z) * (bspline_points[i + 1].z - bspline_points[i].z)
            //     );
            // }

            // // ROS_INFO("planner_utils : line 251 nc = %ld change to double = %.3f", nc, (double)nc);
            // double ds = Lb / (double)nc;

            size_t i = 0;
            guidance_path.push_back(
                Eigen::Vector3d(
                    bspline_points[i].x,
                    bspline_points[i].y,
                    bspline_points[i].z
                )
            );
            while(i < nb - 1){
                Eigen::Vector3d f_bs, b_bs;
                f_bs << Eigen::Vector3d(bspline_points[i+1].x, bspline_points[i+1].y, bspline_points[i+1].z);
                b_bs << Eigen::Vector3d(bspline_points[i].x, bspline_points[i].y, bspline_points[i].z);
                double seg_len = (f_bs - b_bs).norm();
                int n_seg_bs = floor(seg_len / pp.sample_max_ds) + 1;

                int n_insert_p = n_seg_bs - 1;

                for(size_t t = 0; t < n_insert_p; t++){
                    Eigen::Vector3d insert_p;
                    insert_p = b_bs + ((t + 1) * seg_len / n_seg_bs) * (f_bs - b_bs).normalized();
                    guidance_path.push_back(insert_p);
                }
                guidance_path.push_back(f_bs);

                i += 1;
            }

        }else{
            for(const auto& p : bspline_points){
                guidance_path.emplace_back(p.x, p.y, p.z);
            }
        }
        tmp_goal = guidance_path.back();

    }
    else if(mode == 1){
        // 在飞行过程中擦除已经过的控制点位
        if(guidance_path.empty()) return;
        size_t clost_id = -1;
        double dist = 999.0f;
        Eigen::Vector3d pos_now = cur_pos; // 多线程安全保护
        for(size_t i = 0; i < guidance_path.size(); i++){
            double d = (guidance_path[i] - pos_now).norm();
            if(d <= dist){
                dist = d;
                clost_id = i;
            }
        }

        size_t del_id = -1;
        if(clost_id != guidance_path.size() - 1){
            Eigen::Vector3d u = guidance_path[clost_id + 1] - guidance_path[clost_id];
            Eigen::Vector3d v = guidance_path[clost_id] - pos_now;
            if(u.dot(v) < 0) del_id = clost_id;
            else {
                if(clost_id != 0) del_id = clost_id - 1;
                else del_id = 0;
            }
        }

        for(size_t j = 0; j <= del_id; j++){
            guidance_path.erase(guidance_path.begin());
        }

        auto it_start = guidance_path.begin();
        guidance_path.insert(it_start, pos_now);
    }
}

void Planner::find_safty_tmp_goal(Eigen::Vector3d& gsp){
    Eigen::Vector3d d = (tmp_goal - gsp);
    
    double theta = 0.0;
    while(1){
        theta += pp.delta_theta;
        if(map_utils->HitObstacles(tmp_goal)){
            // std::cout << "No safe, keep searching...\n";
            d(0) -= pp.plan_len * (1.0 - cos((theta) * M_PI / 180.0));
            d(1) += pp.plan_len * sin((theta) * M_PI / 180.0);
            tmp_goal = d + gsp;
        }else{
            // std::cout << "Safe!\n";
            // tmp_goal = d + gsp;
            break;
        }
    }

    ROS_WARN("Get safty tmp_goal = (%.3f, %.3f, %.3f)", tmp_goal(0), tmp_goal(1), tmp_goal(2));

    return;
}

void Planner::generate_guidance_path(int mode){
    Eigen::Vector3d guidance_start_pt;
    if(fs.plan_state == 1){
        // Fist time plan
        guidance_start_pt = start_p;
        ROS_INFO("FIRSTPLAN gsp = (%.3f, %.3f, %.3f)", guidance_start_pt(0), guidance_start_pt(1), guidance_start_pt(2));
    }
    if(pp.test_mode) tmp_goal = goal; // For Test
    else{
        if(fs.plan_state == 0){
            // replan
            if(guidance_path.empty()){
                guidance_start_pt = cur_pos;
                ROS_INFO("REPLAN from current pos (passed tmp_goal): (%.3f, %.3f, %.3f)", cur_pos(0), cur_pos(1), cur_pos(2));
            }else{
                // guidance_start_pt = guidance_path.back();
                guidance_start_pt = tmp_goal;
                ROS_INFO("REPLAN [NEW] from tmp_goal = (%.3f, %.3f, %.3f)", guidance_start_pt(0), guidance_start_pt(1), guidance_start_pt(2));
            }
        }
    
        double dist = (goal - guidance_start_pt).norm();
        // (dist >= pp.plan_len) ? tmp_goal = guidance_start_pt + pp.plan_len * (goal - guidance_start_pt).normalized() : tmp_goal = goal;
        if(dist > pp.plan_len){
            tmp_goal = guidance_start_pt + pp.plan_len * (goal - guidance_start_pt).normalized();
            fs.tpg_hit_g = false;
        }else{
            ROS_INFO("%s%s%sTMP_GOAL FIRST TIME GET TO GOAL%s", BOLD, BG_GREEN, WHITE, RESET);
            tmp_goal = goal;
            fs.tpg_hit_g = true;
        }
    
        ROS_WARN("Get tmp_goal : (%.3f, %.3f, %.3f)", tmp_goal(0), tmp_goal(1), tmp_goal(2));
    
        find_safty_tmp_goal(guidance_start_pt);
    }

    if(mode == 0){
        // Generate BSpline guidance path
        // double dist = (start_p - tmp_goal).norm();
        double dist = (guidance_start_pt - tmp_goal).norm();
        double T = 0.0f;
        double max_v = pp.max_v;

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
            0, 0, 2,   0,    0,     0,
            0, 0, 2,   6*T,  12*T2, 20*T3;
        
        Eigen::VectorXd bx = (Eigen::VectorXd(6, 1) << guidance_start_pt(0), tmp_goal(0), 0, 0, 0, 0).finished();
        Eigen::VectorXd by = (Eigen::VectorXd(6, 1) << guidance_start_pt(1), tmp_goal(1), 0, 0, 0, 0).finished();
        Eigen::VectorXd bz = (Eigen::VectorXd(6, 1) << guidance_start_pt(2), tmp_goal(2), 0, 0, 0, 0).finished();

        Eigen::VectorXd cx = A.colPivHouseholderQr().solve(bx);
        Eigen::VectorXd cy = A.colPivHouseholderQr().solve(by);
        Eigen::VectorXd cz = A.colPivHouseholderQr().solve(bz);

        double step_size = 0.3;
        // double ts = (start_p - goal).norm() > 0.1 ? step_size / max_v * 1.2 : step_size / max_v * 5;
        double ts = (guidance_start_pt - tmp_goal).norm() > 0.1 ? step_size / max_v * 1.2 : step_size / max_v * 5;

        ts *= pp.ts;
        double max_dist = pp.sample_max_ds;
        std::vector<Eigen::Vector3d> sampling_pts;
        double t = 0.0f;

        Eigen::Vector3d last_pt = guidance_start_pt;
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
                    ts /= pp.dec_ts;
                }else{
                    last_pt = next_pt;
                    sampling_pts.push_back(last_pt);
                    break;
                }
            }
        }

        bool guidance_was_empty = (fs.plan_state == 0 && guidance_path.empty());

        if(fs.plan_state == 1) guidance_path.clear();
        for(size_t i = 0; i < sampling_pts.size(); i++){
            if(fs.plan_state == 0 && i == 0 && !guidance_was_empty) continue;
            guidance_path.push_back(sampling_pts[i]);
        }

        // 更新tmp_goal
        if(!guidance_path.empty()){
            tmp_goal = guidance_path.back();
        }

        ROS_INFO("[MODE 0 POLYNOMINAL LINE] Guidance path generate finished!");
    }
    else if(mode == 1){
        // Generate straight line guidance path
        double dist = (guidance_start_pt - tmp_goal).norm();

        // ROS_WARN("dist = %.3f m", dist);

        double T = dist / pp.average_v;

        // ROS_WARN("T = %.3f", T);

        int segment_num = T / pp.dt;

        ROS_WARN("Guidance points = %d", segment_num);

        Eigen::Vector3d pnext = guidance_start_pt;
        Eigen::Vector3d u = (tmp_goal - guidance_start_pt).normalized();

        if(fs.plan_state == 1) guidance_path.push_back(guidance_start_pt);
        for(int i = 0; i < segment_num; i++){
            if(i == segment_num - 1 && (tmp_goal - pnext).norm() < pp.average_v * pp.dt) pnext = tmp_goal;
            else pnext = pnext + pp.average_v * pp.dt * u;
            guidance_path.push_back(pnext);
        }

        ROS_INFO("[MODE 1 STRAIGHT LINE] Guidance path generate finished!");
    }

    // ROS_INFO("planner_utils : line 412");
    visualize_guidance_path(); // For test
}

void Planner::set_replan_start(double forward_t){
    // double delta_t = forward_t / 1000.0f;

    // Predict the start point
    if(guidance_path.size() < 2){
        ROS_WARN("Guidance path too short in set_replan_start");
        start_p = cur_pos;
        guidance_path.clear();
        guidance_path.push_back(start_p);
        return;
    }

    double dist = 999.0f;
    size_t f_id = 0;
    for(size_t i = 0; i < guidance_path.size(); i++){
        // Eigen::Vector3d q = guidance_path[i];
        double d = (guidance_path[i] - cur_pos).norm();
        if(d <= dist) {
            dist = d;
            f_id = i;
        }
    }

    // if(f_id == guidance_path.size() - 1 || guidance_path.size() < 2){
    //     return;
    if(f_id == guidance_path.size() - 1){
        ROS_INFO("Reached end of current guidance path segment");
        start_p = guidance_path.back();
        // 保留最后一个点作为参考，不要全部清空
        Eigen::Vector3d last_pt = guidance_path.back();
        guidance_path.clear();
        guidance_path.push_back(last_pt);
        return;
    }else{
        Eigen::Vector3d q1, q2;
        q1 = guidance_path[f_id];
        q2 = guidance_path[f_id + 1];

        Eigen::Vector3d ui = (q2 - q1).normalized();
        Eigen::Vector3d uj = (q1 - cur_pos).normalized();

        if(ui.dot(uj) < 0){
            start_p = q2;
            guidance_path.erase(guidance_path.begin(), guidance_path.begin() + f_id + 1);
        }else{
            start_p = q1;
            guidance_path.erase(guidance_path.begin(), guidance_path.begin() + f_id);
        }
    }

    // 避免重复插入
    if(guidance_path.empty()){
        guidance_path.push_back(start_p);
    } else if((guidance_path[0] - start_p).norm() > 1e-6){
        guidance_path.insert(guidance_path.begin(), start_p);
    }
}

/*-------------------------------------------------------------------------------*/
/*--------------------------路径碰撞检查与安全路径搜索------------------------------*/
/*-------------------------------------------------------------------------------*/
void Planner::check_collision(){
    collision_segs.clear();
    if(guidance_path.size() < 2) return; // 异常保护（新增；测试）

    // size_t start_id, end_id;
    size_t start_id = 0;
    for(size_t id = 0; id < guidance_path.size() - 1; id++){
        if ((!map_utils->HitObstacles(guidance_path[id]) /*|| map_utils->OutOfMap(guidance_path[id])*/) && map_utils->HitObstacles(guidance_path[id + 1])){
            start_id = id;
        }
        else if(map_utils->HitObstacles(guidance_path[id]) && (!map_utils->HitObstacles(guidance_path[id + 1])  /*|| map_utils->OutOfMap(guidance_path[id])*/)){
            size_t end_id = id + 1;
            collision_segs.push_back(std::make_pair(start_id, end_id));
        }
    }

    // ROS_WARN("Collision segment info : \n");
    // size_t j = 0;
    // for(auto seg : collision_segs){
    //     std::cout << "> cs[" << ++j <<"] start_id=" << seg.first << ", end_id=" << seg.second << "\n";
    // }
}

void Planner::find_collision_free_path(){
    collision_free_segs.clear();
    if(guidance_path.size() < 2) {
        ROS_INFO("Guidance path too short, skip A* search");
        return;
    }
    astar->path_search(
        map_utils,
        guidance_path,
        collision_segs,
        collision_free_segs,
        cur_pos
    ); 

    // ROS_WARN("AStar result:");
    // int i = 0;
    // for(auto traj : collision_free_segs){
    //     std::cout << "cfs[" << ++i << "]: ";
    //     for(auto p : traj){
    //         std::cout << "(" << p(0) << ", " << p(1) << ", " << p(2) << "), ";
    //     }
    //     std::cout << "\n";
    // }

    // visualize_searched_path(); // For Test
}

/*-------------------------------------------------------------------------------*/
/*-------------------------------------END---------------------------------------*/
/*-------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------*/
/*-------------------------------优化问题构造与求解--------------------------------*/
/*-------------------------------------------------------------------------------*/
void Planner::construct_optimize_content(){
    optimize_content.clear();

    ROS_INFO("Construct optimization content");

    if (collision_segs.size() != collision_free_segs.size()){
        ROS_ERROR("Collision segment process fault!");
        return;
    }

    if (collision_segs.empty()){
        ROS_WARN("No collision segment, construct original optimize content");
        for(auto p : guidance_path){
            optimize_content.push_back(
                std::make_pair(
                    p,
                    std::make_pair(
                        Eigen::Vector3d::Zero(3),
                        Eigen::Vector3d::Zero(3)
                    )
                )
            );
        }
        return;
    }

    size_t i = 0, j = 0;
    bool c_flag = false;
    while(i < guidance_path.size()){
        // std::cout << "Current cs_id j = " << j << ", gd_id i = " << i;
        if((i != collision_segs[j].first && c_flag == false) || j >= collision_segs.size()){
            // std::cout << " here i != cs[j].first_id" << "\n";
            optimize_content.push_back(
                std::make_pair(
                    guidance_path[i],
                    std::make_pair(Eigen::Vector3d::Zero(3), Eigen::Vector3d::Zero(3))
                )
            );
            i += 1;
        }
        else if(i == collision_segs[j].first || c_flag == true){
            // if(i <= collision_segs[j].second && i != collision_segs[j].first){
            //     std::cout << " here i <= (cs[j].second_id = " << collision_segs[j].second << ")\n";
            // }else{
            //     std::cout << " here i = (cs[j].first_id = " << collision_segs[j].first << ")\n";
            // }

            if(i == collision_segs[j].first){
                c_flag = true;
                optimize_content.push_back(
                    std::make_pair(
                        guidance_path[i],
                        std::make_pair(Eigen::Vector3d::Zero(3), Eigen::Vector3d::Zero(3))
                    )
                );
                i += 1;
                continue;
            }

            if(i == collision_segs[j].second){
                c_flag = false;
                j += 1;
                optimize_content.push_back(
                    std::make_pair(
                        guidance_path[i],
                        std::make_pair(Eigen::Vector3d::Zero(3), Eigen::Vector3d::Zero(3))
                    )
                );
                i += 1;
                continue;
            }

            c_flag = true;

            Eigen::Vector3d R = (guidance_path[i + 1] - guidance_path[i - 1]).normalized();
            Eigen::Vector3d p = Eigen::Vector3d::Zero(3);
            Eigen::Vector3d v = Eigen::Vector3d::Zero(3);

            for(size_t k = 0; k < collision_free_segs[j].size() - 1; k++){
                auto ak = collision_free_segs[j][k];
                auto ak1 = collision_free_segs[j][k+1];

                auto seg_vec = ak1 - ak;
                auto norm_seg = (seg_vec).norm();

                if(norm_seg < 1e-6) continue;
                auto numerator = R.dot((guidance_path[i] - ak));
                auto denominator = R.dot(seg_vec);

                if (std::abs(denominator) < 1e-6) continue;

                auto t = numerator / denominator;
                if (t >= 0.0 && t <= 1.0){
                    p = ak + t * seg_vec;
                    v = (guidance_path[i] - p).normalized();
                    break;
                }
            }

            // Eigen::Vector3d v = (p - guidance_path[i]).normalized();

            optimize_content.push_back(
                std::make_pair(
                    guidance_path[i],
                    std::make_pair(p, v)
                )
            );
            i += 1;
        }
    }

    /* Test log */
    // for(auto c : optimize_content){
    //     std::cout << "- ControlPoint=(" 
    //             << c.first(0) << ", " << c.first(1) << ", " << c.first(2) 
    //             << "), (p,v)=(" 
    //             << c.second.first(0) << ", " << c.second.first(1) << ", " << c.second.first(2)
    //             << "), (" 
    //             << c.second.second(0) << ", " << c.second.second(1) << ", " << c.second.second(2) 
    //             << ")\n";
    // }
}

void Planner::path_optimize(){
    std::vector<Eigen::Vector3d> optimize_result; 

    // ROS_WARN("Control points before optimize:");
    // for(auto c : optimize_content){
    //     std::cout << "q = ("<< c.first(0) << ", " << c.first(1) << ", " << c.first(2) << ")\n"; 
    // }
    // std::cout << "\n";

    double t0 = ros::Time::now().toSec() * 1000.0f;

    optimizer->solve_optimize(optimize_content, optimize_result);

    double t1 = ros::Time::now().toSec() * 1000.0f;

    ROS_WARN("Path optimize cost = %.3f ms.", t1 - t0);

    // ROS_WARN("Control points after optimize:");
    // for(auto c : optimize_result){
    //     std::cout << "q = ("<< c(0) << ", " << c(1) << ", " << c(2) << ")\n"; 
    // }
    // std::cout << "\n";

    // 将优化结果转换为任务点
    bspline_points.clear();
    for(auto p : optimize_result){
        geometry_msgs::Point pt;
        pt.x = p(0), pt.y = p(1), pt.z = p(2);
        bspline_points.push_back(pt);
    }
    // bspline_points.pop_back(); // Temp fix
    ROS_INFO("Optimize successfully finished.");

    visualize_planning_path();

    // ros::shutdown(); // For test
}
/*-------------------------------------------------------------------------------*/
/*--------------------------------------END--------------------------------------*/
/*-------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------*/
/*-----------------------------------RVIZ可视化插件-------------------------------*/
/*-------------------------------------------------------------------------------*/
void Planner::visualize_guidance_path(){
    if(!guidance_path.empty()){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03;
        marker.scale.y = 0.03;
        marker.scale.z = 0.03;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for(const auto& p : guidance_path){
            geometry_msgs::Point pt;
            pt.x = p(0);
            pt.y = p(1);
            pt.z = p(2);
            marker.points.emplace_back(pt);
        }

        guidance_path_visual_pub.publish(marker);

        visualization_msgs::Marker pt_marker;
        pt_marker.header.frame_id = "map";
        pt_marker.header.stamp = ros::Time::now();
        pt_marker.id = 1;
        pt_marker.type = visualization_msgs::Marker::POINTS;
        pt_marker.action = visualization_msgs::Marker::ADD;
        pt_marker.pose.orientation.w = 1.0;
        pt_marker.scale.x = 0.08;
        pt_marker.scale.y = 0.08;
        pt_marker.scale.z = 0.08;
        pt_marker.color.r = 0.0;
        pt_marker.color.g = 0.0;
        pt_marker.color.b = 0.0;
        pt_marker.color.a = 1.0;

        for(const auto& p : guidance_path){
            geometry_msgs::Point pt;
            pt.x = p(0);
            pt.y = p(1);
            pt.z = p(2);
            pt_marker.points.emplace_back(pt);
        }

        guidance_path_visual_pub.publish(pt_marker);
    }
}

void Planner::visualize_searched_path(){
    if(!collision_free_segs.empty()){
        int id = 0;
        for(const auto &traj : collision_free_segs){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.id = id++;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.04;
            marker.scale.y = 0.04;
            marker.scale.z = 0.04;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            for(const auto& p : traj){
                geometry_msgs::Point pt;
                pt.x = p(0);
                pt.y = p(1);
                pt.z = p(2);
                marker.points.emplace_back(pt);
            }

            astar_path_visual_pub.publish(marker);

            visualization_msgs::Marker pt_marker;
            pt_marker.header.frame_id = "map";
            pt_marker.header.stamp = ros::Time::now();
            pt_marker.id = id++;
            pt_marker.type = visualization_msgs::Marker::POINTS;
            pt_marker.action = visualization_msgs::Marker::ADD;
            pt_marker.pose.orientation.w = 1.0;
            pt_marker.scale.x = 0.05;
            pt_marker.scale.y = 0.05;
            pt_marker.scale.z = 0.05;
            pt_marker.color.r = 1.0;
            pt_marker.color.g = 0.0;
            pt_marker.color.b = 0.0;
            pt_marker.color.a = 1.0;

            for(const auto& p : traj){
                geometry_msgs::Point pt;
                pt.x = p(0);
                pt.y = p(1);
                pt.z = p(2);
                pt_marker.points.emplace_back(pt);
            }

            astar_path_visual_pub.publish(pt_marker);
        }
    }
}

void Planner::visualize_planning_path(){
    if(!bspline_points.empty()){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = 120;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 0.5;

        for(const auto& p : bspline_points){
            geometry_msgs::Point pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = p.z;
            marker.points.emplace_back(pt);
        }

        planned_path_visual_pub.publish(marker);
    }
}

/*-------------------------------------------------------------------------------*/
/*--------------------------------------END--------------------------------------*/
/*-------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------------*/
/*-----------------------------------ROS消息发布器--------------------------------*/
/*-------------------------------------------------------------------------------*/
void Planner::publish_mission_points(){
    geometry_msgs::PoseArray mission_points;
    mission_points.header.frame_id = "map";
    mission_points.header.stamp = ros::Time::now();

    mission_points.poses.clear();

    for(const auto& p : bspline_points){
        geometry_msgs::Pose set_point;
        set_point.position.x = p.x;
        set_point.position.y = p.y;
        set_point.position.z = p.z;
        mission_points.poses.emplace_back(set_point);
    }

    planned_path_mission_pub.publish(mission_points);
}

void Planner::publish_emergency_hover_cmd(){
    geometry_msgs::Twist hover;

    hover.linear.x = 0.0f;
    hover.linear.y = 0.0f;
    hover.linear.z = 0.0f;
    hover.angular.x = 0.0f;
    hover.angular.y = 0.0f;
    hover.angular.z = 0.0f;

    emergency_hover_cmd_pub.publish(hover);
}

void Planner::publish_flight_interrupt_signal(){
    flight_interrupt_signal_pub.publish(fs.interrupt_signal);
}
/*-------------------------------------------------------------------------------*/
/*---------------------------------------END-------------------------------------*/
/*-------------------------------------------------------------------------------*/
