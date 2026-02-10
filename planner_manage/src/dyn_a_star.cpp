# include "../include/dyn_a_star.h"
# include "../include/map_utils.h"

void AStarSearch::init_searcher(
    int set_search_mode,
    double set_step_size, 
    int set_max_iter,
    double set_max_time, 
    double set_reached_threshold,
    double set_hfunc_factor
){
    step_size = set_step_size;
    max_iter = set_max_iter;
    reached_threshold = set_reached_threshold;
    search_mode = set_search_mode;
    max_time = set_max_time;
    hfunc_factor = set_hfunc_factor;

    action_space.push_back(Eigen::Vector3d( 1,  0,  0));
    action_space.push_back(Eigen::Vector3d(-1,  0,  0));
    action_space.push_back(Eigen::Vector3d( 0,  1,  0));
    action_space.push_back(Eigen::Vector3d( 0, -1,  0));
    action_space.push_back(Eigen::Vector3d( 1,  1,  0));
    action_space.push_back(Eigen::Vector3d( 1, -1,  0));
    action_space.push_back(Eigen::Vector3d(-1,  1,  0));
    action_space.push_back(Eigen::Vector3d(-1, -1,  0));

    if(search_mode == 3){
        action_space.push_back(Eigen::Vector3d( 0,  0,  1));
        action_space.push_back(Eigen::Vector3d( 0,  0, -1));
        action_space.push_back(Eigen::Vector3d( 1,  0,  1));
        action_space.push_back(Eigen::Vector3d( 1,  0, -1));
        action_space.push_back(Eigen::Vector3d(-1,  0,  1));
        action_space.push_back(Eigen::Vector3d(-1,  0, -1));
        action_space.push_back(Eigen::Vector3d( 0,  1,  1));
        action_space.push_back(Eigen::Vector3d( 0,  1, -1));
        action_space.push_back(Eigen::Vector3d( 0, -1,  1));
        action_space.push_back(Eigen::Vector3d( 0, -1, -1));
        action_space.push_back(Eigen::Vector3d( 1,  1,  1));
        action_space.push_back(Eigen::Vector3d( 1,  1, -1));
        action_space.push_back(Eigen::Vector3d( 1, -1,  1));
        action_space.push_back(Eigen::Vector3d( 1, -1, -1));
        action_space.push_back(Eigen::Vector3d(-1,  1,  1));
        action_space.push_back(Eigen::Vector3d(-1,  1, -1));
        action_space.push_back(Eigen::Vector3d(-1, -1,  1));
        action_space.push_back(Eigen::Vector3d(-1, -1, -1));
    }
}

static inline double h(const Node& pt, const Eigen::Vector3d& goal, double factor) {
    // static double scale_factor = 2.0;
    return (pt.coordinate - goal).norm() * factor;
}

void AStarSearch::path_search(
    MapUtils* map_utils, 
    std::vector<Eigen::Vector3d>& gd_path, 
    std::vector<std::pair<size_t, size_t>>& cs_ids,
    std::vector<std::vector<Eigen::Vector3d>>& csfgs,
    Eigen::Vector3d const& cur_pos
) {
    size_t cs_segs_cnt = cs_ids.size();
    if (!cs_ids.empty()) {
        // ROS_INFO("Find %ld collision segments.", cs_segs_cnt);
        csfgs.clear();

        // std::vector<Eigen::Vector3d> single_path;

        for (const auto& seg : cs_ids) {
            size_t j = 0;
            // std::cout << "Current handle seg > start_id=" << seg.first << ", end_id=" << seg.second;
            // std::cout << " | Current handle seg > start_pt=(" 
            //         << gd_path[seg.first](0) << gd_path[seg.first](1) << gd_path[seg.first](2) << "), end_pt=(" 
            //         << gd_path[seg.second](0) << gd_path[seg.second](1) << gd_path[seg.second](2) << ")\n";
            Eigen::Vector3d start = gd_path[seg.first];
            Eigen::Vector3d goal = gd_path[seg.second];

            // single_path.clear();
            
            // 使用priority_queue模拟heapq，存储(f, counter, Node)
            using QueueItem = std::pair<double, std::pair<int, Node>>;
            std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open_list;
            
            std::vector<Node> close_list;
            int heap_counter = 0;
            
            // 参数检查
            if (start.hasNaN() || goal.hasNaN()) {
                ROS_WARN("ERROR: start or goal or map_config is None !");
                start.hasNaN() ?  csfgs.push_back(std::vector<Eigen::Vector3d>()) : csfgs.push_back(std::vector<Eigen::Vector3d>{start});
                continue;
            }
            
            if ((start - goal).norm() < step_size / 2.0) {
                ROS_WARN("STOP: start and goal are too close !(step_size=%.3f m)", step_size);
                csfgs.push_back(std::vector<Eigen::Vector3d>{start, goal});
                continue;
            }
            
            auto start_time = std::chrono::high_resolution_clock::now();
            std::unordered_set<std::string> explored_xyz; // 用字符串作为键
            int iter_cnt = 0;
            
            double max_search_dist = (start - goal).norm() * 10.0;
            
            // 初始化起点
            Node start_node;
            start_node.parent_coord = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            start_node.coordinate = start;
            start_node.g = 0.0;
            open_list.push({0.0, {heap_counter++, start_node}});
            
            auto search_start_time = std::chrono::high_resolution_clock::now();
            
            // 主循环（完全保留Python逻辑结构）
            while (!open_list.empty()) {
                if (iter_cnt >= max_iter) {
                    ROS_WARN("A* STOP: Reached max iterations (.%d)", max_iter);
                    break;
                }
                
                auto current_item = open_list.top();
                open_list.pop();
                Node current_point = current_item.second.second;
                double current_dist_from_start = current_point.g;
                
                if (current_dist_from_start > max_search_dist) {
                    ROS_WARN("A* STOP: Exceed max search distance (%.3f)", max_search_dist);
                    continue;
                }
                
                if ((current_point.coordinate - goal).norm() < step_size / 2.0) {
                    close_list.push_back(current_point);
                    ROS_WARN("A* SUCCESS: Reached goal");
                    break;
                }
                
                // 检查是否在close_list中（完全翻译Python的any逻辑）
                bool in_close = false;
                for (const auto& p : close_list) {
                    if ((p.coordinate - current_point.coordinate).norm() < 1e-6) {
                        in_close = true;
                        break;
                    }
                }
                if (in_close) continue;
                
                close_list.push_back(current_point);
                iter_cnt++;
                
                // 扩展邻居（完全保留Python的for act循环）
                for (const auto& act : action_space) {
                    double x = current_point.coordinate.x() + step_size * act.x();
                    double y = current_point.coordinate.y() + step_size * act.y();
                    double z = current_point.coordinate.z() + step_size * act.z();
                    double step_cost = act.norm() * step_size;
                    Eigen::Vector3d new_coord(x, y, z);
                    
                    // 创建tuple键（字符串形式）
                    std::string xyz_tuple = std::to_string(int(round(new_coord.x() * 1000))) + "," + 
                                        std::to_string(int(round(new_coord.y() * 1000))) + "," + 
                                        std::to_string(int(round(new_coord.z() * 1000)));
                    
                    // 完全保留Python的条件判断顺序
                    if (explored_xyz.count(xyz_tuple) || 
                        map_utils->HitObstacles(new_coord) || 
                        map_utils->OutOfMap(new_coord, cur_pos)) {
                        continue;
                    }
                    
                    double new_dist_from_start = current_point.g + step_cost;
                    if (new_dist_from_start > max_search_dist) {
                        continue;
                    }
                    
                    explored_xyz.insert(xyz_tuple);
                    
                    Node new_pt;
                    new_pt.parent_coord = current_point.coordinate;
                    new_pt.coordinate = new_coord;
                    new_pt.g = new_dist_from_start;
                    
                    double new_f = new_pt.g + h(new_pt, goal, hfunc_factor);
                    open_list.push({new_f, {heap_counter++, new_pt}});
                }
                
                // 时间限制（完全翻译Python的time.time() > 0.5）
                auto now = std::chrono::high_resolution_clock::now();
                double elapsed = std::chrono::duration<double>(now - search_start_time).count();
                if (elapsed > max_time) {
                    ROS_WARN("A* EXIT: Reaching max search time limit!");
                    break;
                }
            }
            
            // 回溯路径（完全保留Python的逻辑：遍历close_list找parent）
            std::vector<Eigen::Vector3d> path;
            if (close_list.empty()) {
                ROS_WARN("A* WARNING: close_list is empty, return start only");
                csfgs.push_back(std::vector<Eigen::Vector3d>{start});
                continue;
            }
            
            double min_dist = std::numeric_limits<double>::infinity();
            Node best_node = close_list.back();
            for (const auto& node : close_list) {
                double dist = (node.coordinate - goal).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    best_node = node;
                }
            }
            
            Node node = best_node;
            while (true) {
                path.push_back(node.coordinate);
                if (node.parent_coord.hasNaN()) {
                    break;
                }
                bool parent_found = false;
                for (const auto& n : close_list) {
                    if ((n.coordinate - node.parent_coord).norm() < 1e-6) {
                        node = n;
                        parent_found = true;
                        break;
                    }
                }
                if (!parent_found) {
                    break;
                }
            }
            
            std::reverse(path.begin(), path.end());
            
            auto end_time = std::chrono::high_resolution_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            ROS_WARN("A* RESULT: Path length = %ld, time cost = %.3f ms", path.size(), elapsed_ms);
            
            csfgs.push_back(path);
        }
    }
}
    // size_t cs_segs_cnt = cs_ids.size();
    // if (!cs_ids.empty()) {
    //     ROS_INFO("Find %ld collision segments.", cs_segs_cnt);
    //     csfgs.clear();

    //     std::vector<Eigen::Vector3d> single_path;

    //     for (const auto& seg : cs_ids) {
    //         size_t j = 0;
    //         std::cout << "Current handle seg > start_id=" << seg.first << ", end_id=" << seg.second;
    //         std::cout << " | Current handle seg > start_pt=(" << gd_path[seg.first] << "), end_pt=(" << gd_path[seg.second] << ")\n";
    //         Eigen::Vector3d start = gd_path[seg.first];
    //         Eigen::Vector3d goal = gd_path[seg.second];
    //         std::cout << "Start pt occpancy = " << map_utils->HitObstacles(start) << "\n";
    //         std::cout << "End   pt occpancy = " << map_utils->HitObstacles(goal) << "\n";

    //         single_path.clear();
            
    //         std::unordered_map<Eigen::Vector3d, AStarNode, Vec3Hash, Vec3Equal> all_nodes;
    //         std::unordered_set<Eigen::Vector3d, Vec3Hash, Vec3Equal> closed_set;
            
    //         auto cmp = [&](AStarNode* a, AStarNode* b) { return a->g + a->h > b->g + b->h; };
    //         std::priority_queue<AStarNode*, std::vector<AStarNode*>, decltype(cmp)> open_queue(cmp);
            
    //         // 初始化起点
    //         AStarNode start_node;
    //         start_node.pos = start;
    //         start_node.g = 0;
    //         start_node.h = (start - goal).norm();
    //         all_nodes[start] = start_node;
    //         open_queue.push(&all_nodes[start]);
            
    //         AStarNode* goal_node = nullptr;
    //         AStarNode* best_node = &all_nodes[start];
    //         int iter = 0;
            
    //         // 主循环
    //         while (!open_queue.empty() && iter < max_iter) {
    //             iter++;
                
    //             AStarNode* curr = open_queue.top();
    //             open_queue.pop();
                
    //             if (closed_set.count(curr->pos)) continue;
    //             closed_set.insert(curr->pos);
                
    //             if (curr->g + curr->h < best_node->g + best_node->h) best_node = curr;
                
    //             // 目标检查
    //             if (curr->h < reached_threshold) {
    //                 goal_node = curr;
    //                 break;
    //             }
                
    //             // 扩展26邻域
    //             for (const auto& act : action_space) {
    //                 Eigen::Vector3d npos = curr->pos + step_size * act.normalized();
    //                 if (map_utils->OutOfMap(npos, cur_pos) || 
    //                     map_utils->HitObstacles(npos) ||
    //                     closed_set.count(npos)
    //                 )  continue;
                    
    //                 double ng = curr->g + (step_size * act.normalized()).norm();
    //                 auto it = all_nodes.find(npos);
                    
    //                 if (it == all_nodes.end()) {
    //                     AStarNode n;
    //                     n.pos = npos;
    //                     n.g = ng;
    //                     n.h = 2.0 * (npos - goal).norm();
    //                     n.parent_pos = curr->pos;
    //                     all_nodes[npos] = n;
    //                     open_queue.push(&all_nodes[npos]);
    //                 } else if (ng < it->second.g) {
    //                     it->second.g = ng;
    //                     it->second.parent_pos = curr->pos;
    //                     open_queue.push(&it->second);
    //                 }
    //             }
    //         }
            
    //         // 回溯路径
    //         AStarNode* curr = goal_node ? goal_node : best_node;
    //         while ((curr->pos - start).norm() > 1e-6) {
    //             single_path.push_back(curr->pos);
    //             auto it = all_nodes.find(curr->parent_pos);
    //             if (it == all_nodes.end()) break;
    //             curr = &it->second;
    //         }
    //         single_path.push_back(start);
    //         std::reverse(single_path.begin(), single_path.end());
            
    //         if (goal_node) {
    //             ROS_INFO("3D A* SUCCESS: %zu path points", single_path.size());
    //         } else {
    //             ROS_WARN("3D A* FAILED: nearest path %zu points", single_path.size());
    //         }

    //         csfgs.push_back(single_path);
    //     }
    //     size_t csfgs_cnt = csfgs.size();
    //     ROS_INFO("Handle %ld collision free segments.", csfgs_cnt);
    // }