# include "../include/map_utils.h"
# include <math.h>

void MapUtils::InitMapUtils(int st, double angle, double depth, double vr){
    sensor_type = st;
    FOV_angle = angle;
    FOV_depth = depth;
    voxel_resolution = vr;

    GridMap = std::make_shared<std::vector<geometry_msgs::Point>>();
    GridMap->reserve(500); // 预分配大小防止非空检查触发
}

bool MapUtils::InsertNewMap(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg == nullptr || msg->data.empty()) {
        ROS_WARN("Received empty point cloud message!(No obstacle observed!)");
        return true;
    }
    
    // 创建新的vector存储新数据（避免直接清空旧数据造成访问冲突）
    auto new_grid_map = std::make_shared<std::vector<geometry_msgs::Point>>();
    new_grid_map->reserve(msg->width * msg->height);
    
    try {
        // 使用PointCloud2ConstIterator遍历点云（高效且安全）
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        
        // 遍历所有点
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            // 检查有效性
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
                continue;
            }
            
            geometry_msgs::Point point;
            point.x = *iter_x;
            point.y = *iter_y;
            point.z = *iter_z;
            
            new_grid_map->push_back(point);
        }
        
        // 原子交换更新GridMap（线程安全）
        {
            std::lock_guard<std::mutex> lock(grid_map_mutex_);
            GridMap = new_grid_map;
        }
        
        ROS_INFO("Successfully converted PointCloud2 to GridMap with %zu points", GridMap->size());
        return true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to parse point cloud: %s", e.what());
        return false;
    }
}

bool MapUtils::OutOfMap(Eigen::Vector3d const& p, Eigen::Vector3d const& cur_pos){
    if(p(2) < 0.1 || p(2) >= 3.0) return true;
    else if(abs(atan2(p(2) - cur_pos(2), p(0) - cur_pos(0))) >= FOV_angle) return true;

    return false;
}

bool MapUtils::OutOfMap(Eigen::Vector3d const& pt){
    if(pt(2) < 0.1 || pt(2) >= 3.0) return true;
    else return false;
}

bool MapUtils::HitObstacles(Eigen::Vector3d const& p){
    if (!GridMap || GridMap->empty()) {
        ROS_WARN_THROTTLE(1.0, "GridMap is empty (Not initialized) OR No obstacles.");
        return false;
    }
    
    // 多线程信息访问保护
    std::lock_guard<std::mutex> lock(grid_map_mutex_);
    
    double x = floor(p(0) * 10.0 + 0.5) / 10.0;
    double y = floor(p(1) * 10.0 + 0.5) / 10.0;
    double z = floor(p(2) * 10.0 + 0.5) / 10.0;
    
    double r = 0.5 * voxel_resolution;
    for (const auto& pt : *GridMap){
        if (abs(x - pt.x) <= r &&
        abs(y - pt.y) <= r &&
        abs(z - pt.z) <= r) return true;
    }
    return false;
}

void MapUtils::ClearGridMap() {
    if (GridMap && !GridMap->empty()) {
        std::lock_guard<std::mutex> lock(grid_map_mutex_);  // 线程安全
        GridMap->clear();
    }
}