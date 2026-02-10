# include "../include/map_utils.h"
# include <math.h>

void MapUtils::InitMapUtils(
    double _map_z_min,
    double _map_z_max,
    double _map_r,
    double _resolution, 
    double _pro_hit, 
    double _pro_miss, 
    double _clamping_thresmin, 
    double _clamping_thresmax,
    double _FOV_angle,
    double _FOV_depth,
    ros::NodeHandle& nh
){
    map_z_min = _map_z_min;
    map_z_max = _map_z_max;
    map_r = _map_r;
    FOV_angle = _FOV_angle;
    FOV_depth = _FOV_depth;
    resolution = _resolution;
    pro_hit = _pro_hit;
    pro_miss = _pro_miss;
    clamping_thresmin = _clamping_thresmin;
    clamping_thresmax = _clamping_thresmax;
    FOV_angle = _FOV_angle;
    FOV_depth = _FOV_depth;

    map = std::make_shared<octomap::OcTree>(resolution);
    map->setProbHit(pro_hit);
    map->setProbMiss(pro_miss);
    map->setClampingThresMin(clamping_thresmin);
    map->setClampingThresMax(clamping_thresmax);

    octomap_pub = nh.advertise<octomap_msgs::Octomap>("/map_utils/octo_map", 1, true);

    ROS_WARN("Map initialization finished!");
}

bool MapUtils::InsertNewMap(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (!msg) {
        ROS_WARN("MapUtils $ PointCloud2 msg is null, skip map update!");
        return false;
    }
    if (!map) {
        ROS_ERROR("MapUtils $ OctoMap is not initialized, cannot update map!");
        return false;
    }
    
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    try {
        pcl::fromROSMsg(*msg, pcl_cloud);
    } catch (const std::exception& e) {
        ROS_ERROR("MapUtils $ Convert PointCloud2 to PCL failed: %s", e.what());
        return false;
    }

    octomap::Pointcloud octo_cloud;
    for (const auto& pcl_pt : pcl_cloud) {
        if (std::isnan(pcl_pt.x) || std::isnan(pcl_pt.y) || std::isnan(pcl_pt.z) ||
            std::isinf(pcl_pt.x) || std::isinf(pcl_pt.y) || std::isinf(pcl_pt.z)) {
            continue;
        }
        octo_cloud.push_back(pcl_pt.x, pcl_pt.y, pcl_pt.z);
    }

    if (octo_cloud.size() <= 0) {
        ROS_WARN("MapUtils $ No valid points in PointCloud2, skip map update!");
        return false;
    }

    // 经过dyn_mapping处理点云已转换到map帧，传感器位姿设为map帧原点(0,0,0)即可
    // OctoMap会以该位姿为起点，向点云点发射射线，更新「射线沿途自由空间+点云处占用空间」
    octomap::point3d sensor_pose(0.0, 0.0, 0.0);
    map->insertPointCloud(octo_cloud, sensor_pose, -1.0, false, resolution);

    map->prune();

    ROS_INFO("MapUtils $ Map update success!"/*Insert %ld valid points, total map nodes: %lu",octo_cloud.size(), map->size()*/);

    map_visual();

    return true;
}

bool MapUtils::OutOfMap(Eigen::Vector3d const& p, Eigen::Vector3d const& cur_pos){
    // if(!map){
    //     ROS_ERROR("OctomapPtr is empty!");
    //     return true;
    // }

    // octomap::point3d query_point(p(0), p(1), p(2));

    // double min_x, min_y, min_z;
    // double max_x, max_y, max_z;

    // map->getMetricMin(min_x, min_y, min_z);
    // map->getMetricMax(max_x, max_y, max_z);

    // bool in_x = (query_point.x() >= min_x + 1e-6) && (query_point.x() <= max_x - 1e-6);
    // bool in_y = (query_point.y() >= min_y + 1e-6) && (query_point.y() <= max_y - 1e-6);
    // bool in_z = (query_point.z() >= min_z + 1e-6) && (query_point.z() <= max_z - 1e-6);

    // return in_x && in_y && in_z;

    // double max_r = 20.0;

    if(p(2) <= map_z_min || p(2) >= map_z_max) return true;
    else if((p - cur_pos).norm() >= map_r) return true;
    else return false;
}

bool MapUtils::HitObstacles(Eigen::Vector3d const& p){
    if(!map){
        ROS_ERROR("OctomapPtr is empty!");
        return false;
    }

    octomap::point3d query_point(p(0), p(1), p(2));

    if(map->search(query_point) == nullptr){
        return false;
    }else{
        return map->isNodeOccupied(map->search(query_point));
    }
}

bool MapUtils::map_visual(){
    octomap_msgs::Octomap msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    
    // 转换（包含未知空间）
    if (octomap_msgs::fullMapToMsg(*map, msg)) {
        octomap_pub.publish(msg);
        ROS_INFO("Published full OctoMap");
    }

    return true;
}