# pragma once

# include <octomap/octomap.h>
# include <octomap_ros/conversions.h>
# include <Eigen/Core>
# include <sensor_msgs/PointCloud2.h>
# include <sensor_msgs/point_cloud2_iterator.h>
# include <geometry_msgs/Point.h>
# include <queue>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl_ros/transforms.h>

# include "common_utils.h"

class MapUtils{
public:
std::shared_ptr<std::vector<geometry_msgs::Point>> GridMap;
    // std::map<Eigen::Vector3d, int, Vec3Less> GridMap;

    int sensor_type;
    double FOV_angle;
    double FOV_depth;
    double voxel_resolution;

    void InitMapUtils(int, double, double, double);
    bool InsertNewMap(const sensor_msgs::PointCloud2::ConstPtr&);
    bool OutOfMap(Eigen::Vector3d const&, Eigen::Vector3d const&);
    bool OutOfMap(Eigen::Vector3d const&);
    bool HitObstacles(Eigen::Vector3d const&);

    void ClearGridMap();

private:
    std::mutex grid_map_mutex_;
};