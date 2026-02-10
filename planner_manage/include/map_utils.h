# pragma once

# include <octomap/octomap.h>
# include <octomap_ros/conversions.h>
# include <Eigen/Core>
# include <sensor_msgs/PointCloud2.h>
# include <sensor_msgs/point_cloud2_iterator.h>
# include <geometry_msgs/Point.h>
# include <queue>
# include <pcl/point_cloud.h>
# include <pcl/point_types.h>
# include <pcl/conversions.h>
# include <pcl_ros/transforms.h>
# include <pcl_conversions/pcl_conversions.h>

# include <octomap/octomap.h>
# include <octomap/OcTree.h>
# include <octomap_msgs/conversions.h>
# include <octomap_msgs/Octomap.h>

# include "common_utils.h"
# include "ros/ros.h"

using OctoTreePtr = std::shared_ptr<octomap::OcTree>;

class MapUtils{
public:
    OctoTreePtr map;

    double map_z_min;
    double map_z_max;
    double map_r;
    double FOV_angle;
    double FOV_depth;
    double resolution; 
    double pro_hit;
    double pro_miss;
    double clamping_thresmin;
    double clamping_thresmax;

    ros::Publisher octomap_pub;

    void InitMapUtils(
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
    );

    bool InsertNewMap(const sensor_msgs::PointCloud2::ConstPtr& msg);

    bool OutOfMap(Eigen::Vector3d const& p, Eigen::Vector3d const& cur_pos);
    bool HitObstacles(Eigen::Vector3d const& p);

    bool map_visual();
};