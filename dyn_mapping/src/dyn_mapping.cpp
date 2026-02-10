# include "ros/ros.h"
# include <sensor_msgs/PointCloud2.h>
# include <sensor_msgs/point_cloud2_iterator.h>
# include <geometry_msgs/PoseStamped.h>
# include <iostream>
# include <tf/transform_listener.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl_ros/transforms.h>
# include <pcl/filters/voxel_grid.h>
# include <pcl/sample_consensus/method_types.h>
# include <pcl/sample_consensus/model_types.h>
# include <pcl/segmentation/sac_segmentation.h>
# include <pcl/filters/extract_indices.h>
# include <chrono>

tf::TransformListener* tf_listener;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);

ros::Publisher map_pub;

float inflate_r = 0.4f;
bool ground_filter = true;
bool inflate_filter = true;
float ground_threshold = 0.1f;
int inflate_mode = 0;

void voxel_downsample();
void inflate_process();
void filter_ground_points();
void publish_map();

void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

int main(int argc, char* argv[]){
    ros::init(argc, argv, "dyn_mapping");
    ros::NodeHandle nh("~");

    tf_listener = new tf::TransformListener;

    std::string depth_pc_topic;
    std::string map_pub_topic;
    double process_freq = 10.0;
    ros::param::get("depth_pc_topic", depth_pc_topic);
    ros::param::get("map_pub_topic", map_pub_topic);
    ros::param::get("process_freq", process_freq);
    ros::param::get("inflate_filter", inflate_filter);
    ros::param::get("inflate_r", inflate_r);
    ros::param::get("ground_filter", ground_filter);
    ros::param::get("ground_threshold", ground_threshold);
    ros::param::get("inflate_mode", inflate_mode);

    ros::Subscriber pcl_sub = nh.subscribe(depth_pc_topic, 1, pcl_callback);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>(map_pub_topic, 1);

    ros::Rate rate(process_freq);

    ROS_INFO("DYN_MAPPING_NODE LAUNCH FINISH.");
    ROS_INFO("depth_pc_topic : %s", depth_pc_topic.c_str());
    ROS_INFO("map_pub_topic : %s", map_pub_topic.c_str());
    ROS_INFO("Node process frequency = %.1f Hz", process_freq);
    ROS_INFO("Obstacle inflate r = %.1f m", inflate_r);
    ROS_INFO("Ground threshold h0 = %.1f m", ground_threshold);
    ROS_INFO("Inflate filter = %d [0:OFF 1:ON]", inflate_filter);
    ROS_INFO("Ground filter = %d [0:OFF 1:ON]", ground_filter);
    ROS_INFO("Inflate mode = %d [0:CUBE 1:BALL]", inflate_mode);

    while(!ros::isShuttingDown()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void voxel_downsample(){
    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(cloud_map);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_filtered);

    cloud_map = cloud_filtered;
}

void inflate_process(){
    float max_r_sqare = inflate_r * inflate_r;
    float step_size = 0.1f;
    int step_length = static_cast<int>(std::ceil(inflate_r / step_size));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig = cloud_map;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_out->reserve(cloud_orig->size() * 100);

    for(const auto p : cloud_orig->points){
        if(!ground_filter){
            if(p.z <= ground_threshold){
                cloud_out->emplace_back(p);
                continue;
            }
        }
        for(int i = 0; i < step_length; i++){
            float dx = i * step_size;
            for(int j = 0; j < step_length; j++){
                float dy = j * step_size;
                if(inflate_mode == 1){
                    float d_square_xy  = dx * dx + dy * dy;
                    if(d_square_xy - max_r_sqare >= 1e-6){
                        continue;
                    }
                }
                for(int k = 0; k < step_length; k++){
                    float dz = k * step_size;
                    if(inflate_mode == 1){
                        float d_square  = dx * dx + dy * dy + dz * dz;
                        if(d_square - max_r_sqare >= 1e-6){
                            continue;
                        }
                    }

                    cloud_out->points.emplace_back(p.x + dx, p.y + dy, p.z + dz);
                    cloud_out->points.emplace_back(p.x - dx, p.y + dy, p.z + dz);
                    cloud_out->points.emplace_back(p.x + dx, p.y - dy, p.z + dz);
                    cloud_out->points.emplace_back(p.x - dx, p.y - dy, p.z + dz);
                }
            }
        }
    }

    cloud_map = cloud_out;

    pcl::VoxelGrid<pcl::PointXYZ> vg;

    vg.setInputCloud(cloud_map);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_filtered);

    cloud_map = cloud_filtered;
}

void filter_ground_points(){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.15);
    seg.setInputCloud(cloud_map);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_map);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_map);
}

void publish_map(){
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_map, output);
    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();
    map_pub.publish(output);

    ROS_INFO("Map publish finished.");
}

void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("Receive pointcloud message.");

    pcl::fromROSMsg(*msg, *cloud_cam);
    pcl_ros::transformPointCloud("map", *cloud_cam, *cloud_map, *tf_listener);

    ROS_INFO("Transform points from %s -> map", msg->header.frame_id.c_str());

    ROS_WARN("Start points downsample and inflate process...");

    auto start_time = std::chrono::high_resolution_clock::now();

    voxel_downsample();

    if(ground_filter) filter_ground_points();

    if(inflate_filter) inflate_process();

    publish_map();

    auto end_time = std::chrono::high_resolution_clock::now();

    ROS_WARN("Map generate finished! Time Cost = %f ms.", std::chrono::duration<double, std::milli>(end_time - start_time).count());
}
