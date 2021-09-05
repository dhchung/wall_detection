#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/point_cloud_conversion.h>
// #include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include "wall_msg.h"

#include <tf/transform_broadcaster.h>

#include "detect_wall.h"
#include <vector>

DetectWall d_wall;

sensor_msgs::PointCloud2 lidar_front_pc;
sensor_msgs::PointCloud2 lidar_port_pc;
sensor_msgs::PointCloud2 lidar_starboard_pc;

long double rosbag_time;

bool b_lidar_front = false;
bool b_lidar_port = false;
bool b_lidar_starboard = false;
tf::Transform t_front;
tf::Transform t_port2front;
tf::Transform t_starboard2front;

tf::Quaternion q_front;
tf::Quaternion q_port2front;
tf::Quaternion q_starboard2front;
ros::Publisher pub_pc_whole;
ros::Publisher pub_pc_filtered;
ros::Publisher pub_lines;

void LidarFrontHandle(const sensor_msgs::PointCloud2::ConstPtr & pc2_msg) {
    lidar_front_pc = *pc2_msg;
    b_lidar_front = true;
    rosbag_time = lidar_front_pc.header.stamp.toSec();
}

void LidarPortHandle(const sensor_msgs::PointCloud2::ConstPtr & pc2_msg) {
    lidar_port_pc = *pc2_msg;
    b_lidar_port = true;
}

void LidarStarboardHandle(const sensor_msgs::PointCloud2::ConstPtr & pc2_msg) {
    lidar_starboard_pc = *pc2_msg;
    b_lidar_starboard = true;
}

void ProcessPoints(double time){
    if(!b_lidar_front || !b_lidar_port || !b_lidar_starboard) {
        ROS_INFO("Lidar is not loaded");
        return;
    }
    bool lidar_front_time = lidar_front_pc.header.stamp.toSec() > (rosbag_time - 0.2);
    bool lidar_port_time = lidar_port_pc.header.stamp.toSec() > (rosbag_time - 0.2);
    bool lidar_starboard_time = lidar_starboard_pc.header.stamp.toSec() > (rosbag_time - 0.2);


    if(!lidar_front_time || !lidar_port_time || !lidar_starboard_time) {
        ROS_INFO("Time difference is bigger than 0.2s");
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> pcl_pc_lidar_front;
    pcl::fromROSMsg(lidar_front_pc, pcl_pc_lidar_front);

    pcl_ros::transformPointCloud(pcl_pc_lidar_front, pcl_pc_lidar_front, t_front);

    pcl::PointCloud<pcl::PointXYZI> pcl_pc_lidar_port;
    pcl::fromROSMsg(lidar_port_pc, pcl_pc_lidar_port);
    pcl_ros::transformPointCloud(pcl_pc_lidar_port, pcl_pc_lidar_port, t_front*t_port2front);


    pcl::PointCloud<pcl::PointXYZI> pcl_pc_lidar_starboard;
    pcl::fromROSMsg(lidar_starboard_pc, pcl_pc_lidar_starboard);
    pcl_ros::transformPointCloud(pcl_pc_lidar_starboard, pcl_pc_lidar_starboard, t_front*t_starboard2front);

    pcl::PointCloud<pcl::PointXYZI> pcl_pc_whole_pc;
    pcl_pc_whole_pc += pcl_pc_lidar_front;
    pcl_pc_whole_pc += pcl_pc_lidar_port;
    pcl_pc_whole_pc += pcl_pc_lidar_starboard;




    pcl::PointCloud<pcl::PointXYZI> pcl_pc_filtered_pc = pcl_pc_whole_pc;
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(pcl_pc_filtered_pc.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0f, 1.0f);
    pass.filter(pcl_pc_filtered_pc);

    Eigen::Matrix3Xf e_whole_pc;
    e_whole_pc.resize(3, pcl_pc_filtered_pc.points.size());
    for(size_t i = 0; i< pcl_pc_filtered_pc.points.size(); ++i) {
        e_whole_pc(0,i) = pcl_pc_filtered_pc.points[i].x;
        e_whole_pc(1,i) = pcl_pc_filtered_pc.points[i].y;
        e_whole_pc(2,i) = pcl_pc_filtered_pc.points[i].z;
    }

    std::vector<std::pair<std::pair<float, float>, std::pair<float, float>>> Lines =
        d_wall.GetOccupancyMap(e_whole_pc);

    sensor_msgs::PointCloud2 whole_cloud_msg;
    pcl::toROSMsg(pcl_pc_whole_pc, whole_cloud_msg);
    whole_cloud_msg.header.frame_id = "base_link";
    whole_cloud_msg.header.stamp = ros::Time::now();
    pub_pc_whole.publish(whole_cloud_msg);

    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(pcl_pc_filtered_pc, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "base_link";
    filtered_cloud_msg.header.stamp = ros::Time::now();
    pub_pc_filtered.publish(filtered_cloud_msg);

    core_msgs::wall_msg wall_line_msg;
    wall_line_msg.num = Lines.size();
    wall_line_msg.x_start.resize(Lines.size());
    wall_line_msg.y_start.resize(Lines.size());
    wall_line_msg.x_end.resize(Lines.size());
    wall_line_msg.y_end.resize(Lines.size());

    for(size_t i = 0; i < Lines.size(); ++i) {
        wall_line_msg.x_start[i] = Lines[i].first.first;
        wall_line_msg.y_start[i] = Lines[i].first.second;
        wall_line_msg.x_end[i] = Lines[i].second.first;
        wall_line_msg.y_end[i] = Lines[i].second.second;
    }
    
    pub_lines.publish(wall_line_msg);
    ROS_INFO("Published Wall No.: %d", int(Lines.size()));

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "wall_detection_node");
    ROS_INFO("Wall Detection Node Started");

    ros::NodeHandle nh;
    ros::Subscriber sub_lidar_front = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_front/os_cloud_node/points", 1000, LidarFrontHandle);
    ros::Subscriber sub_lidar_port = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_port/os_cloud_node/points", 1000, LidarPortHandle);
    ros::Subscriber sub_lidar_starboard = nh.subscribe<sensor_msgs::PointCloud2>("/lidar_starboard/os_cloud_node/points", 1000, LidarStarboardHandle);

    tf::TransformBroadcaster tf_br;

    t_front.setOrigin(tf::Vector3(0.0f, -0.090000f, 1.079000f));
    q_front.setRPY(0.0f, 0.700000*M_PI/180.0f, 4.600000*M_PI/180.0f);
    t_front.setRotation(q_front);

    t_port2front.setOrigin(tf::Vector3(-0.910000f, 1.646000f, 0.866000f));
    q_port2front.setRPY(-1.5f*M_PI/180.0f, 33.599998*M_PI/180.0f, 85.0f*M_PI/180.0f);
    t_port2front.setRotation(q_port2front);

    t_starboard2front.setOrigin(tf::Vector3(-1.120000f, -1.460000f, 0.908000f));
    q_starboard2front.setRPY(-2.100000f*M_PI/180.0f, 30.600000f*M_PI/180.0f, -100.599998*M_PI/180.0f);
    t_starboard2front.setRotation(q_starboard2front);


    pub_pc_filtered = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1000);
    pub_pc_whole = nh.advertise<sensor_msgs::PointCloud2>("whole_cloud",1000);
    pub_lines = nh.advertise<core_msgs::wall_msg>("wall_lines", 1000);

    rosbag_time = 0.0f;

    while(ros::ok()) {
        ProcessPoints(ros::Time::now().toSec());
        ros::Time time;
        time.fromSec(rosbag_time);

        // tf_br.sendTransform(tf::StampedTransform(t_front, ros::Time::now(), "map", "/lidar_front/os_sensor"));
        // tf_br.sendTransform(tf::StampedTransform(t_port2front, ros::Time::now(), "/lidar_front/os_sensor", "/lidar_port/os_sensor"));
        // tf_br.sendTransform(tf::StampedTransform(t_starboard2front, ros::Time::now(), "/lidar_front/os_sensor", "/lidar_starboard/os_sensor"));

        tf_br.sendTransform(tf::StampedTransform(t_front, time, "map", "/lidar_front/os_sensor"));
        tf_br.sendTransform(tf::StampedTransform(t_port2front, time, "/lidar_front/os_sensor", "/lidar_port/os_sensor"));
        tf_br.sendTransform(tf::StampedTransform(t_starboard2front, time, "/lidar_front/os_sensor", "/lidar_starboard/os_sensor"));

        ros::spinOnce();
    }

}