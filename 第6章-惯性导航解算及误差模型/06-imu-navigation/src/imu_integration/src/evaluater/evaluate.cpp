/*
轨迹评估
*/
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

static std::ofstream ground_truth_str;
static std::ofstream estimation_str;

bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
        std::cout << "无法生成文件: " << file_path;
        return false;
    }

    return true;
}

bool CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "无法建立文件夹: " << directory_path;
        return false;
    }
    return true;
}

void callback_truth(const nav_msgs::OdometryConstPtr & ground_truth){
    
    static bool is_file_created = false;

    if (!is_file_created) {
        if (!CreateDirectory("/workspace/assignments/06-imu-navigation/src/imu_integration/slam_data/trajectory"))
            return;
        if (!CreateFile(ground_truth_str, "/workspace/assignments/06-imu-navigation/src/imu_integration/slam_data/trajectory/ground_truth.txt"))
            return;
        is_file_created = true;
    }

    nav_msgs::Odometry odom = *ground_truth;
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Zero();
    transformation(0,3) = odom.pose.pose.position.x;
    transformation(1,3) = odom.pose.pose.position.y;
    transformation(2,3) = odom.pose.pose.position.z;

    Eigen::Quaternionf q;
    q.w() = odom.pose.pose.orientation.w;
    q.x() = odom.pose.pose.orientation.x;
    q.y() = odom.pose.pose.orientation.y;
    q.z() = odom.pose.pose.orientation.z;
    transformation.block<3,3>(0,0) = q.toRotationMatrix();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth_str << transformation(i, j);
            // 数据结束
            if (i == 2 && j == 3) {
                ground_truth_str << std::endl;
            } else {
                ground_truth_str << " ";
            }
        }
    }
}

void callback_estimation(const nav_msgs::OdometryConstPtr & estimate){
        static bool is_file_created = false;

    if (!is_file_created) {
        if (!CreateDirectory("/workspace/assignments/06-imu-navigation/src/imu_integration/slam_data/trajectory"))
            return;
        if (!CreateFile(estimation_str, "/workspace/assignments/06-imu-navigation/src/imu_integration/slam_data/trajectory/estimation.txt"))
            return;
        is_file_created = true;
    }

    nav_msgs::Odometry odom = *estimate;
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Zero();
    transformation(0,3) = odom.pose.pose.position.x;
    transformation(1,3) = odom.pose.pose.position.y;
    transformation(2,3) = odom.pose.pose.position.z;

    Eigen::Quaternionf q;
    q.w() = odom.pose.pose.orientation.w;
    q.x() = odom.pose.pose.orientation.x;
    q.y() = odom.pose.pose.orientation.y;
    q.z() = odom.pose.pose.orientation.z;
    transformation.block<3,3>(0,0) = q.toRotationMatrix();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            estimation_str << transformation(i, j);
            // 数据结束
            if (i == 2 && j == 3) {
                estimation_str << std::endl;
            } else {
                estimation_str << " ";
            }
        }
    }
}

int main(int argc,char ** argv){

    std::string node_name{"evaluater_node"};
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
    ros::Subscriber grd_sub = nh.subscribe("/pose/ground_truth",10,&callback_truth);
    ros::Subscriber est_sub = nh.subscribe("/pose/estimation",10,&callback_estimation);
    
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    ground_truth_str.close();
    return 0;
}