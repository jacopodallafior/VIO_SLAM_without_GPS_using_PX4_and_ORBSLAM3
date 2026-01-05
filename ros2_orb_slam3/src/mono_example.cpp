/*
* Originally adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
* Author: Azmyin Md. Kamal
* Version: 1.0
* Date: 01/01/2024
* Compatible for ROS2 Humble
*/

//* Import all necessary modules
#include "ros2_orb_slam3/common.hpp" //* equivalent to orbslam3_ros/include/common.h

//* main
/* 1.0.0
int main(int argc, char **argv){
    rclcpp::init(argc, argv); // Always the first line, initialize this node
    
    //* Declare a node object
    auto node = std::make_shared<MonocularMode>(); 
    
    // rclcpp::Rate rate(20); // Set the desired update rate (e.g., 10 Hz)

    rclcpp::spin(node); // Blocking node
    rclcpp::shutdown();
    return 0;
}
*/
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonocularMode>();

    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
// ------------------------------------------------------------ EOF ---------------------------------------------


