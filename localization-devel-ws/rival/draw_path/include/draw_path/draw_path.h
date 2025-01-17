#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include <cstdlib>
#include <iostream>
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::placeholders;

class Draw  : public rclcpp::Node{

    public:

        Draw();

    private:

        void lidar_final_callback(const nav_msgs::msg::Odometry::ConstPtr& msg);

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr draw_lidar_final_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr draw_lidar_final_pub_;

        nav_msgs::msg::Path lidar_final_Pose_;
};