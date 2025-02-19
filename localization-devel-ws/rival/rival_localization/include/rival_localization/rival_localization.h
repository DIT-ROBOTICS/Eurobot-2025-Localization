#pragma once

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "obstacle_detector/msg/obstacles.hpp"

using std::placeholders::_1;

typedef struct rivalState {
    double x;
    double y;
}RivalState;

class Rival : public rclcpp::Node {

public:

    Rival() : Node("rival_localization"){

        initialize();
    }

private:
    void initialize();
    void obstacles_callback(const obstacle_detector::msg::Obstacles::ConstPtr& msg);
    void publish_rival();
    void fusion();
    void broadcast_rival_tf();
    bool in_playArea_obs(geometry_msgs::msg::Point center);
    bool within_lock(geometry_msgs::msg::Point pre, geometry_msgs::msg::Point cur, double dt);
    geometry_msgs::msg::Vector3 lpf(double gain, geometry_msgs::msg::Vector3 pre, geometry_msgs::msg::Vector3 cur);
    void timerCallback();

    rclcpp::Subscription<obstacle_detector::msg::Obstacles>::SharedPtr obstacles_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rival_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    obstacle_detector::msg::Obstacles obstacle;
    nav_msgs::msg::Odometry rival_output;
    geometry_msgs::msg::Point obstacle_pose;
    geometry_msgs::msg::Point rival_pose;
    geometry_msgs::msg::Vector3 obstacle_vel;
    geometry_msgs::msg::Vector3 rival_vel;
    rclcpp::Time obstacle_stamp;
    rclcpp::Time rival_stamp;

    geometry_msgs::msg::TransformStamped rival_tf;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> br;

    std::string robot_name;
    std::string rival_name;

    double x_max, x_min, y_max, y_min;
    double vel_lpf_gain;
    double locking_rad, p_locking_rad, freq;
    double lockrad_growing_rate;

    bool obstacle_ok, rival_ok;
};