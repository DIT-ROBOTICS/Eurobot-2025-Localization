#include "draw_path/draw_path.h"

Draw::Draw () : Node("draw_path_node"){

    rclcpp::Clock clock;

    lidar_final_Pose_.header.frame_id = "robot/map";
    lidar_final_Pose_.header.stamp = clock.now();

    draw_lidar_final_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("final_pose", 10, std::bind(&Draw::lidar_final_callback, this, _1));
    draw_lidar_final_pub_ = this->create_publisher<nav_msgs::msg::Path>("/lidar_final_Pose", 10);
}

void Draw::lidar_final_callback(const nav_msgs::msg::Odometry::ConstPtr& msg){

    geometry_msgs::msg::PoseStamped lidar_final_PoseStamped;

    lidar_final_PoseStamped.pose = msg->pose.pose;
    lidar_final_PoseStamped.header.frame_id = "robot/map";
    lidar_final_PoseStamped.header.stamp = msg->header.stamp;

    lidar_final_Pose_.poses.push_back(lidar_final_PoseStamped);

    draw_lidar_final_pub_->publish(lidar_final_Pose_);
}

int main(int argc, char * argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Draw>());
    rclcpp::shutdown();

    return 0;
}