#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>

class Scan {
public:
    Scan(ros::NodeHandle& nh, ros::NodeHandle& nh_);

private:
    void initialize();
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void local_filte_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    ros::NodeHandle nh, nh_;
    ros::Subscriber scan_sub;
    ros::Subscriber local_filte_sub;
    ros::Subscriber imu_sub;
    ros::Publisher scan_pub;

    std::string robot_name;
    int robot_angular_vel_topic;

    double lidar_frequency;
    double lidar_angular_vel;
    double robot_angular_vel;

    double stamp;
    double stamp_pre;
};

Scan::Scan(ros::NodeHandle& nh_g, ros::NodeHandle& nh_l){
    nh = nh_g;
    nh_ = nh_l;
    initialize();
}

void Scan::initialize(){
    bool ok = true;
    ok &= nh_.getParam("robot_name", robot_name);
    ok &= nh_.getParam("lidar_frequency", lidar_frequency);
    ok &= nh_.getParam("robot_angular_vel_topic", robot_angular_vel_topic);

    switch(robot_angular_vel_topic){
    case 0:
        local_filte_sub = nh.subscribe("global_filter", 10, &Scan::local_filte_callback, this);
    case 1:
        imu_sub = nh.subscribe("imu/data_cov", 10, &Scan::imu_callback, this);
    default:
        imu_sub = nh.subscribe("imu/data_cov", 10, &Scan::imu_callback, this);
    }
    scan_sub = nh.subscribe("scan", 10, &Scan::scan_callback, this);
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_adjust", 10);

    lidar_angular_vel = -6.28318 * lidar_frequency;
    stamp = 0;
    stamp_pre = 0;
}

void Scan::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    sensor_msgs::LaserScan scan_output;
    stamp_pre = stamp;
    stamp = msg->header.stamp.toSec();
    scan_output = *msg;
    // scan_output.angle_increment = 
    //     (1 + robot_angular_vel / lidar_angular_vel) * msg->angle_increment;
    scan_output.angle_min = msg->angle_min + robot_angular_vel * (stamp - stamp_pre);
    scan_pub.publish(scan_output);
}

void Scan::local_filte_callback(const nav_msgs::Odometry::ConstPtr& msg){
    robot_angular_vel = msg->twist.twist.angular.z;
}

void Scan::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    robot_angular_vel = msg->angular_velocity.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_adjust");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    Scan scan(nh, nh_local);

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}
