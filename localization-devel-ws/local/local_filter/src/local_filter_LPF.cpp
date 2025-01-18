#include "rclcpp/rclcpp.hpp"
// msg
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
// matrix calulation
#include <eigen3/Eigen/Dense>
#include <math.h>
// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

struct RobotState
{
    Eigen::Vector3d mu;
    Eigen::Matrix3d sigma;
};

class GlobalFilterNode {
public:
    GlobalFilterNode(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_local) :
    nh_(nh), nh_local_(nh_local){
        // Initialize filter coefficients and initial values
        linear_x_ = 0.0;
        linear_y_ = 0.0;
        angular_z_ = 0.0;

        nh_local_->declare_parameter("LPF_alpha_x", 0.5); // filter coefficient
        nh_local_->declare_parameter("LPF_alpha_y", 0.5); // filter coefficient
        nh_local_->declare_parameter("linear_cov_max", 0.1);
        nh_local_->declare_parameter("angular_cov_max", 0.05);

        alpha_x=nh_local_->get_parameter("LPF_alpha_x").as_double();
        alpha_y=nh_local_->get_parameter("LPF_alpha_y").as_double();
        linear_cov_max_=nh_local_->get_parameter("linear_cov_max").as_double();
        angular_cov_max_=nh_local_->get_parameter("angular_cov_max").as_double();

        setpose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 50, std::bind(&GlobalFilterNode::setposeCallback, this, std::placeholders::_1));
        odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&GlobalFilterNode::odomCallback, this, std::placeholders::_1));
        imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>("imu/data_cov", 10, std::bind(&GlobalFilterNode::imuCallback, this, std::placeholders::_1));

        global_filter_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("local_filter", 100);
    }

    void diff_model(double v, double w, double dt)
    {
        double theta = robotstate_.mu(2);
        double s = sin(theta); double s_ = sin(theta + w * dt);
        double c = cos(theta); double c_ = cos(theta + w * dt);
        
        if ((w < 0.00001) && (w > -0.00001)){
            robotstate_.mu = robotstate_.mu + Eigen::Vector3d{ v * c * dt, v * s * dt, 0.0 };
        }else{
            robotstate_.mu = robotstate_.mu + Eigen::Vector3d{ v * (-s + s_) / w, v * (c - c_) / w, w * dt };
        }
        robotstate_.mu(2) = angleLimitChecking(robotstate_.mu(2));
    }

    void omni_model(double v_x, double v_y, double w, double dt)
    {
        /* ekf prediction function for omni wheel */
        Eigen::Vector3d d_state;

        d_state << (v_x * dt), (v_y * dt), (w * dt);

        double theta_ = robotstate_.mu(2) + d_state(2) / 2;
        double s__theta = sin(theta_);
        double c__theta = cos(theta_);

        Eigen::Matrix3d A;
        A << 1, 0, 0, 0, 1, 0, 0, 0, 1;

        Eigen::Matrix3d B;
        B << c__theta, -s__theta, 0, s__theta, c__theta, 0, 0, 0, 1;

        Eigen::Matrix3d cov_past;
        cov_past = robotstate_.sigma;

        /* Update robot state mean */
        robotstate_.mu = A * robotstate_.mu + B * d_state;
    }

    void setposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped & pose_msg)
    {
        double x = pose_msg.pose.pose.position.x;
        double y = pose_msg.pose.pose.position.y;

        tf2::Quaternion q;
        tf2::fromMsg(pose_msg.pose.pose.orientation, q);
        tf2::Matrix3x3 qt(q);
        double _, yaw;
        qt.getRPY(_, _, yaw);

        robotstate_.mu(0) = x;
        robotstate_.mu(1) = y;
        robotstate_.mu(2) = yaw;

        robotstate_.sigma(0, 0) = pose_msg.pose.covariance[0];   // x-x
        robotstate_.sigma(0, 1) = pose_msg.pose.covariance[1];   // x-y
        robotstate_.sigma(0, 2) = pose_msg.pose.covariance[5];   // x-theta
        robotstate_.sigma(1, 0) = pose_msg.pose.covariance[6];   // y-x
        robotstate_.sigma(1, 1) = pose_msg.pose.covariance[7];   // y-y
        robotstate_.sigma(1, 2) = pose_msg.pose.covariance[11];  // y-theta
        robotstate_.sigma(2, 0) = pose_msg.pose.covariance[30];  // theta-x
        robotstate_.sigma(2, 1) = pose_msg.pose.covariance[31];  // theta-y
        robotstate_.sigma(2, 2) = pose_msg.pose.covariance[35];  // theta-theta
    }

    void odomCallback(const nav_msgs::msg::Odometry & odom_msg) {
        // get velocity data
        twist_x_ = odom_msg.twist.twist.linear.x;
        twist_y_ = odom_msg.twist.twist.linear.y;
        // Apply low-pass filter to linear xy from odom
        linear_x_ = alpha_x * odom_msg.twist.twist.linear.x + (1 - alpha_x) * linear_x_;
        linear_y_ = alpha_y * odom_msg.twist.twist.linear.y + (1 - alpha_y) * linear_y_;
        linear_x_cov_ = std::min(linear_cov_max_, odom_msg.twist.covariance[0]);
        linear_y_cov_ = std::min(linear_cov_max_, odom_msg.twist.covariance[7]);
    }

    void imuCallback(const sensor_msgs::msg::Imu & imu_msg) {
        angular_z_ = imu_msg.angular_velocity.z;
        local_filter_pub(imu_msg.header.stamp, std::min(angular_cov_max_, imu_msg.angular_velocity_covariance[8])); //cov_max
        prev_stamp_ = imu_msg.header.stamp;
    }

    void local_filter_pub(rclcpp::Time stamp, double imu_cov)
    {
        // Publish global_filter message
        nav_msgs::msg::Odometry global_filter_msg;
        global_filter_msg.header.stamp = stamp; // imu callback stamp
        //velocity
            global_filter_msg.twist.twist.linear.x = linear_x_; //filtered x velocity
            global_filter_msg.twist.twist.linear.y = linear_y_;
            global_filter_msg.twist.twist.angular.z = angular_z_; //raw imu data
        //pose
            global_filter_msg.pose.pose.position.x = robotstate_.mu(0);
            global_filter_msg.pose.pose.position.y = robotstate_.mu(1);
            tf2::Quaternion quaternion_;
            quaternion_.setRPY( 0, 0, robotstate_.mu(2) );
            quaternion_ = quaternion_.normalize();
            global_filter_msg.pose.pose.orientation.z = quaternion_.getZ();
            global_filter_msg.pose.pose.orientation.w = quaternion_.getW();
        //covariance
            global_filter_msg.twist.covariance[0] = linear_x_cov_; //x-x
            global_filter_msg.twist.covariance[7] = linear_y_cov_; //y-y
            global_filter_msg.twist.covariance[35] = imu_cov; //theta-theta
            global_filter_pub_->publish(global_filter_msg);
    }

    double angleLimitChecking(double theta)
    {
        while (theta > M_PI)
        {
            theta -= M_PI * 2;
        }
        while (theta <= -M_PI)
        {
            theta += M_PI * 2;
        }
        return theta;
    }

private:
    std::shared_ptr<rclcpp::Node> nh_;
    std::shared_ptr<rclcpp::Node> nh_local_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr setpose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_filter_pub_;

    //raw
    double twist_x_;
    double twist_y_;
    //filtered
    double alpha_x; // filter coefficient
    double alpha_y; // filter coefficient
    double linear_x_; 
    double linear_x_cov_;
    double linear_y_;
    double linear_y_cov_;
    double angular_z_;
    // double imu_cov_;
    RobotState robotstate_;
    rclcpp::Time prev_stamp_;
    double linear_cov_max_;
    double angular_cov_max_;
};

int main(int argc, char** argv) {
    
    rclcpp::init(argc, argv);

    auto exec=std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("nh");
    std::shared_ptr<rclcpp::Node> nh_local = rclcpp::Node::make_shared("nh_local");
    GlobalFilterNode global_filter_node(nh, nh_local);

    exec->add_node(nh);
    exec->add_node(nh_local);
    exec->spin();

    rclcpp::shutdown();
    
    return 0;
}
