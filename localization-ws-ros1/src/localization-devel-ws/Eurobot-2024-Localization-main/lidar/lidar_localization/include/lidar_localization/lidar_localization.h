/**
 *
 * @file lidar_localization.h
 * @brief
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author sunfu.chou (sunfu.chou@gmail.com)
 * @version 0.1
 * @date 2021-05-02
 *
 */

#pragma once

#define _USE_MATH_DEFINES
#include <armadillo>
#include <cmath>
#include <vector>

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <lidar_localization/util/math_util.h>
#include <obstacle_detector/Obstacles.h>

// matrix calulation
#include <Eigen/Dense>
#include <math.h>

#define BEACON_NUMBER 3

namespace lidar_localization
{


/**
 * @struct ObstacleCircle
 * @brief for restore obstacle circles
 */
typedef struct ObstacleCircle 
{
  /* -- Obstacle center position to base_footprint -- */
  geometry_msgs::Point center;

  /* -- Obstacle radius -- */
  double radius;

  /* -- Obstacle Velocity -- */
  geometry_msgs::Vector3 velocity;

  /* -- Distance between beacons -- */
  double beacon_distance[BEACON_NUMBER];

} ObstacleCircle;



/**
 * @struct BeaconRef
 * @brief for restore beacons
 */
typedef struct BeaconRef 
{
  /* -- Real beacon center (obstacle) -- */
  geometry_msgs::Point real;

  /* -- Ideal beacon center (tf) -- */
  geometry_msgs::Point ideal;

  /* Min error */
  double beaconError;

} BeaconRef;



/**
 * @class LidarLocalization
 * @brief A class that use obstacles to localize robot position
 */
class LidarLocalization
{
public:
  /**
   * @brief Construct for the class `Lidar Localization`
   *
   * @param nh the global node handler
   * @param nh_local the local node handler, use for get params
   */
  LidarLocalization(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
  ~LidarLocalization();

private:
  /**
   * @brief To get params and set to this node in the constructor
   *
   */
  void initialize()
  {
    std_srvs::Empty empt;
    updateParams(empt.request, empt.response);
  }

  /**
   * @brief A service call that get params and set to this node
   *
   * @param req The service request
   * @param res The service response
   * @return true if the service call is succeeds,
   * @return false otherwise
   */
  bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * @brief Topic `cmd_vel` callback function
   *
   * @param ptr The command velocity data
   */
  void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& ptr);

  /**
   * @brief Topic `odom` callback function
   *
   * @param ptr The command velocity data
   */
  void odomCallback(const nav_msgs::Odometry::ConstPtr& ptr);

  /**
   * @brief Topic `obstacles` callback function
   *
   * @param ptr The obstaacles data
   */
  void obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr);

  /**
   * @brief Topic `obstacles` callback function
   *
   * @param ptr The obstaacles data
   */
  void ekfposeCallback(const nav_msgs::Odometry::ConstPtr& ptr);

  /**
   * @brief Topic `lidar_pose` publisher function
   *
   */
  void publishLocation();

  /**
   * @brief Tf broadcasters that send three fixed beacons pose to **map**
   *
   */
  void publishBeacons();

  /**
   * @brief A blocking function to check if Tf is ok
   *
   * @return true if all tf is ok
   * @return false never, instead, stuck in while loop
   */
  bool checkTFOK();

  /**
   * @brief Send tf of the beacons to **map**
   *
   */
  void setBeacontoMap();

  /**
   * @brief To loopup tf of beacons to **map**
   *
   */
  void getBeacontoMap();

  /**
   * @brief To loopup tf of beacons to **robot**
   *
   */
  void getBeacontoRobot();

  /**
   * @brief To find the nearest obstacles to beacon tf pose
   *
   */
  void findBeacon();

  /**
   * @brief To check the geometry of three obstacles is satisfy tf geometry
   *
   * @return true if geometry is valid
   * @return false if at least one geometry is wrong
   */
  bool validateBeaconGeometry();

  /**
   * @brief To get robot pose by beacon
   *
   */
  void getRobotPose();

  /* Get beacon prediction function */
  void updateBeacons();

  /* ros node */
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer params_srv_;

  /* ros inter-node */
  ros::Subscriber sub_obstacles_;
  // ros::Subscriber sub_toposition_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_ekfpose_;
  ros::Publisher pub_location_;
  ros::Publisher pub_beacon_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  std::vector<ObstacleCircle> realtime_circles_;
  geometry_msgs::PoseWithCovarianceStamped output_robot_pose_;
  geometry_msgs::PoseArray output_beacons_;

  /* private variables */
  geometry_msgs::Point beacon_to_map_[BEACON_NUMBER];
  BeaconRef beacons_[BEACON_NUMBER];
  BeaconRef predict_beacons_[BEACON_NUMBER];

  geometry_msgs::Pose ekf_pose_;

  geometry_msgs::Point robot_to_map_vel_;
  geometry_msgs::Point robot_to_map_vel_pre;

  bool failed_tf_;

  /* ros param */
  bool p_active_;

  double p_cov_x_;
  double p_cov_y_;
  double p_cov_yaw_;
  double p_beacon_1_x_;
  double p_beacon_1_y_;
  double p_beacon_2_x_;
  double p_beacon_2_y_;
  double p_beacon_3_x_;
  double p_beacon_3_y_;
  double p_theta_;

  double p_beacon_tolerance_;

  double p_predict_magnification_;

  double p_threshold_;
  double p_cov_dec_;

  std::string p_obstacle_topic_;
  // std::string p_toposition_topic_;
  std::string p_odom_topic_;
  std::string p_beacon_parent_frame_id_;
  std::string p_predict_frame_id_;
  std::string p_beacon_frame_id_prefix_;
  std::string p_robot_parent_frame_id_;
  std::string p_robot_frame_id_;
  std::string p_ekfpose_topic_;
  int chassis_type;

  ros::Time stamp_get_obs;

  //ready process
  ros::Subscriber ready_sub;
  void readyCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  bool ready_signal_active;
};
}  // namespace lidar_localization
