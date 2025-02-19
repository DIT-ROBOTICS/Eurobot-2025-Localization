/**
 *
 * @file lidar_localization.cpp
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
 * @date 2021-05-11
 *
 */

#include "lidar_localization/lidar_localization.h"

using namespace std;
using namespace lidar_localization;
using namespace arma;

LidarLocalization::LidarLocalization(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
  : nh_(nh), nh_local_(nh_local), tf2_listener_(tf2_buffer_)
{
  params_srv_ = nh_local_.advertiseService("params", &LidarLocalization::updateParams, this);
  initialize();
}

LidarLocalization::~LidarLocalization()
{
  nh_local_.deleteParam("active");

  nh_local_.deleteParam("cov_x");
  nh_local_.deleteParam("cov_y");
  nh_local_.deleteParam("cov_yaw");
  // nh_local_.deleteParam("beacon_ax");
  // nh_local_.deleteParam("beacon_ay");
  // nh_local_.deleteParam("beacon_bx");
  // nh_local_.deleteParam("beacon_by");
  // nh_local_.deleteParam("beacon_cx");
  // nh_local_.deleteParam("beacon_cy");
  nh_local_.deleteParam("theta");

  nh_local_.deleteParam("beacon_tolerance");
  nh_local_.deleteParam("threshold");
  nh_local_.deleteParam("cov_dec");

  nh_local_.deleteParam("obstacle_topic");
  // nh_local_.deleteParam("toposition_topic");
  nh_local_.deleteParam("odom_topic");
  nh_local_.deleteParam("ekfpose_topic");

  nh_local_.deleteParam("beacon_parent_frame_id");
  nh_local_.deleteParam("robot_parent_frame_id");
  nh_local_.deleteParam("robot_frame_id");

  nh_local_.deleteParam("beacon_frame_id_prefix");
}

bool LidarLocalization::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  bool get_param_ok = true;
  bool prev_active = p_active_;

  get_param_ok &= nh_local_.param<bool>("active", p_active_, true);

  get_param_ok &= nh_local_.param<double>("cov_x", p_cov_x_, 1e-1);
  get_param_ok &= nh_local_.param<double>("cov_y", p_cov_y_, 1e-1);
  get_param_ok &= nh_local_.param<double>("cov_yaw", p_cov_yaw_, 1e-1);
  get_param_ok &= nh_local_.param<double>("theta", p_theta_, 0);

  get_param_ok &= nh_local_.param<double>("beacon_tolerance", p_beacon_tolerance_, 0.13);
  get_param_ok &= nh_local_.param<double>("threshold", p_threshold_, 0.24);
  get_param_ok &= nh_local_.param<double>("cov_dec", p_cov_dec_, 0.01);
  get_param_ok &= nh_local_.param<double>("predict_magnification", p_predict_magnification_, 0.1);

  get_param_ok &= nh_local_.param<string>("obstacle_topic", p_obstacle_topic_, "obstacles");
  // get_param_ok &= nh_local_.param<string>("toposition_topic", p_toposition_topic_, "Toposition");
  get_param_ok &= nh_local_.param<string>("odom_topic", p_odom_topic_, "odom");
  get_param_ok &= nh_local_.param<string>("ekfpose_topic", p_ekfpose_topic_, "ekf_pose");

  get_param_ok &= nh_local_.param<string>("beacon_parent_frame_id", p_beacon_parent_frame_id_, "map");
  get_param_ok &= nh_local_.param<string>("robot_parent_frame_id", p_robot_parent_frame_id_, "map");
  get_param_ok &= nh_local_.param<string>("robot_frame_id", p_robot_frame_id_, "base_footprint");
  get_param_ok &= nh_local_.param<string>("beacon_predict_frame_id", p_predict_frame_id_, "predict");

  get_param_ok &= nh_local_.param<string>("beacon_frame_id_prefix", p_beacon_frame_id_prefix_, "beacon");

  if (p_active_ != prev_active)
  {
    if (p_active_)
    {
      sub_obstacles_ = nh_.subscribe(p_obstacle_topic_, 10, &LidarLocalization::obstacleCallback, this);
      // sub_toposition_ = nh_.subscribe(p_toposition_topic_, 10, &LidarLocalization::cmdvelCallback, this);
      sub_odom_ = nh_.subscribe(p_odom_topic_, 10, &LidarLocalization::odomCallback, this);
      sub_ekfpose_ = nh_.subscribe(p_ekfpose_topic_, 10, &LidarLocalization::ekfposeCallback, this);
      pub_location_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar_bonbonbon", 10);
      pub_beacon_ = nh_.advertise<geometry_msgs::PoseArray>("beacons", 10);
    }
    else
    {
      sub_obstacles_.shutdown();
      pub_location_.shutdown();
      pub_beacon_.shutdown();
    }
  }

  get_param_ok &= nh_.param<bool>("localization/ready_signal_active", ready_signal_active, false);
  if(ready_signal_active){
      ready_sub = nh_.subscribe("startup/ready_signal", 10, &LidarLocalization::readyCallback, this);
  }else{
    geometry_msgs::PointStamped initial_msg;
    get_param_ok &= nh_.param<string>("localization/side", initial_msg.header.frame_id, "0");
    get_param_ok &= nh_.param<double>("localization/initial_pose/x", initial_msg.point.x, 0.3);
    get_param_ok &= nh_.param<double>("localization/initial_pose/y", initial_msg.point.y, 1.5);
    get_param_ok &= nh_.param<double>("localization/initial_pose/z", initial_msg.point.z, 0.0);
    
    std::string side;
    if(strcmp(initial_msg.header.frame_id.c_str(), "0") == 0) side = "Blue";
    else if(strcmp(initial_msg.header.frame_id.c_str(), "1") == 0) side = "Yellow";
    
    get_param_ok &= nh_.param<double>(side + "/beacon_ax", p_beacon_1_x_, -0.094);
    get_param_ok &= nh_.param<double>(side + "/beacon_ay", p_beacon_1_y_, 0.052);
    get_param_ok &= nh_.param<double>(side + "/beacon_bx", p_beacon_2_x_, -0.094);
    get_param_ok &= nh_.param<double>(side + "/beacon_by", p_beacon_2_y_, 1.948);
    get_param_ok &= nh_.param<double>(side + "/beacon_cx", p_beacon_3_x_, 3.094);
    get_param_ok &= nh_.param<double>(side + "/beacon_cy", p_beacon_3_y_, 1.0);
    
    /* Setup beacon position for triangular localization */
    setBeacontoMap();
    checkTFOK();
    getBeacontoMap();

  }

  if (get_param_ok)
  {
    // ROS_INFO_STREAM("[Lidar Localization]: "
    //                 << "set param ok");
  }
  else
  {
    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "set param failed");
  }

  return true;
}

void LidarLocalization::readyCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
  static bool received_ready = false;
  if(received_ready) return;
  std::string side = msg->header.frame_id;
  if(strcmp(side.c_str(), "0") == 0) side = "Blue";
  else if(strcmp(side.c_str(), "1") == 0) side = "Yellow";

  bool ok = true;
  ok &= nh_.param<double>(side + "/beacon_ax", p_beacon_1_x_, -0.094);
  ok &= nh_.param<double>(side + "/beacon_ay", p_beacon_1_y_, 0.052);
  ok &= nh_.param<double>(side + "/beacon_bx", p_beacon_2_x_, -0.094);
  ok &= nh_.param<double>(side + "/beacon_by", p_beacon_2_y_, 1.948);
  ok &= nh_.param<double>(side + "/beacon_cx", p_beacon_3_x_, 3.094);
  ok &= nh_.param<double>(side + "/beacon_cy", p_beacon_3_y_, 1.0);

  /* Setup beacon position for triangular localization */
  setBeacontoMap();
  checkTFOK();
  getBeacontoMap();

  if(!ok) ROS_WARN("[Lidar Localization]: ready failed");
  received_ready = true;

}

// void LidarLocalization::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& ptr)
// {
//   robot_to_map_vel_.x = ptr->linear.x;
//   robot_to_map_vel_.y = ptr->linear.y;
//   robot_to_map_vel_.z = ptr->angular.z;
// }

void LidarLocalization::odomCallback(const nav_msgs::Odometry::ConstPtr& ptr)
{
  robot_to_map_vel_.x = ptr->twist.twist.linear.x;
  robot_to_map_vel_.y = ptr->twist.twist.linear.y;
  robot_to_map_vel_.z = ptr->twist.twist.angular.z;
}

void LidarLocalization::ekfposeCallback(const nav_msgs::Odometry::ConstPtr& ptr)
{
  this->ekf_pose_ = ptr->pose.pose;
}

/* MAIN */
void LidarLocalization::obstacleCallback(const obstacle_detector::Obstacles::ConstPtr& ptr)
{
  stamp_get_obs = ptr->header.stamp;

  /* Remove previous obstacle circles */
  realtime_circles_.clear();

  /* Transform beacon to robot frame */
  getBeacontoRobot();

  /* Get update prediction beacons */
  updateBeacons();

  /* Restore the obstacle circles */
  for (const obstacle_detector::CircleObstacle& obstacle : ptr->circles)
  {
    ObstacleCircle obstaclecircle;
    obstaclecircle.center = obstacle.center;
    obstaclecircle.radius = obstacle.true_radius;
    obstaclecircle.velocity = obstacle.velocity;
    for (int i = 0 ; i < 3 ; i++)
    {
      obstaclecircle.beacon_distance[i] = length(obstaclecircle.center, predict_beacons_[i].ideal);
    }
    realtime_circles_.push_back(obstaclecircle);
  }

  findBeacon();
  getRobotPose();
}

void LidarLocalization::updateBeacons()
{
  /* First get ekf_pose + cmd_vel */
  /* And broadcast the pose to map frame */
  geometry_msgs::TransformStamped transform;
  ros::Time now = ros::Time::now();
  // transform.header.stamp = now;
  transform.header.stamp = stamp_get_obs;
  transform.header.frame_id = p_robot_parent_frame_id_;
  transform.child_frame_id = p_predict_frame_id_;

  transform.transform.translation.x = ekf_pose_.position.x;
  transform.transform.translation.y = ekf_pose_.position.y;
  transform.transform.translation.z = 0;

  tf2::Quaternion q;
  tf2::fromMsg(ekf_pose_.orientation, q);
  tf2::Matrix3x3 qt(q);
  double _, yaw;
  qt.getRPY(_, _, yaw);

  yaw += robot_to_map_vel_.z * p_predict_magnification_;
  tf2::Quaternion yaw_quaternion;
  yaw_quaternion.setRPY(0, 0, yaw);

  transform.transform.rotation.x = yaw_quaternion.getX();
  transform.transform.rotation.y = yaw_quaternion.getY();
  transform.transform.rotation.z = yaw_quaternion.getZ();
  transform.transform.rotation.w = yaw_quaternion.getW();

  failed_tf_ = !(ekf_pose_.orientation.x + ekf_pose_.orientation.y + ekf_pose_.orientation.z + ekf_pose_.orientation.w);

  if(!failed_tf_) static_broadcaster_.sendTransform(transform);
  else return;

  /* Get beacon transform from the tf to map */
  bool tf_ok = true;
  for (int i = 1; i <= 3; ++i)
  {
    try
    {
      transform = tf2_buffer_.lookupTransform(p_predict_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i), ros::Time());

      predict_beacons_[i - 1].ideal.x = transform.transform.translation.x;
      predict_beacons_[i - 1].ideal.y = transform.transform.translation.y;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM(ex.what());
      tf_ok = false;
    }
  }

  if (tf_ok)
  {
    // ROS_INFO_STREAM("[Lidar Localization]: " << "get beacon to map tf ok");
  }
  else
  {
    ROS_WARN_STREAM("[Lidar Localization]: " << "get beacon to map tf failed");
  }
}

void LidarLocalization::setBeacontoMap()
{
  geometry_msgs::TransformStamped transform;
  ros::Time now = ros::Time::now();
  transform.header.stamp = now;
  transform.header.frame_id = p_beacon_parent_frame_id_;

  transform.transform.translation.z = 0;
  transform.transform.rotation.x = 0;
  transform.transform.rotation.y = 0;
  transform.transform.rotation.z = 0;
  transform.transform.rotation.w = 1;

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "1";
  transform.transform.translation.x = p_beacon_1_x_;
  transform.transform.translation.y = p_beacon_1_y_;
  static_broadcaster_.sendTransform(transform);

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "2";
  transform.transform.translation.x = p_beacon_2_x_;
  transform.transform.translation.y = p_beacon_2_y_;
  static_broadcaster_.sendTransform(transform);

  transform.child_frame_id = p_beacon_frame_id_prefix_ + "3";
  transform.transform.translation.x = p_beacon_3_x_;
  transform.transform.translation.y = p_beacon_3_y_;
  static_broadcaster_.sendTransform(transform);

//   ROS_INFO_STREAM("[Lidar Localization]: "
//                   << "set beacon tf ok");
}

bool LidarLocalization::checkTFOK()
{
  int tf_retry_count = 0;
  while (ros::ok())
  {
    ++tf_retry_count;
    ros::Duration(0.5).sleep();

    bool tf_ok = true;
    for (int i = 1; i <= 3; i++)
    {
      if (!tf2_buffer_.canTransform(p_beacon_parent_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i),
                                    ros::Time()))
      {
        tf_ok = false;
      }
      if (!tf2_buffer_.canTransform(p_robot_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i), ros::Time()))
      {
        tf_ok = false;
      }
    }

    if (tf_ok)
      return true;

    ROS_WARN_STREAM("[Lidar Localization]: "
                    << "tf not ok");

    if (tf_retry_count % 20 == 0)
    {
      ROS_ERROR_STREAM("[Lidar Localization]: "
                       << "tf error after retry " << tf_retry_count << " times");
    }
  }

  return false;
}

void LidarLocalization::getBeacontoMap()
{
  bool tf_ok = true;
  geometry_msgs::TransformStamped transform;
  for (int i = 1; i <= 3; ++i)
  {
    try
    {
      transform = tf2_buffer_.lookupTransform(p_beacon_parent_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i), ros::Time());

      beacon_to_map_[i - 1].x = transform.transform.translation.x;
      beacon_to_map_[i - 1].y = transform.transform.translation.y;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM(ex.what());
      tf_ok = false;
    }
  }

  if (tf_ok)
  {
    // ROS_INFO_STREAM("[Lidar Localization]: " << "get beacon to map tf ok");
  }
  else
  {
    ROS_WARN_STREAM("[Lidar Localization]: " << "get beacon to map tf failed");
  }
}

void LidarLocalization::getBeacontoRobot()
{
  bool tf_ok = true;
  geometry_msgs::TransformStamped transform;
  for (int i = 1; i <= 3; ++i)
  {
    try
    {
      transform = tf2_buffer_.lookupTransform(p_robot_frame_id_, p_beacon_frame_id_prefix_ + std::to_string(i), ros::Time());

      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;
      
      beacons_[i - 1].ideal.x = x;
      beacons_[i - 1].ideal.y = y;
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM(ex.what());
      tf_ok = false;
    }
  }

  if (!tf_ok)
  {
    ROS_WARN_STREAM("[Lidar Localization]: " << "get beacon to robot tf failed");
  }
}

void LidarLocalization::findBeacon()
{
  /* Iterate the three beacons */
  for (int i = 0; i < 3; ++i)
  {
    if (realtime_circles_.empty())
    {
      ROS_WARN_STREAM_THROTTLE(2, "[Lidar Localization]: " << "no obstacle information");
      continue;
    }

    /* Check the closest obstacle in all of the obstacles */
    double min_distance = realtime_circles_[0].beacon_distance[i];
    for (auto circle : realtime_circles_)
    {
      if (circle.beacon_distance[i] <= min_distance)
      {
        min_distance = circle.beacon_distance[i];
        beacons_[i].real.x = circle.center.x;
        beacons_[i].real.y = circle.center.y;
        beacons_[i].beaconError = min_distance;
      }
    }
  }
}

bool LidarLocalization::validateBeaconGeometry()
{
  double beacon_distance[3][3] = {};
  double real_beacon_distance[3][3] = {};
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      beacon_distance[i][j] = length(beacons_[i].real, beacons_[j].real);
    }
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      real_beacon_distance[i][j] = length(beacon_to_map_[i], beacon_to_map_[j]);
    }
  }

  double tolerance = p_beacon_tolerance_ + robot_to_map_vel_.z * 0.08;

  if (is_whthin_tolerance(beacon_distance[0][1], real_beacon_distance[0][1], tolerance) &&
      is_whthin_tolerance(beacon_distance[0][2], real_beacon_distance[0][2], tolerance) &&
      is_whthin_tolerance(beacon_distance[1][2], real_beacon_distance[1][2], tolerance))
  {
    return true;
  }
  else
  {
    // ROS_INFO_STREAM_THROTTLE(2, "[Lidar Localization] : current beacon tolerance " << tolerance);
    // ROS_INFO_STREAM_THROTTLE(2, "reacon distance: " << real_beacon_distance[0][1] << ", " << real_beacon_distance[0][2] << ", "
    //                                     << real_beacon_distance[1][2]);
    ROS_WARN_STREAM_THROTTLE(2, "beacon distance: " << beacon_distance[0][1] << ", " << beacon_distance[0][2] << ", "
                                        << beacon_distance[1][2]);
    ROS_WARN_STREAM_THROTTLE(2, "real_beacon distance: " << real_beacon_distance[0][1] << ", " << real_beacon_distance[0][2] << ", "
                                        << real_beacon_distance[1][2]);
    return false;
  }
}

void LidarLocalization::getRobotPose()
{
  if (!validateBeaconGeometry())
  {
    ROS_WARN_STREAM_THROTTLE(2, "geometry error");
    return;
  }

  vector<double> dist_beacon_robot;
  for (int i = 0; i < 3; ++i)
  {
    dist_beacon_robot.push_back(length(beacons_[i].real));
  }

  // least squares method to solve Ax=b
  // i.e to solve (A^T)Ax=(A^T)b
  mat A(2, 2);
  vec b(2);
  vec X(2);

  A(0, 0) = 2 * (beacon_to_map_[0].x - beacon_to_map_[2].x);
  A(0, 1) = 2 * (beacon_to_map_[0].y - beacon_to_map_[2].y);

  A(1, 0) = 2 * (beacon_to_map_[1].x - beacon_to_map_[2].x);
  A(1, 1) = 2 * (beacon_to_map_[1].y - beacon_to_map_[2].y);

  b(0) = (pow(beacon_to_map_[0].x, 2) - pow(beacon_to_map_[2].x, 2)) +
         (pow(beacon_to_map_[0].y, 2) - pow(beacon_to_map_[2].y, 2)) +
         (pow(dist_beacon_robot[2], 2) - pow(dist_beacon_robot[0], 2));
  b(1) = (pow(beacon_to_map_[1].x, 2) - pow(beacon_to_map_[2].x, 2)) +
         (pow(beacon_to_map_[1].y, 2) - pow(beacon_to_map_[2].y, 2)) +
         (pow(dist_beacon_robot[2], 2) - pow(dist_beacon_robot[1], 2));
  try
  {
    X = solve(A.t() * A, A.t() * b, solve_opts::no_approx);

    output_robot_pose_.pose.pose.position.x = X(0);
    output_robot_pose_.pose.pose.position.y = X(1);

    double robot_yaw = 0;
    double robot_sin = 0;
    double robot_cos = 0;

    for (int i = 0; i < 3; i++)
    {
      double theta = atan2(beacon_to_map_[i].y - output_robot_pose_.pose.pose.position.y,
                           beacon_to_map_[i].x - output_robot_pose_.pose.pose.position.x) -
                     atan2(beacons_[i].real.y, beacons_[i].real.x);

      robot_sin += sin(theta);
      robot_cos += cos(theta);
    }

    robot_yaw = atan2(robot_sin, robot_cos) + p_theta_ / 180.0 * 3.1415926;

    /*
      TODO: should know exactly how much the delay is
      known: lidar publish stamp, obstacle stamp, imu rotation, odom linear x and y
      better to also to with linear
      and then publish with ros::Time::now()
    */

    ros::Time now = ros::Time::now();
    ros::Duration delay = now - stamp_get_obs;

    // the robotstate calculated with obstacles, which is a few ms ago
    Eigen::Vector3d robotstate;
    robotstate(0) = X(0);
    robotstate(1) = X(1);
    robotstate(2) = robot_yaw; 

    /* prediction function for omni wheel */
    double v_x = robot_to_map_vel_.x;
    double v_y = robot_to_map_vel_.y;
    double w = robot_to_map_vel_.z;
    double dt = delay.toSec();
    
    Eigen::Vector3d d_state;

    d_state << (v_x * dt), (v_y * dt), (w * dt);

    double theta_ = robotstate(2) + d_state(2) / 2;
    double s__theta = sin(theta_);
    double c__theta = cos(theta_);

    Eigen::Matrix3d A;
    A << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    Eigen::Matrix3d B;
    B << c__theta, -s__theta, 0, s__theta, c__theta, 0, 0, 0, 1;

    robotstate = A * robotstate + B * d_state;

    output_robot_pose_.pose.pose.position.x = robotstate(0);
    output_robot_pose_.pose.pose.position.y = robotstate(1);

    tf2::Quaternion q;
    q.setRPY(0., 0., robot_yaw);
    output_robot_pose_.pose.pose.orientation = tf2::toMsg(q);

    publishLocation();
  }
  catch (const std::runtime_error& ex)
  {
    ROS_WARN_STREAM(A);
    ROS_WARN_STREAM(b);
    ROS_WARN_STREAM(ex.what());
  }
  // publishBeacons();
}

void LidarLocalization::publishLocation()
{
  ros::Time now = ros::Time::now();

  output_robot_pose_.header.frame_id = p_robot_parent_frame_id_;
  output_robot_pose_.header.stamp = now;
  // output_robot_pose_.header.stamp = stamp_get_obs;

  double error_length = length(output_robot_pose_.pose.pose.position, ekf_pose_.position);
  double cov_x = (error_length > p_threshold_) ? p_cov_x_ * p_cov_dec_ : p_cov_x_;
  double cov_y = (error_length > p_threshold_) ? p_cov_y_ * p_cov_dec_ : p_cov_y_;
  double cov_yaw = (error_length > p_threshold_) ? p_cov_yaw_ * p_cov_dec_ : p_cov_yaw_;

  double geometryError = 0;
  for(int i = 0 ; i < 3 ; i++) geometryError += beacons_[i].beaconError;

  /* TODO : debug and test */
  // ROS_WARN_STREAM_THROTTLE(2, "[Lidar localization] : current beacons error " << geometryError);

  // clang-format off
                                       // x         y         z  pitch roll yaw
  output_robot_pose_.pose.covariance = {cov_x,    0,        0, 0,    0,   0,
                                        0,        cov_y,    0, 0,    0,   0,
                                        0,        0,        0, 0,    0,   0,
                                        0,        0,        0, 0,    0,   0,
                                        0,        0,        0, 0,    0,   0,
                                        0,        0,        0, 0,    0,   cov_yaw};
  // clang-format on
  pub_location_.publish(output_robot_pose_);
}

void LidarLocalization::publishBeacons()
{
  ros::Time now = ros::Time::now();
  output_beacons_.header.stamp = now;
  output_beacons_.header.frame_id = p_robot_frame_id_;
  output_beacons_.poses.clear();

  for (int i = 0; i < 3; ++i)
  {
    geometry_msgs::Pose pose;
    pose.position.x = beacons_[i].real.x;
    pose.position.y = beacons_[i].real.y;
    output_beacons_.poses.push_back(pose);
  }

  pub_beacon_.publish(output_beacons_);
}
