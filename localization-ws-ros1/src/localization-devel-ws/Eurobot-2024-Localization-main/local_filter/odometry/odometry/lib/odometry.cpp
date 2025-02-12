#include "odometry/odometry.h"

Odometry::Odometry (ros::NodeHandle &nh, ros::NodeHandle &nh_local){

    this->nh_ = nh;
    this->nh_local_ = nh_local;
    this->Initialize();
}

void Odometry::Initialize(){

    std_srvs::Empty empty;
	
    this->p_active_ = false;
    ROS_INFO_STREAM("[Odometry] : inactive node");

    if(this->UpdateParams(empty.request, empty.response)){
        ROS_INFO_STREAM("[Odometry] : Initialize param ok");
    }
    else {
        ROS_INFO_STREAM("[Odometry] : Initialize param failed");    
    } 

}

bool Odometry::UpdateParams(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

    bool prev_active = p_active_;

    double covariance[6];
    double multi_covariance[3];

    /* get param */
    if(this->nh_local_.param<bool>("active", p_active_, true)){
        ROS_INFO_STREAM("[Odometry] : active set to " << p_active_);
    }

    if(this->nh_local_.param<bool>("publish", p_publish_, true)){
        ROS_INFO_STREAM("[Odometry] : publish set to " << p_publish_);
    }

    if(this->nh_local_.param<std::string>("twist_topic", p_twist_topic_, "/Toposition")){
        ROS_INFO_STREAM("[Odometry] : Current subscribe topic [ " << p_twist_topic_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("odom_topic", p_odom_topic_, "/odom")){
        ROS_INFO_STREAM("[Odometry] : Current publish topic [ " << p_odom_topic_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("fixed_frame", p_fixed_frame_, "odom")){
        ROS_INFO_STREAM("[Odometry] : Current fixed frame [ " << p_fixed_frame_ << " ]"); 
    }

    if(this->nh_local_.param<std::string>("target_frame", p_target_frame_, "base_footprint")){
        ROS_INFO_STREAM("[Odometry] : Current target frame [ " << p_target_frame_ << " ]"); 
    }

    if(this->nh_local_.param<bool>("update_params", p_update_params_, false)){
        ROS_INFO_STREAM("[Odometry] : update params set to " << p_update_params_); 
    }
    
    if(this->nh_local_.param<double>("covariance_x", covariance[0], 0.)){
        ROS_INFO_STREAM("[Odometry] : x covariance set to " << covariance[0]);
        this->odometry_output_.twist.covariance[0] = covariance[0];
    }

    if(this->nh_local_.param<double>("covariance_y", covariance[1], 0.)){
        ROS_INFO_STREAM("[Odometry] : y covariance set to " << covariance[1]);
        this->odometry_output_.twist.covariance[7] = covariance[1];
    }

    if(this->nh_local_.param<double>("covariance_z", covariance[2], 0.)){
        ROS_INFO_STREAM("[Odometry] : z covariance set to " << covariance[2]); 
        this->odometry_output_.twist.covariance[14] = covariance[2];
    }

    if(this->nh_local_.param<double>("covariance_vx", covariance[3], 0.)){
        ROS_INFO_STREAM("[Odometry] : vx covariance set to " << covariance[3]); 
        this->odometry_output_.twist.covariance[21] = covariance[3];
    }

    if(this->nh_local_.param<double>("covariance_vy", covariance[4], 0.)){
        ROS_INFO_STREAM("[Odometry] : vy covariance set to " << covariance[4]); 
        this->odometry_output_.twist.covariance[28] = covariance[4];
    }

    if(this->nh_local_.param<double>("covariance_vz", covariance[5], 0.)){
        ROS_INFO_STREAM("[Odometry] : vz covariance set to " << covariance[5]); 
        this->odometry_output_.twist.covariance[35] = covariance[5];
    }

    if(this->nh_local_.param<double>("covariance_multi_vx", multi_covariance[0], 0.)){
        ROS_INFO_STREAM("[Odometry] : vx covariance multiplicant set to " << multi_covariance[0]); 
        covariance_multi_[0] = multi_covariance[0];
    }

    if(this->nh_local_.param<double>("covariance_multi_vy", multi_covariance[1], 0.)){
        ROS_INFO_STREAM("[Odometry] : vy covariance multiplicant set to " << multi_covariance[1]); 
        covariance_multi_[1] = multi_covariance[1];
    }

    if(this->nh_local_.param<double>("covariance_multi_vz", multi_covariance[2], 0.)){
        ROS_INFO_STREAM("[Odometry] : vz covariance multiplicant set to " << multi_covariance[2]); 
        covariance_multi_[2] = multi_covariance[2];
    }

    if(this->nh_local_.param<bool>("using_nav_vel_cb", p_sub_from_nav_, 0.)){
        ROS_INFO_STREAM("[Odometry] : current subscribe from nav cmd_vel is set to " << p_sub_from_nav_); 
	}

    if(this->nh_local_.param<bool>("using_dynamic_reconf", p_use_dynamic_reconf_, true)){
        ROS_INFO_STREAM("[Odometry] : using dynamic reconfigure is set to " << p_use_dynamic_reconf_); 
	}

    if(this->nh_local_.param<bool>("use_stm_integral", p_use_stm_integral_, true)){
        ROS_INFO_STREAM("[Odometry] : use stm integral " << p_use_stm_integral_); 
    }

    if(p_active_ != prev_active) {

        if (p_active_) {

            ROS_INFO_STREAM("[Odometry] : active node");
            this->twist_sub_ = nh_.subscribe(p_twist_topic_, 10, &Odometry::TwistCallback, this);
            this->odom_pub_ = nh_.advertise<nav_msgs::Odometry>(p_odom_topic_, 10);

            if(this->p_sub_from_nav_){
                this->vel_sub_ = nh_.subscribe("cmd_vel", 10, &Odometry::P_VelocityCallback, this);
            }
            else{
                this->vel_sub_ = nh_.subscribe("local_filter", 10, &Odometry::VelocityCallback, this);
            }

            if(this->p_update_params_){
                this->param_srv_ = nh_local_.advertiseService("params", &Odometry::UpdateParams, this);
            }

			if(this->p_use_dynamic_reconf_){
				this->SetDynamicReconfigure();
			}
			
        }
        else {
            this->twist_sub_.shutdown();
            this->odom_pub_.shutdown();
			this->vel_sub_.shutdown();

            if(this->p_update_params_){
                this->param_srv_.shutdown();
            }
        }
    }


    /* -- Backup covariance -- */
    this->odometry_output_backup_ = this->odometry_output_;
		
	/* -- Set basic variables -- */
    this->odometry_output_.header.frame_id = this->p_fixed_frame_;
    this->odometry_output_.child_frame_id = this->p_target_frame_;

    return true;

}

void Odometry::TwistCallback(const geometry_msgs::Twist::ConstPtr &msg){

    static unsigned int sequence = 0;
    sequence++;

    this->odometry_output_.header.seq = sequence;
    this->odometry_output_.header.stamp = ros::Time::now();
    
    if(p_use_stm_integral_){
        this->odometry_output_.pose.pose.position.x = msg->angular.x;                                                                           
        this->odometry_output_.pose.pose.position.y = msg->angular.y;                                                                           
                                                                                                                                            
        tf2::Quaternion quaternion;                                                                                                             
        quaternion.setRPY(0, 0, msg->linear.z);                                                                                                 

        this->odometry_output_.pose.pose.orientation = tf2::toMsg(quaternion);
    }

    this->odometry_output_.twist.twist.linear.x = msg->linear.x;
    this->odometry_output_.twist.twist.linear.y = msg->linear.y;
    this->odometry_output_.twist.twist.angular.z = msg->angular.z;
    this->odometry_output_.pose.covariance[0] = 0.1;
    this->odometry_output_.pose.covariance[7] = 0.1;
    this->odometry_output_.pose.covariance[14] = 0.1;
    this->odometry_output_.pose.covariance[21] = 0.1;
    this->odometry_output_.pose.covariance[28] = 0.1;
    this->odometry_output_.pose.covariance[35] = 0.1;

    if(this->p_publish_) this->publish();

}


void Odometry::P_VelocityCallback(const nav_msgs::Odometry::ConstPtr &msg){

	double covariance_multi[3];
	
	covariance_multi[0] = this->covariance_multi_[0] * abs(msg->twist.twist.linear.x); 
	covariance_multi[1] = this->covariance_multi_[1] * abs(msg->twist.twist.linear.y); 
	covariance_multi[2] = this->covariance_multi_[2] * abs(msg->twist.twist.angular.z); 

	this->odometry_output_.twist.covariance[0] = std::max(0.00000001, covariance_multi[0] + this->odometry_output_backup_.twist.covariance[0]);
	this->odometry_output_.twist.covariance[7] = std::max(0.00000001, covariance_multi[1] + this->odometry_output_backup_.twist.covariance[7]);
	this->odometry_output_.twist.covariance[14] = std::max(0.00000001, covariance_multi[2]+ this->odometry_output_backup_.twist.covariance[14]);
	this->odometry_output_.twist.covariance[21] = std::max(0.00000001, covariance_multi[0] + this->odometry_output_backup_.twist.covariance[21]);
	this->odometry_output_.twist.covariance[28] = std::max(0.00000001, covariance_multi[1] + this->odometry_output_backup_.twist.covariance[28]);
	this->odometry_output_.twist.covariance[35] = std::max(0.00000001, covariance_multi[2] + this->odometry_output_backup_.twist.covariance[35]);

}


void Odometry::VelocityCallback(const nav_msgs::Odometry::ConstPtr &msg){

	boost::shared_ptr<nav_msgs::Odometry> odometry_ptr(new nav_msgs::Odometry());

	odometry_ptr->twist.twist.linear = msg->twist.twist.linear;
	odometry_ptr->twist.twist.angular = msg->twist.twist.angular;

	this->P_VelocityCallback(odometry_ptr);

}

void Odometry::publish(){

    this->odom_pub_.publish(this->odometry_output_);	

}

void Odometry::DynamicParamCallback(odometry::odometry_paramConfig &config, uint32_t level){

    /* get param */
    if(p_publish_ != config.publish){
		this->p_publish_ = config.publish;
        ROS_INFO_STREAM("[Odometry] : publish set to " << p_publish_);
    }

    if(p_twist_topic_ != config.twist_topic){

		this->p_twist_topic_ = config.twist_topic;

		if(p_active_){
            this->twist_sub_ = nh_.subscribe(p_twist_topic_, 10, &Odometry::TwistCallback, this);
		}

        ROS_INFO_STREAM("[Odometry] : Current subscribe topic [ " << p_twist_topic_ << " ]"); 
    }

    if(p_odom_topic_ != config.odom_topic){

		this->p_odom_topic_ = config.odom_topic;

		if(p_active_){
            this->odom_pub_ = nh_.advertise<nav_msgs::Odometry>(p_odom_topic_, 10);
		}

        ROS_INFO_STREAM("[Odometry] : Current publish topic [ " << p_odom_topic_ << " ]"); 
    }

    if(p_fixed_frame_ != config.fixed_frame){

		this->p_fixed_frame_ = config.fixed_frame;
		this->odometry_output_.header.frame_id = this->p_fixed_frame_;

        ROS_INFO_STREAM("[Odometry] : Current fixed frame [ " << p_fixed_frame_ << " ]"); 
    }

    if(p_target_frame_ != config.target_frame){

		this->p_target_frame_ = config.target_frame;
		this->odometry_output_.child_frame_id = this->p_target_frame_;

        ROS_INFO_STREAM("[Odometry] : Current target frame [ " << p_target_frame_ << " ]"); 
    }

    if(this->odometry_output_backup_.twist.covariance[0] != config.covariance_x){

        this->odometry_output_backup_.twist.covariance[0] = config.covariance_x;

        ROS_INFO_STREAM("[Odometry] : x covariance set to " << this->odometry_output_backup_.twist.covariance[0]);
    }

    if(this->odometry_output_backup_.twist.covariance[7] != config.covariance_y){

        this->odometry_output_backup_.twist.covariance[7] = config.covariance_y;

        ROS_INFO_STREAM("[Odometry] : y covariance set to " << this->odometry_output_backup_.twist.covariance[7]);
    }

    if(this->odometry_output_backup_.twist.covariance[14] != config.covariance_z){

        this->odometry_output_backup_.twist.covariance[14] = config.covariance_z;

        ROS_INFO_STREAM("[Odometry] : z covariance set to " << this->odometry_output_backup_.twist.covariance[14]);
    }

    if(this->odometry_output_backup_.twist.covariance[21] != config.covariance_vx){

        this->odometry_output_backup_.twist.covariance[21] = config.covariance_vx;

        ROS_INFO_STREAM("[Odometry] : vx covariance set to " << this->odometry_output_backup_.twist.covariance[21]);
    }
	
    if(this->odometry_output_backup_.twist.covariance[28] != config.covariance_vy){

        this->odometry_output_backup_.twist.covariance[28] = config.covariance_vy;

        ROS_INFO_STREAM("[Odometry] : vy covariance set to " << this->odometry_output_backup_.twist.covariance[28]);
    }

    if(this->odometry_output_backup_.twist.covariance[35] != config.covariance_vz){

        this->odometry_output_backup_.twist.covariance[35] = config.covariance_vz;

        ROS_INFO_STREAM("[Odometry] : vz covariance set to " << this->odometry_output_backup_.twist.covariance[35]);
    }

    if(this->covariance_multi_[0] != config.covariance_multi_vx){

        this->covariance_multi_[0] = config.covariance_multi_vx;

        ROS_INFO_STREAM("[Odometry] : vx covariance multiplicant set to " << this->covariance_multi_[0]);
    }

    if(this->covariance_multi_[1] != config.covariance_multi_vy){

        this->covariance_multi_[1] = config.covariance_multi_vy;

        ROS_INFO_STREAM("[Odometry] : vy covariance multiplicant set to " << this->covariance_multi_[1]);
    }

    if(this->covariance_multi_[2] != config.covariance_multi_vz){

        this->covariance_multi_[2] = config.covariance_multi_vz;

        ROS_INFO_STREAM("[Odometry] : vz covariance multiplicant set to " << this->covariance_multi_[2]);
    }
    
    this->odometry_output_.twist.covariance = this->odometry_output_backup_.twist.covariance;

}

void Odometry::SetDynamicReconfigure(){

	static dynamic_reconfigure::Server<odometry::odometry_paramConfig> dynamic_param_srv_;

	dynamic_reconfigure::Server<odometry::odometry_paramConfig>::CallbackType callback;

    // If the function is a class member : 
    // boost::bind(&function, class instance, _1, _2)
    callback = boost::bind(&Odometry::DynamicParamCallback, this, _1, _2);

    // Set callback function to param server
    dynamic_param_srv_.setCallback(callback);
}
