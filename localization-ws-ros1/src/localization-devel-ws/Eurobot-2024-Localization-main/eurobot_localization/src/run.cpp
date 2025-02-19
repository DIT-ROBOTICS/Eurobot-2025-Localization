#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Run {
public:
    Run(ros::NodeHandle& nh, ros::NodeHandle& nh_);
private:
    void pub_vel(double x, double y, double w);
    void timerCallback(const ros::TimerEvent&);

    ros::Publisher vel_pub;
    geometry_msgs::Twist cmd;
    ros::Timer timer;
    std::string robot_name;
    double freq;
    double acc;
    double max_vel;
    double max_dis;
    int mode; /* 0: linear, 1: rotation */
};

Run::Run(ros::NodeHandle& nh, ros::NodeHandle& nh_) {
    bool ok = true;
    ok &= nh_.getParam("robot_name", robot_name);
    ok &= nh_.getParam("frequency", freq);
    ok &= nh_.getParam("acceleration", acc);
    ok &= nh_.getParam("max_velocity", max_vel);
    ok &= nh_.getParam("max_distance", max_dis);
    ok &= nh_.getParam("mode", mode);

    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    timer = nh.createTimer(ros::Duration(1.0 / freq), &Run::timerCallback, this);
}

void Run::pub_vel(double x, double y, double w) {
    geometry_msgs::Twist msg;
    msg.linear.x = x;
    msg.linear.y = y;
    msg.angular.z = w;
    vel_pub.publish(msg);
}

void Run::timerCallback(const ros::TimerEvent& event) {
    static double count = 0;
    static double odom = 0;
    double out = 0;

    out = acc * count;
    if(out > max_vel) out = max_vel;

    odom += double(out / freq);
    if(odom <= max_dis){
        if(mode == 0) pub_vel(out, 0, 0);
        if(mode == 1) pub_vel(0, out, 0);
        if(mode == 2) pub_vel(0, 0, out);
    } 
    else{
        pub_vel(0, 0, 0);
    } 
    
    count += double(1.0 / freq);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "run");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_local("~");

    Run run(nh, nh_local);

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}