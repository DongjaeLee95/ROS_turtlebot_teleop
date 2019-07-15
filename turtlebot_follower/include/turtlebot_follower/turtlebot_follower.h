#pragma once
#include <ros/ros.h>
#include <vector>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
//#include <tf/transform_listener.h>

struct XYpsi{
    double x;
    double y;
    double psi;
    double x_dot;
    double y_dot;
    double psi_dot;
};

class turtlebot_follower
{

public:
    turtlebot_follower();
    ~turtlebot_follower();

    std::string tracker_name, target_name;
    geometry_msgs::Pose tracker_pose, target_pose;
   // geometry_msgs::Twist tracker_twist, target_twist;

    XYpsi target, tracker;

    int freq;
    double ang_err, ang_err_old;
    double dist_err, dist_err_old;
    //double target_x, target_y, target_psi;
    //double tracker_x, tracker_y, tracker_psi;

    //double K_x_p, K_y_p, K_psi_p, K_x_d, K_y_d, K_psi_d; // PD control gain

    double Kl_p, Kl_d, Ka_p, Ka_d;

    geometry_msgs::Twist controller();
//    void tfFunction();
    void PublishFunction();
    void VariConversion();

protected:
    void ModelStateCallBack(const gazebo_msgs::ModelStates::ConstPtr& gazebo_msg);

private:
    ros::Subscriber state_sub;
    ros::Publisher vel_pub;
    ros::NodeHandle nh_;
//   tf::TransformListener listener;
//   tf::StampedTransform transform;
};
