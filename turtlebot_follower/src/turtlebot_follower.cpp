#include "turtlebot_follower/turtlebot_follower.h"
//#include <tf/tf.h>
#include <cmath>
//#include <tf/transform_datatypes.h>
turtlebot_follower::turtlebot_follower()
:nh_("~")
{

    ROS_INFO("turtlebot_follower Initiated");

    //default value
    ang_err = 0; ang_err_old = 0; dist_err = 0; dist_err_old = 0;

    nh_.getParam("turtle1_name", target_name);
    nh_.getParam("turtle2_name", tracker_name);
    nh_.getParam("overall_frequency", freq);
    nh_.getParam("gain_linear_p", Kl_p);
    nh_.getParam("gain_angular_p", Ka_p);
    nh_.getParam("gain_linear_d", Kl_d);
    nh_.getParam("gain_angular_d", Ka_d);

    state_sub = nh_.subscribe("/gazebo/model_states", freq, &turtlebot_follower::ModelStateCallBack, this);

}

turtlebot_follower::~turtlebot_follower()
{
    ros::shutdown();
}

geometry_msgs::Twist turtlebot_follower::controller()
{
    turtlebot_follower::VariConversion();
    geometry_msgs::Twist controlOut;

//    controlOut.linear.x = K_x_p * (this->target.x - this->tracker.x) +
//                          K_x_d * (this->target.x_dot - this->tracker.x_dot);
//    controlOut.linear.y = K_y_p * (this->target.y - this->tracker.y) +
//                          K_y_d * (this->target.y_dot - this->tracker.y_dot);
//    controlOut.linear.z = 0;
//    controlOut.angular.x = 0;
//    controlOut.angular.y = 0;
//    controlOut.angular.z = K_psi_p * (this->target.psi - this->tracker.psi) +
//                           K_psi_d * (this->target.psi_dot - this->tracker.psi_dot);


    // angle to which a tracker turtlebot should rotate to be in line to target turtlebot
    ang_err_old = ang_err;
    ang_err = std::atan2(target.y-tracker.y,target.x-tracker.x) - tracker.psi;

    dist_err_old = dist_err;
    dist_err = (sqrt(pow(target.x-tracker.x,2) + pow(target.y-tracker.y,2)) - 1.0);

    while(ang_err < -3.1415926) ang_err += 2*3.1415926;
    while(ang_err > 3.1415926) ang_err -= 2*3.1415926;

    controlOut.angular.x = 0;
    controlOut.angular.y = 0;
//    controlOut.angular.z = 1.0 * (atan2(target.y,target.x) - atan2(tracker.y,tracker.x));
    controlOut.angular.z = Ka_p * ang_err + Ka_d * (ang_err - ang_err_old) * freq;
//    controlOut.angular.z = 1;
    controlOut.linear.x = Kl_p * dist_err + Kl_d * (dist_err - dist_err_old) * freq;
    controlOut.linear.y = 0;
    controlOut.linear.z = 0;
    return controlOut;
}

// With TF
//geometry_msgs::Twist turtlebot_follower::controller()
//{
//    //turtlebot_follower::VariConversion();
//    geometry_msgs::Twist controlOut;
//    controlOut.angular.z = 4.0 * atan2(transform.getOrigin().y(),
//                                    transform.getOrigin().x());
//    controlOut.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
//                                  pow(transform.getOrigin().y(), 2));
//    return controlOut;
//}


//void turtlebot_follower::tfFunction()
//{
////    listener.lookupTransform("/turtlebot_2", "/turtlebot_1",
////                             ros::Time(0), transform);
//    this->listener.lookupTransform("turtlebot_2/base_link", "/carrot1",
//                             ros::Time(0), this->transform);
//}

void turtlebot_follower::PublishFunction()
{
    this->vel_pub = this->nh_.advertise<geometry_msgs::Twist>("/turtlebot_2/vel_teleop", freq);
    vel_pub.publish(turtlebot_follower::controller());
}

void turtlebot_follower::VariConversion()
{

//    tf::Quaternion q;
//    quaternionTFToMsg(q, tracker_pose.orientation);
//    tf::Matrix3x3 m(q);
//    double roll, pitch, yaw;
//    m.getRPY(roll, pitch, yaw);

    geometry_msgs::Quaternion q = tracker_pose.orientation;
    double yaw = std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));

    this->target.x = this->target_pose.position.x;
    this->target.y = this->target_pose.position.y;
 //   this->target.psi = atan2(2*(W1*Z1+X1*Y1),1-2*(Y1*Y1+Z1*Z1));
//    this->target.x_dot = this->target_twist.linear.x;
//    this->target.y_dot = this->target_twist.linear.y;
//    this->target.psi_dot = this->target_twist.angular.z;

    this->tracker.x = this->tracker_pose.position.x;
    this->tracker.y = this->tracker_pose.position.y;
    this->tracker.psi = yaw;
 //   this->tracker.psi = atan2(2*(W2*Z2+X2*Y2),1-2*(Y2*Y2+Z2*Z2));
//    this->tracker.x_dot = this->tracker_twist.linear.x;
//    this->tracker.y_dot = this->tracker_twist.linear.y;
//    this->tracker.psi_dot = this->tracker_twist.angular.z;
}

void turtlebot_follower::ModelStateCallBack(const gazebo_msgs::ModelStates::ConstPtr &gazebo_msg)
{
    std::vector<std::string> model_names = gazebo_msg->name;
    std::vector<geometry_msgs::Pose> pose_vector = gazebo_msg->pose;
    // std::vector<geometry_msgs::Twist> twist_vector = gazebo_msg->twist;
    int tracker_idx = (int) (std::find(model_names.begin(),model_names.end(),this->tracker_name) - model_names.begin());
    int target_idx = (int) (std::find(model_names.begin(),model_names.end(),this->target_name) - model_names.begin());
    if (tracker_idx < model_names.size()) {
        this->tracker_pose = pose_vector[tracker_idx]; //current tracker position
        //ROS_INFO("tracker_idx: %d", tracker_idx);
        //   this->tracker_twist = twist_vector[tracker_idx];
    }
    else
        ROS_WARN_ONCE("specified tracker name was not found in gazebo");
    if (target_idx<model_names.size()) {
        this->target_pose = pose_vector[target_idx]; //current tracker position
        //ROS_INFO("target_idx: %d", target_idx);
     //   this->target_twist = twist_vector[target_idx];
    }
    else
        ROS_WARN_ONCE("specified target name was not found in gazebo");
}
