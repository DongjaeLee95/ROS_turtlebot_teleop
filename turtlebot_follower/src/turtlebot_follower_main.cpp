#include <ros/ros.h>
#include "turtlebot_follower/turtlebot_follower.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlebot_follower");

    turtlebot_follower turtlebotFollower;

    int freq = turtlebotFollower.freq;

    ros::Rate loop_rate(freq);

    while (ros::ok())
    {
//        try{
//            turtlebotFollower.tfFunction();
//        }
//        catch (tf::TransformException &ex) {
//            ROS_ERROR("%s",ex.what());
//            ros::Duration(1.0).sleep();
//            continue;
//        }

        turtlebotFollower.PublishFunction();

        ros::spinOnce();

        loop_rate.sleep();
    }

    //ros::spin();

    return 0;
}
