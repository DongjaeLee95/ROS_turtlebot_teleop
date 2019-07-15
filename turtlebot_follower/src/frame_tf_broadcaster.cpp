#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10.0);
  while (node.ok()){
    // transform.setOrigin( tf::Vector3(-2.0, 0.0, 0.0) );
    // transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

    transform.setOrigin( tf::Vector3(-1.0, 0.0, 0.0) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "turtlebot_1/base_link", "/carrot1"));
    rate.sleep();
  }
  return 0;
};
