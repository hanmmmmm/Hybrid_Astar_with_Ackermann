
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
// #include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "node_odom_from_tf");

  ros::NodeHandle node;

  ros::Publisher odom_filt_pub = node.advertise<nav_msgs::Odometry>("/odom/filtered", 10);

  nav_msgs::Odometry odom_msg;

  tf::TransformListener listener;

  std::string odom_frame_name = "/odom_combined";
  std::string robot_frame_name = "/base_footprint";

  std::string odom_frame_name_noslash = odom_frame_name.substr(1);
  std::string robot_frame_name_noslash = robot_frame_name.substr(1);

  odom_msg.child_frame_id = robot_frame_name_noslash;
  odom_msg.header.frame_id = odom_frame_name_noslash;

  ros::Rate rate(20.0);

  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(odom_frame_name, robot_frame_name, ros::Time(0), transform);

      odom_msg.header.stamp = ros::Time::now();

      odom_msg.pose.pose.position.x = transform.getOrigin().x();
      odom_msg.pose.pose.position.y = transform.getOrigin().y();
      odom_msg.pose.pose.position.z = transform.getOrigin().z();
      
      odom_msg.pose.pose.orientation.x = transform.getRotation().x();
      odom_msg.pose.pose.orientation.y = transform.getRotation().y();
      odom_msg.pose.pose.orientation.z = transform.getRotation().z();
      odom_msg.pose.pose.orientation.w = transform.getRotation().w();

      odom_filt_pub.publish( odom_msg );

    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    rate.sleep();
  }
  return 0;
};














// frame_id: "odom_combined"
// child_frame_id: "base_footprint"
// rosrun tf tf_echo /odom_combined /base_footprint






