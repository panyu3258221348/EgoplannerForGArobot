#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <mutex>

nav_msgs::Odometry odom_;
std::mutex odom_mutex_;
ros::Publisher odom_pub_;
double dt_ = 0.01;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  double v = msg->linear.x;
  double w = msg->angular.z;
  std::lock_guard<std::mutex> lock(odom_mutex_);
  double yaw = tf::getYaw(odom_.pose.pose.orientation);
  yaw += w * dt_;
  odom_.pose.pose.position.x += v * cos(yaw) * dt_;
  odom_.pose.pose.position.y += v * sin(yaw) * dt_;
  odom_.pose.pose.position.z = 0.0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
}

void timerCallback(const ros::TimerEvent &) {
  std::lock_guard<std::mutex> lock(odom_mutex_);
  odom_.header.stamp = ros::Time::now();
  odom_pub_.publish(odom_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ground_sim");
  ros::NodeHandle nh("~");

  odom_.header.frame_id = "world";
  odom_.child_frame_id = "base_link";
  odom_.pose.pose.orientation.w = 1.0;
  nh.param("dt", dt_, 0.01);

  ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, cmdVelCallback);
  ros::Timer timer = nh.createTimer(ros::Duration(dt_), timerCallback);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("/visual_slam/odom", 10);

  ros::spin();
  return 0;
}
