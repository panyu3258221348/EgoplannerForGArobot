#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

enum Mode { GROUND, AIR, TAKEOFF, LANDING };
Mode mode_ = GROUND;
double ground_thresh_ = 0.3;
double kp_xy_ = 0.5, kp_z_ = 0.3, kp_yaw_ = 1.0;
ros::Publisher cmd_vel_pub_, px4_pub_;
nav_msgs::Odometry last_odom_;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  last_odom_ = *msg;
}

void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd) {
  double target_z = cmd->position.z;

  if (target_z < ground_thresh_) {
    mode_ = GROUND;
  } else if (mode_ == GROUND) {
    mode_ = TAKEOFF;
  }

  if (mode_ == GROUND || mode_ == TAKEOFF) {
    geometry_msgs::Twist twist;
    double dx = cmd->position.x - last_odom_.pose.pose.position.x;
    double dy = cmd->position.y - last_odom_.pose.pose.position.y;
    twist.linear.x = kp_xy_ * sqrt(dx*dx + dy*dy);
    twist.angular.z = kp_yaw_ * atan2(dy, dx);
    cmd_vel_pub_.publish(twist);

    if (mode_ == TAKEOFF && target_z >= ground_thresh_ + 0.1)
      mode_ = AIR;
  }

  if (mode_ == AIR) {
    px4_pub_.publish(cmd);
    if (target_z < ground_thresh_)
      mode_ = LANDING;
  }

  if (mode_ == LANDING) {
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = 0;
    cmd_vel_pub_.publish(twist);
    if (fabs(last_odom_.pose.pose.position.z) < 0.05)
      mode_ = GROUND;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "tabv_controller");
  ros::NodeHandle nh("~");

  nh.param("ground_thresh", ground_thresh_, 0.3);

  ros::Subscriber odom_sub = nh.subscribe("/visual_slam/odom", 1, odomCallback);
  ros::Subscriber cmd_sub = nh.subscribe("/planning/pos_cmd", 1, posCmdCallback);

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  px4_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 1);

  ros::spin();
  return 0;
}
