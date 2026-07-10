#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <memory>

ros::Publisher odom_pub, cloud_pub, cam_pose_pub;
ros::ServiceClient set_state_client;
std::string model_name;
std::unique_ptr<tf::TransformBroadcaster> tf_br;
Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
Eigen::Matrix4d T_base_cam = Eigen::Matrix4d::Identity();

void posCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd) {
  gazebo_msgs::ModelState state;
  state.model_name = model_name;
  state.reference_frame = "world";
  state.pose.position.x = cmd->position.x;
  state.pose.position.y = cmd->position.y;
  state.pose.position.z = cmd->position.z;
  double yaw = cmd->yaw;
  if (fabs(cmd->velocity.x) > 0.01 || fabs(cmd->velocity.y) > 0.01)
    yaw = atan2(cmd->velocity.y, cmd->velocity.x);
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
  state.pose.orientation.x = q.x();
  state.pose.orientation.y = q.y();
  state.pose.orientation.z = q.z();
  state.pose.orientation.w = q.w();
  gazebo_msgs::SetModelState srv;
  srv.request.model_state = state;
  set_state_client.call(srv);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  nav_msgs::Odometry odom = *msg;
  odom.header.frame_id = "world";
  odom.child_frame_id = "base_link";
  odom_pub.publish(odom);

  tf::Transform t;
  t.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
  t.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));
  tf_br->sendTransform(tf::StampedTransform(t, odom.header.stamp, "world", "base_link"));

  tf::Transform t_cam;
  t_cam.setOrigin(tf::Vector3(0.1, 0, 0.05));
  t_cam.setRotation(tf::Quaternion::getIdentity());
  tf_br->sendTransform(tf::StampedTransform(t_cam, odom.header.stamp, "base_link", "depth_cam"));

  // publish camera pose as PoseStamped for grid_map depth callback
  geometry_msgs::PoseStamped cam_pose;
  cam_pose.header.stamp = odom.header.stamp;
  cam_pose.header.frame_id = "world";
  Eigen::Matrix4d T_wc = T_world_base * T_base_cam;
  cam_pose.pose.position.x = T_wc(0,3);
  cam_pose.pose.position.y = T_wc(1,3);
  cam_pose.pose.position.z = T_wc(2,3);
  Eigen::Quaterniond q_cam(T_wc.block<3,3>(0,0));
  cam_pose.pose.orientation.w = q_cam.w();
  cam_pose.pose.orientation.x = q_cam.x();
  cam_pose.pose.orientation.y = q_cam.y();
  cam_pose.pose.orientation.z = q_cam.z();
  cam_pose_pub.publish(cam_pose);

  static int cnt = 0;
  if (cnt++ == 0) {
    ROS_ERROR(">>> cam_pose: pos(%.2f,%.2f,%.2f) orient(w=%.3f,x=%.3f,y=%.3f,z=%.3f)",
             cam_pose.pose.position.x, cam_pose.pose.position.y, cam_pose.pose.position.z,
             cam_pose.pose.orientation.w, cam_pose.pose.orientation.x,
             cam_pose.pose.orientation.y, cam_pose.pose.orientation.z);
    Eigen::Matrix3d R = Eigen::Quaterniond(cam_pose.pose.orientation.w,
                                            cam_pose.pose.orientation.x,
                                            cam_pose.pose.orientation.y,
                                            cam_pose.pose.orientation.z).toRotationMatrix();
    ROS_ERROR("  R col0(%.2f,%.2f,%.2f) col1(%.2f,%.2f,%.2f) col2(%.2f,%.2f,%.2f)",
             R(0,0),R(1,0),R(2,0), R(0,1),R(1,1),R(2,1), R(0,2),R(1,2),R(2,2));
  }

  Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
  T_world_base = Eigen::Matrix4d::Identity();
  T_world_base.block<3,3>(0,0) = q.toRotationMatrix();
  T_world_base(0,3) = odom.pose.pose.position.x;
  T_world_base(1,3) = odom.pose.pose.position.y;
  T_world_base(2,3) = odom.pose.pose.position.z;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  static int n = 0;
  pcl::PointCloud<pcl::PointXYZ> cloud_in, cloud_out;
  pcl::fromROSMsg(*msg, cloud_in);

  if (n++ == 0) {
    int valid = 0;
    for (auto& p : cloud_in) if (!std::isnan(p.x)) valid++;
    ROS_ERROR(">>> ctrl got %zu pts, valid=%d, first=(%.2f,%.2f,%.2f)", cloud_in.size(), valid,
             cloud_in[0].x, cloud_in[0].y, cloud_in[0].z);
  }
  pcl::fromROSMsg(*msg, cloud_in);
  cloud_out.resize(cloud_in.size());

  Eigen::Matrix4d T_world_cam = T_world_base * T_base_cam;
  cloud_out.clear();
  cloud_out.reserve(1000);
  size_t step = std::max((size_t)1, cloud_in.size() / 1000);
  for (size_t i = 0; i < cloud_in.size(); i += step) {
    if (std::isnan(cloud_in[i].x)) continue;
    Eigen::Vector4d p(cloud_in[i].x, cloud_in[i].y, cloud_in[i].z, 1.0);
    Eigen::Vector4d pw = T_world_cam * p;
    if (pw(2) < 0.1) continue;
    cloud_out.push_back({(float)pw(0), (float)pw(1), (float)pw(2)});
    if (cloud_out.size() >= 1000) break;
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(cloud_out, out);
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = "world";
  cloud_pub.publish(out);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "gazebo_controller");
  ros::NodeHandle nh("~");


  tf_br = std::make_unique<tf::TransformBroadcaster>();
  nh.param<std::string>("model_name", model_name, "tabv");

  T_base_cam(0,0) = 0.0; T_base_cam(0,1) = 0.0; T_base_cam(0,2) =  1.0;
  T_base_cam(1,0) = -1.0; T_base_cam(1,1) = 0.0; T_base_cam(1,2) = 0.0;
  T_base_cam(2,0) = 0.0; T_base_cam(2,1) = -1.0; T_base_cam(2,2) = 0.0;
  T_base_cam(0,3) = 0.1;
  T_base_cam(2,3) = 0.05;

  ros::Subscriber pos_sub = nh.subscribe("/planning/pos_cmd", 1, posCmdCallback);
  ros::Subscriber gt_sub = nh.subscribe("/ground_truth/odom", 1, odomCallback);
  ros::Subscriber pc_sub = nh.subscribe("/d435/depth/points", 1, cloudCallback);

  odom_pub = nh.advertise<nav_msgs::Odometry>("/visual_slam/odom", 10);
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/grid_map/cloud", 10);
  cam_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/depth_cam/pose", 10);

  set_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  ros::spin();
  return 0;
}
