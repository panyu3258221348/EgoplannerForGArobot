#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace plan_manage;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ego_planner_node");
  ros::NodeHandle nh("~");

  EGOReplanFSM rebo_replan;

  fprintf(stderr, ">>> FSM init start\n"); fflush(stderr);
  rebo_replan.init(nh);
  fprintf(stderr, ">>> FSM init done, spinning, use_sim_time=%d\n", (int)ros::Time::isSimTime()); fflush(stderr);

  ros::spin();

  return 0;
}
