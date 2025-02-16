#include <plan_manage/kino_replan_fsm.h>
#include <plan_manage/local_explore_fsm.h>
#include <plan_manage/topo_replan_fsm.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

// lb change
std_msgs::Int32 ego_intofinish_flag;

void ego_stop_callback(const std_msgs::Int32::ConstPtr& msg) {
  ego_intofinish_flag = *msg;
  ROS_INFO("fuel_start_flag:%d", ego_intofinish_flag.data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fast_planner_node");
  ros::NodeHandle nh("~");

  // lb change
  ego_intofinish_flag.data = 0;
  ros::Subscriber ego_stop_sub =
      nh.subscribe("/ego/into_end_flag", 10, &ego_stop_callback);

  while (ego_intofinish_flag.data == 0) {
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  int planner;
  nh.param("planner_node/planner", planner, -1);

  TopoReplanFSM topo_replan;
  KinoReplanFSM kino_replan;
  LocalExploreFSM local_explore;

  if (planner == 1) {
    kino_replan.init(nh);
  } else if (planner == 2) {
    topo_replan.init(nh);
  } else if (planner == 3) {
    local_explore.init(nh);
  }

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
