#ifndef __DATA_MANAGER_H__
#define __DATA_MANAGER_H__
#include <DataManager/common.h>
#include <DataManager/data_ros_input.h>
#include <DataManager/data_ros_output.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <queue>
enum class FlightState { TAKEOFF, SPIN,SPIN1, FOLLOW_TRAJECTORY,Landing,FINISH };
class DataManager {
 public:
  DataManager(ros::NodeHandle &nh);
  ~DataManager() = default;
  void Execute();
  void update();
  void takeoff();
  void spin();
  void spin1();
  void landing();
  void ego_state_Callback(const std_msgs::Int32::ConstPtr &msg);
    void ego_intoend_Callback(const std_msgs::Int32::ConstPtr &msg);
  std_msgs::Int32 ego_start_flag;
  std_msgs::Int32 ego_intofinish_flag;
 private:
  ros::NodeHandle nh_;
  int SIM_OR_REAL;
  std::shared_ptr<DataRosInput> data_input_;
  std::shared_ptr<DataRosOutput> data_output_;
  FlightState current_fsm_;
  ros::Time start_time_;
  mavros_msgs::State cur_state_;
  bool if_first_in;
  UavState last_desire_uav_state_;
  UavState desire_uav_state_;
  UavState odom_;
  std::queue<UavState> plan_queue_;
  std::string planner_state_{"INIT"};
  ros::Publisher spin_flag_pub;
  ros::Publisher spin_position_pub;
  ros::Subscriber ego_state_sub;
   ros::Subscriber ego_into_sub;
  std_msgs::Int32 fuel_stop_flag;
  bool is_spin_success_{false};
};
#endif