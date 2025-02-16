/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "quadrotor_msgs/PositionCommand.h"
#define VELOCITY2D_CONTROL \
  0b101111000111  // 设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
// 设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.

class Ctrl {
 public:
  Ctrl();
  void state_cb(const mavros_msgs::State::ConstPtr& msg);
  void position_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
  void control(const ros::TimerEvent&);
  void fuel_stop_callback(const std_msgs::Int32::ConstPtr& msg);
  ros::NodeHandle nh;
  visualization_msgs::Marker trackpoint;
  quadrotor_msgs::PositionCommand ego;
  tf::StampedTransform ts;  // 用来发布无人机当前位置的坐标系坐标轴
  tf::TransformBroadcaster tfBroadcasterPointer;  // 广播坐标轴
  unsigned short velocity_mask = VELOCITY2D_CONTROL;
  mavros_msgs::PositionTarget current_goal;
  mavros_msgs::RCIn rc;
  nav_msgs::Odometry position_msg;
  geometry_msgs::PoseStamped target_pos;
  geometry_msgs::PoseStamped mission_point1, mission_point2;
  mavros_msgs::State current_state;
  float position_x, position_y, position_z, now_x, now_y, now_yaw, current_yaw,
      targetpos_x, targetpos_y;
  float ego_pos_x, ego_pos_y, ego_pos_z, ego_vel_x, ego_vel_y, ego_vel_z,
      ego_a_x, ego_a_y, ego_a_z, ego_yaw,
      ego_yaw_rate;           // EGO planner information has position velocity
                              // acceleration yaw yaw_dot
  bool receive, get_now_pos;  // 触发轨迹的条件判断
  bool start_flag;
  int mission_flag;
  ros::Subscriber state_sub, twist_sub, target_sub, position_sub, fuel_stop_sub;
  ros::Publisher local_pos_pub, pubMarker, mission_waypoint_pub, ego_stop_pub;
  ros::ServiceClient arming_client, set_mode_client;
  ros::Timer timer;
  ros::Time last_request;
  // 标志flag 表示ego进入房间内完成
  std_msgs::Int32 ego_into_end_flag;
  mavros_msgs::SetMode offb_set_mode;
  mavros_msgs::CommandBool arm_cmd;
};
Ctrl::Ctrl() {
  timer = nh.createTimer(ros::Duration(0.02), &Ctrl::control, this);
  state_sub = nh.subscribe("/mavros/state", 10, &Ctrl::state_cb, this);
  position_sub = nh.subscribe("/mavros/local_position/odom", 10,
                              &Ctrl::position_cb, this);
  mission_waypoint_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  target_sub =
      nh.subscribe("/move_base_simple/goal", 10, &Ctrl::target_cb, this);
  twist_sub = nh.subscribe("/position_cmd", 10, &Ctrl::twist_cb, this);
  local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>(
      "/mavros/setpoint_raw/local", 1);
  pubMarker = nh.advertise<visualization_msgs::Marker>("/track_drone_point", 5);
  arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  get_now_pos = false;
  receive = true;
  start_flag = false;
  mission_flag = 0;
  ego_into_end_flag.data = 1;
  ego_stop_pub = nh.advertise<std_msgs::Int32>("/ego/into_end_flag", 10);
  fuel_stop_sub =
      nh.subscribe("/fuel/stop_flag", 10, &Ctrl::fuel_stop_callback, this);
  // 进门两个点
  mission_point1.header.frame_id = "world";
  mission_point1.pose.position.x = 1.0;
  mission_point1.pose.position.y = 0.0;
  mission_point2.header.frame_id = "world";
  mission_point2.pose.position.x = 2.0;
  mission_point2.pose.position.y = 0.0 ;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;
}
void Ctrl::state_cb(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

// read vehicle odometry
void Ctrl::position_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  position_msg = *msg;
  tf2::Quaternion quat;
  tf2::convert(
      msg->pose.pose.orientation,
      quat);  // 把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ts.stamp_ = msg->header.stamp;
  ts.frame_id_ = "world";
  ts.child_frame_id_ = "drone_frame";
  ts.setRotation(tf::Quaternion(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y,
                           msg->pose.pose.position.z));
  tfBroadcasterPointer.sendTransform(ts);
  if (!get_now_pos) {
    now_x = position_msg.pose.pose.position.x;
    now_y = position_msg.pose.pose.position.y;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat);
    now_yaw = yaw;
    get_now_pos = true;
  }
  position_x = position_msg.pose.pose.position.x;
  position_y = position_msg.pose.pose.position.y;
  position_z = position_msg.pose.pose.position.z;
  current_yaw = yaw;
}

void Ctrl::target_cb(
    const geometry_msgs::PoseStamped::ConstPtr& msg)  // 读取rviz的航点
{
  ROS_INFO("WAYPOINT GET");
  if (mission_flag == 4) {
    ROS_INFO("waypoint rviz get");
    receive = true;
    target_pos = *msg;
    targetpos_x = target_pos.pose.position.x;
    targetpos_y = target_pos.pose.position.y;

    mission_flag = 5;
    for (int i = 0; i < 100; i++) ROS_INFO("mf:%d", mission_flag);
  }
}

// 读取ego里的位置速度加速度yaw和yaw-dot信息，其实只需要ego的位置速度和yaw就可以了
void Ctrl::twist_cb(
    const quadrotor_msgs::PositionCommand::ConstPtr& msg)  // ego的回调函数
{
  ego = *msg;
  ego_pos_x = ego.position.x;
  ego_pos_y = ego.position.y;
  ego_pos_z = ego.position.z;
  ego_vel_x = ego.velocity.x;
  ego_vel_y = ego.velocity.y;
  ego_vel_z = ego.velocity.z;
  ego_yaw = ego.yaw;
  ego_yaw_rate = ego.yaw_dot;
}

void Ctrl::control(const ros::TimerEvent&) {
  ros::Rate rate(20.0);
  mavros_msgs::PositionTarget init_pos;
  init_pos.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  init_pos.header.stamp = ros::Time::now();
  init_pos.type_mask = 0b101111111000;
  init_pos.position.x = 0;
  init_pos.position.y = 0;
  init_pos.position.z = 0;
  init_pos.yaw = current_yaw;

  // 在进入Offboard模式之前，必须已经启动了LocalPosPub_数据流，否则模式切换将被拒绝。
  // 这里的100可以被设置为任意数
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(init_pos);
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok()) {
    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(0.1)) &&
        start_flag == false) {
      // 客户端set_mode_client向服务端offb_set_mode发起请求call，然后服务端回应response将模式返回，这就打开了offboard模式
      if (set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent) {
        // 切换到 OFFBOARD 模式
        ROS_INFO("OFFBOARD MODE");
      } else
        ROS_INFO("OFFBOARD fail");
      last_request = ros::Time::now();
    }

    // 判断当前状态是否解锁，如果没有解锁，则进入if语句内部
    // 这里是5秒钟进行一次判断，避免飞控被大量的请求阻塞
    else if (!current_state.armed &&
             (ros::Time::now() - last_request > ros::Duration(0.1)) &&
             start_flag == false) {
      if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("lift off!");
        start_flag = true;
      } else
        ROS_INFO("ARM XXX");
      last_request = ros::Time::now();
    }

    while (start_flag == true) {
      if (mission_flag == 0) {
        current_goal.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_goal.header.stamp = ros::Time::now();
        current_goal.type_mask = 0b101111111000;
        current_goal.position.x = 0;
        current_goal.position.y = 0;
        current_goal.position.z = 1.0;
        current_goal.yaw = current_yaw;
        ROS_INFO("ready to fly\n");

        if (abs(position_z - current_goal.position.z) < 0.2f) {
          ROS_INFO("Takeoff successfully");
          mission_flag = 1;
        }

        local_pos_pub.publish(current_goal);
      }

      // 定点进门
      else if (mission_flag == 1)  // 触发后进行轨迹跟踪
      {
        for (int i = 0; i < 10; i++) {
          mission_point1.header.stamp = ros::Time::now();
          mission_waypoint_pub.publish(mission_point1);
          rate.sleep();
        }

        while (abs(mission_point1.pose.position.x - position_x) > 0.1f ||
               abs(mission_point1.pose.position.y - position_y) > 0.1f) {
          current_goal.coordinate_frame = mavros_msgs::PositionTarget::
              FRAME_LOCAL_NED;  // 选择local系，一定要local系
          current_goal.header.stamp = ros::Time::now();
          current_goal.type_mask =
              velocity_mask;  // 这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
          current_goal.velocity.x =
              0.8 * ego_vel_x + (ego_pos_x - position_x) * 1;
          current_goal.velocity.y =
              0.8 * ego_vel_y + (ego_pos_y - position_y) * 1;
          current_goal.velocity.z = (ego_pos_z - position_z) * 1;
          current_goal.yaw = ego_yaw;
          ROS_INFO("已触发控制器，当前EGO规划速度：vel_x = %.2f\n",
                   sqrt(pow(current_goal.velocity.x, 2) +
                        pow(current_goal.velocity.y, 2)));

          local_pos_pub.publish(current_goal);
          ros::spinOnce();
          rate.sleep();
        }

        // 定点1任务结束
        mission_flag = 2;
      }

      // 定点进房
      else if (mission_flag == 2)  // 触发后进行轨迹跟踪
      {
        for (int i = 0; i < 10; i++) {
          mission_waypoint_pub.publish(mission_point2);
        }

        while (abs(mission_point2.pose.position.x - position_x) > 0.1f ||
               abs(mission_point2.pose.position.y - position_y) > 0.1f) {
          current_goal.coordinate_frame = mavros_msgs::PositionTarget::
              FRAME_LOCAL_NED;  // 选择local系，一定要local系
          current_goal.header.stamp = ros::Time::now();
          current_goal.type_mask =
              velocity_mask;  // 这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
          current_goal.velocity.x =
              0.8 * ego_vel_x + (ego_pos_x - position_x) * 1;
          current_goal.velocity.y =
              0.8 * ego_vel_y + (ego_pos_y - position_y) * 1;
          current_goal.velocity.z = (ego_pos_z - position_z) * 1;
          current_goal.yaw = ego_yaw;
          ROS_INFO("已触发控制器，当前EGO规划速度：vel_x = %.2f\n",
                   sqrt(pow(current_goal.velocity.x, 2) +
                        pow(current_goal.velocity.y, 2)));

          local_pos_pub.publish(current_goal);
          ros::spinOnce();
          rate.sleep();
        }

        // 定点2任务结束
        mission_flag = 3;
        // 发送指令让fuel启动
        for (int i = 0; i < 10; i++) ego_stop_pub.publish(ego_into_end_flag);
      }

      else if (mission_flag == 3)
        ROS_INFO("FUEL RUNNING");

      else if (mission_flag ==
               4)  // 如果没有在rviz上打点，则offboard模式下会保持在1m的高度
      {
        current_goal.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        current_goal.header.stamp = ros::Time::now();
        current_goal.type_mask = 0b101111111000;
        current_goal.position.x = position_x;
        current_goal.position.y = position_y;
        current_goal.position.z = position_z;
        current_goal.yaw = current_yaw;
        ROS_INFO("REMAIN POSITION\n");

        local_pos_pub.publish(current_goal);
      }

      else if (mission_flag == 5) {
        current_goal.coordinate_frame = mavros_msgs::PositionTarget::
            FRAME_LOCAL_NED;  // 选择local系，一定要local系
        current_goal.header.stamp = ros::Time::now();
        current_goal.type_mask =
            velocity_mask;  // 这个就算对应的掩码设置，可以看mavros_msgs::PositionTarget消息格式
        current_goal.velocity.x =
            0.8 * ego_vel_x + (ego_pos_x - position_x) * 1;
        current_goal.velocity.y =
            0.8 * ego_vel_y + (ego_pos_y - position_y) * 1;
        current_goal.velocity.z = (ego_pos_z - position_z) * 1;
        current_goal.yaw = ego_yaw;
        ROS_INFO("已触发控制器，当前EGO规划速度：vel_x = %.2f\n",
                 sqrt(pow(current_goal.velocity.x, 2) +
                      pow(current_goal.velocity.y, 2)));

        local_pos_pub.publish(current_goal);
      }
      ROS_INFO("FLAG:%d", mission_flag);
      ros::spinOnce();
      rate.sleep();
    }
  }
}

void Ctrl::fuel_stop_callback(const std_msgs::Int32::ConstPtr& msg) {
  if (msg->data == 1) mission_flag = 4;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cxr_egoctrl_v1");
  setlocale(LC_ALL, "");

  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  Ctrl ctrl;
  ros::spin();
  return 0;
}
