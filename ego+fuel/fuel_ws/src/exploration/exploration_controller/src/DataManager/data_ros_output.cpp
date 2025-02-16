#include <DataManager/data_ros_output.h>
DataRosOutput::DataRosOutput(ros::NodeHandle &nh) : nh_(nh) { init(); };
void DataRosOutput::init() {
  position_yaw_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      "/iris_0/mavros/setpoint_position/local", 1);
  // vel_pub_=nh_.advertise<geometry_msgs::TwistStamped>( "/iris_0/mavros/setpoint_vel/local", 1);
  // 无人机控制话题，包括：位置，速度，加速度，角度，角速度
  target_pub_=nh_.advertise<mavros_msgs::PositionTarget>( "/mavros/setpoint_raw/local", 1);

  arming_client_ =
      nh_.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");

  ros::ServiceClient set_mode_client_ =
      nh_.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");
  trigger_pub_ =
      nh_.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 1);
  landing_client_ =
      nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
}
// void DataRosOutput::sendDesireState(const UavState &desire_state) {
//   geometry_msgs::PoseStamped pos_pub;
//   pos_pub.pose.position.x = desire_state.pos(0);
//   pos_pub.pose.position.y = desire_state.pos(1);
//   pos_pub.pose.position.z = desire_state.pos(2);
//   pos_pub.pose.orientation.x = desire_state.q.x();
//   pos_pub.pose.orientation.y = desire_state.q.y();
//   pos_pub.pose.orientation.z = desire_state.q.z();
//   pos_pub.pose.orientation.w = desire_state.q.w();
//   publishPositionYaw(pos_pub);
// }

// void DataRosOutput::sendDesireState(const UavState &desire_state,
//                                     const UavState &now_odom) {
//   //   geometry_msgs::PoseStamped pos_pub;
//   //   pos_pub.pose.position.x = desire_state.pos(0);
//   //   pos_pub.pose.position.y = desire_state.pos(1);
//   //   pos_pub.pose.position.z = desire_state.pos(2);
//   //   pos_pub.pose.orientation.x = desire_state.q.x();
//   //   pos_pub.pose.orientation.y = desire_state.q.y();
//   //   pos_pub.pose.orientation.z = desire_state.q.z();
//   //   pos_pub.pose.orientation.w = desire_state.q.w();
//   //   publishPositionYaw(pos_pub);

//   //----------------------------------------------------------
//   mavros_msgs::PositionTarget local_raw;
//   local_raw.header.stamp = ros::Time::now();
//   local_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
//   // local_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
//   // mavros_msgs::PositionTarget::IGNORE_AFY |
//   // mavros_msgs::PositionTarget::IGNORE_AFZ |
//   //                       mavros_msgs::PositionTarget::IGNORE_VX |
//   //                       mavros_msgs::PositionTarget::IGNORE_VY |
//   //                       mavros_msgs::PositionTarget::IGNORE_VZ|
//   //                       mavros_msgs::PositionTarget::FORCE |
//   //                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
//   // local_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
//   // mavros_msgs::PositionTarget::IGNORE_AFY |
//   // mavros_msgs::PositionTarget::IGNORE_AFZ |
//   // mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
//   local_raw.type_mask = 0b101111000111;
//   //   local_raw.position.x = desire_state.pos(0);
//   //   local_raw.position.y = desire_state.pos(1);
//   //   local_raw.position.z = desire_state.pos(2);
//   //   local_raw.velocity.x = desire_state.vel(0);
//   //   local_raw.velocity.y = desire_state.vel(1);
//   //   local_raw.velocity.z = desire_state.vel(2);
//   // local_raw.acceleration_or_force.x = desire_state.acc(0);
//   // local_raw.acceleration_or_force.y = desire_state.acc(1);
//   // local_raw.acceleration_or_force.z = desire_state.acc(2);
//   local_raw.velocity.x = 0.6 * (0.8 * desire_state.vel(0) +
//                                 (desire_state.pos(0) - now_odom.pos(0)) * 1);
//   local_raw.velocity.y = 0.6 * (0.8 * desire_state.vel(1) +
//                                 (desire_state.pos(1) - now_odom.pos(1)) * 1);
//   local_raw.velocity.z = 0.6 * (0.8 * desire_state.vel(2) +
//                                 (desire_state.pos(2) - now_odom.pos(2)) * 1);
//   local_raw.yaw = desire_state.yaw;
//   publishPositionYaw(local_raw);
// }

// void DataRosOutput::sendDesireVel(const UavState &desire_state) {
//   geometry_msgs::TwistStamped vel_pub;
// //   vel_pub.pose.position.x = desire_state.pos(0);
// //   vel_pub.pose.position.y = desire_state.pos(1);
// //   vel_pub.pose.position.z = desire_state.pos(2);
//   vel_pub.pose.orientation.x = desire_state.q.x();
//   vel_pub.pose.orientation.y = desire_state.q.y();
//   vel_pub.pose.orientation.z = desire_state.q.z();
//   vel_pub.pose.orientation.w = desire_state.q.w();
//   publishVel(vel_pub);
// }
void DataRosOutput::sendDesireState(const UavState &desire_state) {
  mavros_msgs::PositionTarget local_raw;
  local_raw.header.stamp = ros::Time::now();
  local_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  local_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ|
                        mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
 // local_raw.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
   
  local_raw.position.x = desire_state.pos(0);
  local_raw.position.y = desire_state.pos(1);
  local_raw.position.z = desire_state.pos(2);
//   local_raw.velocity.x = desire_state.vel(0);
//   local_raw.velocity.y = desire_state.vel(1);
//   local_raw.velocity.z = desire_state.vel(2);
  //local_raw.acceleration_or_force.x = desire_state.acc(0);
  //local_raw.acceleration_or_force.y = desire_state.acc(1);
  //local_raw.acceleration_or_force.z = desire_state.acc(2);
  local_raw.yaw = desire_state.yaw;
  publishPositionYaw(local_raw);
}