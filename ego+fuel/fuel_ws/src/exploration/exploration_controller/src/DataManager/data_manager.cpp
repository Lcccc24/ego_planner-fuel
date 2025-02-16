#include <DataManager/data_manager.h>

// 7,9日更改,更改了data_manager.h文件,加上include<cmath>,多定义了一个time_add

DataManager::DataManager(ros::NodeHandle &nh) : nh_(nh) {
  data_input_ = std::make_shared<DataRosInput>(nh_);
  data_output_ = std::make_shared<DataRosOutput>(nh_);
  ros::param::get("sim_or_real", SIM_OR_REAL);
  ego_start_flag.data = 0;
  fuel_stop_flag.data = 0;
   if_first_in = true;
  ego_intofinish_flag.data = 0;
  spin_flag_pub = nh_.advertise<std_msgs::Int32>("/fuel/stop_flag", 1);
  spin_position_pub =
      nh_.advertise<geometry_msgs::PoseStamped>("/fuel/hover_position", 1);
  ego_state_sub = nh_.subscribe<std_msgs::Int32>(
      "/ego/start_flag", 10, &DataManager::ego_state_Callback, this);
  ego_into_sub = nh_.subscribe<std_msgs::Int32>(
      "/ego/into_end_flag", 10, &DataManager::ego_intoend_Callback, this);
}

void DataManager::update() {
  data_input_->getUavState(cur_state_);
  if (cur_state_.connected == false) {
    ROS_ERROR("未更新无人机状态，可能存在断连");
  } else {
    if (cur_state_.mode != "OFFBOARD" && current_fsm_ != FlightState::FINISH) {
      current_fsm_ = FlightState::TAKEOFF;
    }
  }

  if (data_input_->getPlannerState(planner_state_)) {
    ROS_INFO("规划器状态：%s", planner_state_.c_str());
    if (planner_state_.compare("FINISH_PLANNER") == 0) {
      current_fsm_ = FlightState::SPIN1;
    }
  }
data_input_->getOdomInfo(odom_);
  if (data_input_->getPlanState(desire_uav_state_) &&
      (last_desire_uav_state_.pos(0) != desire_uav_state_.pos(0) ||
       last_desire_uav_state_.pos(1) != desire_uav_state_.pos(1) ||
       last_desire_uav_state_.pos(2) != desire_uav_state_.pos(2))) {
    plan_queue_.push(desire_uav_state_);
    last_desire_uav_state_ = desire_uav_state_;
  }
  ROS_WARN("队列大小：%d", plan_queue_.size());

  switch (current_fsm_) {
    case FlightState::TAKEOFF:
      takeoff();
      ROS_WARN("current_fsm_：：TAKEOFF");
      break;
    case FlightState::SPIN:
      spin();
      ROS_WARN("current_fsm_：：SPIN");
      break;
    case FlightState::FOLLOW_TRAJECTORY:
      Execute();
      ROS_WARN("current_fsm_：：FOLLOW_TRAJECTORY");
      break;
    case FlightState::SPIN1:
      spin1();
      ROS_WARN("current_fsm_：：spin1");
      break;
    case FlightState::Landing:
      landing();
      ROS_WARN("current_fsm_：：Landing");
      break;

    default:
      break;
  }
}
void DataManager::landing() {
  data_output_->sendLandingCommand();
  current_fsm_ = FlightState::FINISH;
}

/*
void DataManager::spin() {
  //data_output_->sendTriggerSignal();
  if (!plan_queue_.empty() && is_spin_success_) {
    current_fsm_ = FlightState::FOLLOW_TRAJECTORY;
  }
  // 取消以下注释会去掉自旋部分，悬停在起飞点，等待探索轨迹
  // else {
  //   UavState takeoff_state;
  //   takeoff_state.pos = Eigen::Vector3d(0, 0, 1.5);
  //   takeoff_state.yaw = 0;
  //   data_output_->sendDesireState(takeoff_state);
  // data_output_->sendTriggerSignal();
  // }
  // return;

  double T = 15;  // 自旋速度
  //   mavros_msgs::State cur_state;
  //   data_input_->getUavState(cur_state);
  data_input_->getOdomInfo(odom_);
  UavState takeoff_state;
  double yaw = (ros::Time::now() - start_time_).toSec() * T * M_PI / 180.0;
  takeoff_state.yaw = yaw;
  takeoff_state.q.w() = cos(yaw / 2.0);
  takeoff_state.q.x() = 0.0;
  takeoff_state.q.y() = 0.0;
  takeoff_state.q.z() = sin(yaw / 2.0);
  takeoff_state.pos = Eigen::Vector3d(0, 0, 1.2);  // 原：odom_.pos;
  if ((ros::Time::now() - start_time_).toSec() * T >= 360) {
    is_spin_success_ = true;
    data_output_->sendTriggerSignal();  // 触发feul启动，开始生成探索轨迹
  }

  data_output_->sendDesireState(takeoff_state);
}

*/

void DataManager::spin() {
  if (!plan_queue_.empty() && is_spin_success_) {
    current_fsm_ = FlightState::FOLLOW_TRAJECTORY;
  }
  double T = 20;  // 自旋速度
  // mavros_msgs::State cur_state;
  // data_input_->getUavState(cur_state);
  data_input_->getOdomInfo(odom_);
  UavState takeoff_state;
  double yaw = ((ros::Time::now() - start_time_).toSec() * T * M_PI / 180.0);

  if ((ros::Time::now() - start_time_).toSec() * T >= 360) {
    yaw = odom_.yaw;
  }
  // double yaw = odom_.yaw;     //悬停代码
  ROS_INFO("当前旋转角度和当前角度：%f,%f",
           (ros::Time::now() - start_time_).toSec() * T, yaw);
  takeoff_state.q.w() = cos(yaw / 2.0);  // 下面的四元数没有用到，用到的是yaw
  takeoff_state.q.x() = 0.0;
  takeoff_state.q.y() = 0.0;
  takeoff_state.q.z() = sin(yaw / 2.0);
  takeoff_state.pos = odom_.pos;
  takeoff_state.yaw = yaw;
  if ((ros::Time::now() - start_time_).toSec() * T >= 360) {
    is_spin_success_ = true;
    data_output_->sendTriggerSignal();
  }
  // is_spin_success_ = true;      //悬停代码
  // data_output_->sendTriggerSignal();

  if (ego_intofinish_flag.data == 1)
    data_output_->sendDesireState(takeoff_state);
}


void DataManager::ego_state_Callback(const std_msgs::Int32::ConstPtr &msg) {
  ego_start_flag = *msg;
}

void DataManager::spin1() {
  ROS_INFO("探索结束，悬停等待");
  data_input_->getOdomInfo(odom_);
  if (ego_start_flag.data == 0) {
    fuel_stop_flag.data = 1;
    for (int i = 0; i < 10; i++) spin_flag_pub.publish(fuel_stop_flag);
    geometry_msgs::PoseStamped spin_pos_pub;
    spin_pos_pub.pose.position.x = last_desire_uav_state_.pos(0);
    spin_pos_pub.pose.position.y = last_desire_uav_state_.pos(1);
    spin_pos_pub.pose.position.z = last_desire_uav_state_.pos(2);
    spin_pos_pub.pose.orientation.x = last_desire_uav_state_.q.x();
    spin_pos_pub.pose.orientation.y = last_desire_uav_state_.q.y();
    spin_pos_pub.pose.orientation.z = last_desire_uav_state_.q.z();
    spin_pos_pub.pose.orientation.w = last_desire_uav_state_.q.w();
    spin_position_pub.publish(spin_pos_pub);
    data_output_->sendDesireState(last_desire_uav_state_);
    start_time_ = ros::Time::now();
    ego_start_flag.data = 1;
  } else {
    ROS_INFO("fuel停止发点");
  }
}

void DataManager::ego_intoend_Callback(const std_msgs::Int32::ConstPtr &msg) {
  ego_intofinish_flag = *msg;
  ROS_INFO("fuel_start_flag:%d", ego_intofinish_flag.data);
}

/*
void DataManager::takeoff() {
  data_input_->getOdomInfo(odom_);
  mavros_msgs::State cur_state;
  data_input_->getUavState(cur_state);
  UavState takeoff_state;
  takeoff_state.pos = Eigen::Vector3d(0, 0, 1.2);  // 原：Eigen::Vector3d(odom_.pos(0), odom_.pos(1), 1.5);
  takeoff_state.yaw = 0;
  data_output_->sendDesireState(takeoff_state);
  if (SIM_OR_REAL == 0) {
    if (cur_state_.armed != true) {
      data_output_->sendArmingCommand();
    }
    if (cur_state_.mode != "OFFBOARD") {
      data_output_->sendOffboradCommand();
    }

  } else {
    ROS_WARN("实际飞行，需要手动解锁并切换OFFBOARD模式");
  }

  if ((odom_.pos - takeoff_state.pos).norm() < 0.2) {
    ROS_INFO("起飞成功");
    current_fsm_ = FlightState::SPIN;
    start_time_ = ros::Time::now();
  }
}
*/
void DataManager::takeoff() {
  data_input_->getOdomInfo(odom_);
  mavros_msgs::State cur_state;
  data_input_->getUavState(cur_state);
  UavState takeoff_state;
  //  takeoff_state.pos = Eigen::Vector3d(odom_.pos(0), odom_.pos(1), 2);
  takeoff_state.pos = Eigen::Vector3d(odom_.pos(0), odom_.pos(1), odom_.pos(2));
  data_output_->sendDesireState(takeoff_state);
  if (SIM_OR_REAL == 0) {
    if (cur_state_.armed != true) {
      data_output_->sendArmingCommand();
    }
    if (cur_state_.mode != "OFFBOARD") {
      data_output_->sendOffboradCommand();
    }

  } else {
    ROS_WARN("实际飞行，需要手动解锁并切换OFFBOARD模式");
  }

  if ((odom_.pos - takeoff_state.pos).norm() < 0.5) {
    ROS_INFO("起飞成功");
    current_fsm_ = FlightState::SPIN;
    start_time_ = ros::Time::now();
  }
}


void DataManager::Execute() {
  mavros_msgs::State cur_state;
  data_input_->getUavState(cur_state);
  data_input_->getOdomInfo(odom_);


  if (plan_queue_.empty()) {
    ROS_INFO("规划结束或没有数据");
    // current_fsm_ = FlightState::SPIN;
    data_output_->sendDesireState(last_desire_uav_state_);
    start_time_ = ros::Time::now();
    return;
  }
  // control controller;
  // controller.getVelControl();
  // data_output_->sendDesireVel();
  
  UavState cur_uav_state = plan_queue_.front();
  if ((cur_uav_state.pos - odom_.pos).norm() < 0.3) {
    plan_queue_.pop();
  }

  ROS_INFO("cur_uav_state: x = %.2f m, y = %.2f m, z = %.2f m",
           cur_uav_state.pos(0), cur_uav_state.pos(1), cur_uav_state.pos(2));
  
  if (ego_intofinish_flag.data == 1)
    data_output_->sendDesireState(cur_uav_state);

  //   } else {
  //     ROS_INFO("没有接收到轨迹");
  //     // current_fsm_ = FlightState::SPIN;
  //   }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "autoPlanner");
  setlocale(LC_ALL, "");
  ros::NodeHandle nh;
  std::shared_ptr<DataManager> data_manager = std::make_shared<DataManager>(nh);
  ros::Rate rate(150.0);
  ROS_INFO("启动");
    while (ros::ok()) {
    while (ros::ok() && (data_manager->ego_start_flag.data == 0) &&
           (data_manager->ego_intofinish_flag.data == 1)) {
      // ROS_INFO("YUZHIDI");
      // ROS_INFO("fuel_start_flag:%d", data_manager->ego_intofinish_flag.data);
      data_manager->update();
      ros::spinOnce();
      rate.sleep();
    }
    ros::spinOnce();
    rate.sleep();
  }

}
