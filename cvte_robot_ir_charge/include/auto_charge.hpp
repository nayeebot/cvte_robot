#pragma once
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sensor_msgs/msg/range.hpp>

#include "chassis_interfaces/srv/pile_request.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using namespace BT;

class InfraredPile {
 public:
  InfraredPile(const std::string &name, const NodeConfiguration &config)
      : AsyncActionNode(name, config) {
    init();
  }
  NodeStatus tick() override;
  void halt() override { AsyncActionNode::halt(); };

 private:
  struct BaseSpeed {
    BaseSpeed(double left = 0, double right = 0) {
      left_speed = left;
      right_speed = right;
    }

    double left_speed = 0;
    double right_speed = 0;
  };
  enum class FixMoveState { unfix = 0, fixing, rotate, fixed };

  const double INIT_ANGLE = 100.0;
  time_t range_interval_time_ = 0;
  uint16_t charge_feedback_ = 0;
  double speed_multiple_ = 0.12;
  double wheel_spacing_ = 0.35;
  std::atomic<bool> is_rotate_end_failed_ = false;
  std::atomic<double> current_yaw_ = INIT_ANGLE;
  ZbusClient::SharedPtr pile_request_client_;
  FixMoveState fix_move_state_ = FixMoveState::unfix;
  std::mutex fix_move_state_mutex_;
  time_t start_fix_time_ = 0;
  double last_angular_vel_ = 0.0;  // 上次设备红外速度
  double last_yaw_ = INIT_ANGLE;   // 上次设备角度，单位：rad弧度
  double last_data_ = 100;         // 上次红外数据
  double variation_yaw_ = 0.0;     // 角度差
  double angle_direction_ = 0.0;
  std::optional<double> opt_variation_yaw_;
  std::mutex opt_variation_mutex_;
  // TODO: 可配置
  double high_speed_ = 0.9;
  double low_speed_ = 0.2;
  double center_speed_ = 0.3;
  double rotate_speed_ = 0.3;
  double fix_move_speed_ = -0.05;
  double fix_rotate_speed_ = 0.2;
  double swing_angle_ = M_PI / 9;

 private:
  void init();
  bool sendPileRequest();
  // 无数据需要根据上一次速度反向旋转，红外速度需要更新，逻辑速度使用默认值
  void setBaseVel(double v, double w, bool update_direction = false);
  void setFixMoveVel();
  void getSpeed(float input_state, double &output_v, double &output_w);
  void updateDockPosition(double data);
  void updateRangePostion(sensor_msgs::msg::Range pile_location);
  void chargeFeedbackCallback(std_msgs::msg::UInt8 msg);
  BaseSpeed getPositionSpeed(double input_ir);
  void swingForInfrared();
  void pubRoateSPeed();
  void pubSwingSPeed(double direction);
  void setFixMoveState(FixMoveState state);
  bool isEqualState(FixMoveState state);
  void updateOdom(nav_msgs::msg::Odometry odom_msg);
};
