#include <cmath>
#include <iomanip>
#include <iostream>

#include "Bt/helper.hpp"
#include "Fal/Service/ChargeHandler.hpp"
#include "InfraredPile.hpp"
#include "pose2d/pose2d.hpp"
#include "std_msgs/msg/string.hpp"

const static uint8_t kBounce = 0;
const static uint8_t kAutomatic = 1;
const static uint8_t kHandPush = 2;
const static uint8_t kFeedbackUnkonw = 255;

NodeStatus InfraredPile::tick() {
  using namespace CVTE_BABOT;
  namespace co = std::chrono;
  is_rotate_end_failed_ = false;
  co::system_clock::time_point deadline =
      co::system_clock::now() + co::seconds(60);

  sendPileRequest();
  range_interval_time_ = time(NULL);
  last_yaw_ = INIT_ANGLE;
  sleep(1);  // 保证数据先到达
  {
    std::lock_guard<std::mutex> lock(opt_variation_mutex_);
    opt_variation_yaw_.reset();
  }

  setFixMoveState(FixMoveState::unfix);
  while (true) {
    LOG_EVERY_N(INFO, 10) << "InfraredPile";
    if (charge_feedback_ == kAutomatic || charge_feedback_ == kHandPush) {
      LOG(INFO) << "对桩成功";
      break;
    }

    if (co::system_clock::now() > deadline) {
      LOG(ERROR) << "机器人充电失败! 原因: 红外对桩超时.";
      return BT::NodeStatus::FAILURE;
    }

    if (is_rotate_end_failed_) {
      LOG(ERROR) << "机器人充电失败! 原因: 无法获取红外数据.";
      return BT::NodeStatus::FAILURE;
    }

    // 大于2s开始无数据自旋
    if ((time(NULL) - range_interval_time_) > 2) {
      sendPileRequest();
      swingForInfrared();
    }

    if (isEqualState(FixMoveState::fixing)) {
      setFixMoveVel();
      LOG(INFO) << "start fix move..........";
      if ((time(NULL) - start_fix_time_) > 5) {
        setFixMoveState(FixMoveState::rotate);
        LOG(INFO) << "................ end fix move, start fix rotate";
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  return NodeStatus::SUCCESS;
}

void InfraredPile::setFixMoveState(FixMoveState state) {
  std::lock_guard<std::mutex> lock(fix_move_state_mutex_);
  std::cout << "old:" << static_cast<int>(fix_move_state_)
            << " new:" << static_cast<int>(state) << std::endl;
  fix_move_state_ = state;
}

bool InfraredPile::isEqualState(FixMoveState state) {
  std::lock_guard<std::mutex> lock(fix_move_state_mutex_);
  return fix_move_state_ == state;
}

void InfraredPile::init() {
  using namespace std_msgs::msg;
  auto msg_manager_ = Ros2MsgManager::getInstance();

  pile_request_client_ =
      msg_manager_->getNode()->create_client("/auto_charge/pile_request");

  pile_range_iter_ =
      msg_manager_
          ->getRos2MsgTable<sensor_msgs::msg::Range>("/auto_charge/ir_position")
          ->addCallback(
              std::bind(&InfraredPile::updateRangePostion, this, sph::_1));
  charge_fb_iter_ =
      msg_manager_->getRos2MsgTable<UInt8>("/auto_charge/charge_feedback")
          ->addCallback(
              std::bind(&InfraredPile::chargeFeedbackCallback, this, sph::_1));
  odom_sub_ =
      msg_manager_->getRos2MsgTable<nav_msgs::msg::Odometry>("/odom")
          ->addCallback(std::bind(&InfraredPile::updateOdom, this, sph::_1));
}

void InfraredPile::updateOdom(nav_msgs::msg::Odometry odom_msg) {
  current_yaw_ = CVTE_BABOT::AngleCalculate::quaternionToYaw(
      {odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y,
       odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w});
}

bool InfraredPile::sendPileRequest() {
  uint16_t pile_id = 0;
  chassis_interfaces::srv::PileRequest::Request request;
  chassis_interfaces::srv::PileRequest::Response response;
  request.pile_id = pile_id;
  if (!pile_request_client_
           ->async_send_request<chassis_interfaces::srv::PileRequest>(
               request, response, std::chrono::seconds(4))) {
    LOG(ERROR) << "sendPileRequest wait_for_service timeout";
    return false;
  }
  return true;
}

void InfraredPile::chargeFeedbackCallback(std_msgs::msg::UInt8 msg) {
  charge_feedback_ = msg.data;
  LOG(INFO) << "charge_feedback_: " << charge_feedback_;
}

void InfraredPile::updateDockPosition(double data) {
  range_interval_time_ = time(NULL);
  last_yaw_ = INIT_ANGLE;

  {
    std::lock_guard<std::mutex> lock(opt_variation_mutex_);
    opt_variation_yaw_.reset();
  }

  // 1:第一次收到中心速度开始定距移动
  if (fabs(data - 0.00) < 0.01 && isEqualState(FixMoveState::unfix)) {
    {
      LOG(INFO) << "开始定距移动";
      setFixMoveState(FixMoveState::fixing);
      start_fix_time_ = time(NULL);
    }
  }

  // 2:旋转寻中
  if (isEqualState(FixMoveState::rotate)) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    if (data > 0) {
      msg.angular.z = fix_rotate_speed_;
    } else if (data < 0) {
      msg.angular.z = -1 * fix_rotate_speed_;
    } else {
      LOG(INFO) << "收到中心区域，寻中结束";
      setFixMoveState(FixMoveState::fixed);
      return;
    }

    // 如果慢速旋转时，越过中心区域立即停止旋转
    if (fabs(data + last_data_) < (fabs(data) + fabs(last_data_))) {
      LOG(INFO) << "寻中结束"
                << "last_data:" << last_data_ << " current_data:" << data;
      setFixMoveState(FixMoveState::fixed);
      return;
    }

    last_data_ = data;
    Ros2MsgManager::getInstance()->publishMsg("/dangerous_zone_vel", msg);
    return;
  }
  last_data_ = data;

  double w = 0, v = 0;
  getSpeed(data, v, w);
  setBaseVel(v, w, true);
}

void InfraredPile::setBaseVel(double v, double w, bool update_direction) {
  LOG_EVERY_N(INFO, 10) << "V = " << v << " W = " << w;
  geometry_msgs::msg::Twist msg;
  msg.linear.x = v;
  msg.angular.z = w;
  Ros2MsgManager::getInstance()->publishMsg("/ir_vel", msg);

  if (update_direction) {
    last_angular_vel_ = w;
  }
}

void InfraredPile::setFixMoveVel() {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = fix_move_speed_;
  msg.angular.z = 0;
  LOG_EVERY_N(INFO, 10) << "fixmove v=" << msg.linear.x
                        << " w=" << msg.angular.z;
  Ros2MsgManager::getInstance()->publishMsg("/dangerous_zone_vel", msg);
}

void InfraredPile::updateRangePostion(sensor_msgs::msg::Range pile_location) {
  updateDockPosition(pile_location.range);
}

void InfraredPile::getSpeed(float input_state, double& output_v,
                            double& output_w) {
  BaseSpeed speed;
  speed = getPositionSpeed(input_state);

  double tmp_left = speed.right_speed * speed_multiple_ * (-1);
  double tmp_right = speed.left_speed * speed_multiple_ * (-1);

  output_v = (tmp_left + tmp_right) / 2;
  output_w = (tmp_right - tmp_left) / wheel_spacing_;

  LOG(INFO) << "input_state:" << input_state << " [" << speed.right_speed << ":"
            << speed.left_speed << "]    "
            << " V = " << output_v << " W = " << output_w;
}

InfraredPile::BaseSpeed InfraredPile::getPositionSpeed(double input_ir) {
  LOG(INFO) << "input_ir:" << input_ir;
  if (fabs(input_ir) > 1) {
    LOG(ERROR) << "input error";
    return BaseSpeed(0, 0);
  }

  if (input_ir < 0) {
    return BaseSpeed(fabs(input_ir) * high_speed_, fabs(input_ir) * low_speed_);
  } else if (input_ir > 0) {
    return BaseSpeed(fabs(input_ir) * low_speed_, fabs(input_ir) * high_speed_);
  } else {
    return BaseSpeed(center_speed_, center_speed_);
  }
}

void InfraredPile::pubRoateSPeed() {
  if (last_angular_vel_ <= 0.0) {
    setBaseVel(0.0, rotate_speed_, false);
    LOG(INFO) << "last_angular_vel:" << last_angular_vel_
              << "pub rotate speed :" << rotate_speed_;
  } else {
    setBaseVel(0.0, -1 * rotate_speed_, false);
    LOG(INFO) << "last_angular_vel:" << last_angular_vel_
              << "pub rotate speed: -" << rotate_speed_;
  }
}

void InfraredPile::swingForInfrared() {
  auto isEqualTwoDouble = [&](double x, const double y,
                              double epsilon = 0.001) {
    double diff = std::fabs(x - y);
    return (diff <= epsilon);
  };

  {
    std::lock_guard<std::mutex> lock(opt_variation_mutex_);
    if (!opt_variation_yaw_.has_value()) {  // 初始化
      opt_variation_yaw_ = swing_angle_;
      last_yaw_ = current_yaw_;
      variation_yaw_ = 0;
      angle_direction_ = (last_data_ >= 0) ? 1 : -1;
    }

    double delta_yaw =
        CVTE_BABOT::AngleCalculate::angle_diff(current_yaw_, last_yaw_);

    variation_yaw_ += delta_yaw;
    last_yaw_ = current_yaw_;

    LOG(INFO) << "last_yaw_:" << last_yaw_ << "  current_yaw:" << current_yaw_
              << "  variation_yaw_:" << variation_yaw_;

    double fabs_var_yaw = std::fabs(variation_yaw_);

    if (fabs_var_yaw >= opt_variation_yaw_.value()) {
      if (isEqualTwoDouble(opt_variation_yaw_.value(), swing_angle_)) {
        opt_variation_yaw_.value() *= 2;
        angle_direction_ *= -1;
        variation_yaw_ = 0;
      } else {
        is_rotate_end_failed_ = true;
      }
    }
  }

  pubSwingSPeed(angle_direction_);
  LOG_EVERY_N(INFO, 3) << "无数据自旋 last_data:" << last_data_;
}

void InfraredPile::pubSwingSPeed(double direction) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = direction * fix_rotate_speed_;
  Ros2MsgManager::getInstance()->publishMsg("/dangerous_zone_vel", msg);
}
