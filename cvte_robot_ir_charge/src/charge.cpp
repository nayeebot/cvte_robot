#include <condition_variable>
#include <optional>
#include <queue>
#include <thread>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/UInt8.h"

sensor_msgs::Range rang_msg;

class IrCharge {
 public:
  IrCharge() {
    ir_request_pub =
        node.advertise<std_msgs::UInt8>("/auto_charge/pile_request", 1000);
    twist_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ir_position_sub = node.subscribe("/auto_charge/ir_position", 1000,
                                     &IrCharge::irPositionCallback, this);
    feedback_sub = node.subscribe("/auto_charge/charge_feedback", 1000,
                                  &IrCharge::chargeFeedbackCallback, this);
    find_pile = std::thread(std::bind(&IrCharge::find_pile_thread, this));
  }

  ~IrCharge() { find_pile.join(); }

 private:
  ros::NodeHandle node;
  ros::Subscriber ir_position_sub;
  ros::Subscriber feedback_sub;
  ros::Publisher ir_request_pub;
  ros::Publisher twist_pub;
  std::thread find_pile;
  // std::optional<sensor_msgs::Range> range;
  std::optional<std_msgs::UInt8> feedback;

  std::queue<sensor_msgs::Range> queue_;
  std::mutex mutex_;
  std::condition_variable cond_var_;

  void find_pile_thread() {
    std_msgs::UInt8 msg;
    ir_request_pub.publish(msg);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::optional<sensor_msgs::Range> last_msg;
    while (1) {
      geometry_msgs::Twist twist;
      twist.linear.x = 0;
      twist.angular.z = -0.3;
      std::unique_lock<std::mutex> lock(mutex_);
      if (!cond_var_.wait_for(lock, std::chrono::milliseconds(100),
                              [this] { return !queue_.empty(); })) {
        ir_request_pub.publish(msg);
        twist_pub.publish(twist);
        continue;
      }
      auto msg = queue_.front();
      queue_.pop();
      if (!last_msg) {
        last_msg = msg;
        continue;
      }

      if (std::abs(msg.range) < 0.4 ||
          (msg.range * last_msg.value().range < 0)) {
        break;
      }
      twist_pub.publish(twist);
    }

    uint8_t count = 50;
    while (count--) {
      std_msgs::UInt8 msg;
      ir_request_pub.publish(msg);
      geometry_msgs::Twist twist;
      twist.linear.x = -0.05;
      twist_pub.publish(twist);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    sensor_msgs::Range latest_msg;
    while (1) {
      std::unique_lock<std::mutex> lock(mutex_);
      if (cond_var_.wait_for(lock, std::chrono::milliseconds(100),
                             [this] { return !queue_.empty(); })) {
        latest_msg = queue_.front();
        queue_.pop();
      }

      geometry_msgs::Twist twist;
      if (latest_msg.range < 0) {
        twist.linear.x = 0;
        twist.angular.z = -0.1;
      } else if (latest_msg.range > 0) {
        twist.linear.x = 0;
        twist.angular.z = 0.1;
      } else {
        twist.linear.x = -0.05;
        twist.angular.z = 0;
      }
      twist_pub.publish(twist);

      if (feedback.has_value() &&
          (feedback.value().data == 1 || feedback.value().data == 2)) {
        break;
      }
    }
  }

  void irPositionCallback(sensor_msgs::Range msg) {
    if (msg.range < -1 || msg.range > 1) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push(msg);
    cond_var_.notify_one();
  }

  void chargeFeedbackCallback(std_msgs::UInt8 msg) { feedback = msg; }
};

int main(int argc, char** argv) {
  rang_msg.range = -2;
  ros::init(argc, argv, "auto_charge");
  ros::NodeHandle node;
  IrCharge ircharge;
  ros::spin();
}