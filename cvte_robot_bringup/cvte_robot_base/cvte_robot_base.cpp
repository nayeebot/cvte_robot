#include "eigen3/Eigen/Eigen"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Int64MultiArray.h"
#include "tf/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class OdomPublisher {
 public:
  OdomPublisher() {
    pub = node.advertise<nav_msgs::Odometry>("odom", 1000);
    sub = node.subscribe("encoding", 1000, &OdomPublisher::EncodingCallback,
                         this);
  }

  void EncodingCallback(const std_msgs::Int64MultiArray& msg) {
    /***********配置***********/
    const double left_wheel_diameter = 0.09;   // 左轮直径
    const double right_wheel_diameter = 0.09;  // 右轮直径
    const double wheel_distance = 0.344;       // 左右轮间距
    const int64_t encoding_circle = 51814;  // 轮子转动一圈，编码器数值增量

    static Eigen::Vector3d curr_pos_ = {0.0, 0.0, 0.0};
    static double curr_theta_ = 0.0;
    /****************************/

    static double _last_stamp = -1.0;
    static Eigen::Vector2d _last_encode = {0.0, 0.0};

    double stamp = ros::Time::now().toSec();

    int64_t encoding_num_left = msg.data[0];
    int64_t encoding_num_right = msg.data[1];

    double encoding_left =
        M_PI * left_wheel_diameter * encoding_num_left / encoding_circle;
    double encoding_right =
        M_PI * right_wheel_diameter * encoding_num_right / encoding_circle;

    Eigen::Vector2d encode{encoding_left, encoding_right};

    if (_last_stamp < 0) {
      _last_encode = encode;
      _last_stamp = stamp;
      curr_pos_.x() = 0.0;
      curr_pos_.y() = 0.0;
      curr_pos_.z() = 0.0;
      curr_theta_ = 0.0;
      return;
    }
    double delta_t = stamp - _last_stamp;

    double delta_left = encode.x() - _last_encode.x();
    double delta_right = encode.y() - _last_encode.y();

    double delta_x = (delta_left + delta_right) * 0.5;
    double vel_x = delta_x / delta_t;

    double delta_theta = (delta_right - delta_left) / wheel_distance;
    double angular_z = delta_theta / delta_t;

    _last_stamp = stamp;
    _last_encode = encode;

    Eigen::Vector3d delta_trans = {0.0, 0.0, 0.0};
    if (fabs(delta_theta) > 0.01) {
      // delta_x是弧长，根据角度得到半径，转换成弦长再进行分解；
      double radius = delta_x / delta_theta;
      delta_trans.x() = radius * std::sin(delta_theta);
      delta_trans.y() = radius * (1 - std::cos(delta_theta));
    } else {
      delta_trans.x() = delta_x;
    }

    curr_theta_ += delta_theta;

    Eigen::Quaterniond q(cos(curr_theta_ * 0.5), 0.0, 0.0,
                         sin(curr_theta_ * 0.5));  // 顺序:w,x,y,z
    q.normalize();

    Eigen::Matrix3d orientation = q.toRotationMatrix();
    curr_pos_ = orientation * delta_trans + curr_pos_;

    nav_msgs::Odometry odom_pose_pub_msgs;
    odom_pose_pub_msgs.header.stamp.sec = stamp;
    odom_pose_pub_msgs.header.stamp.nsec =
        (stamp - odom_pose_pub_msgs.header.stamp.sec) * 1e9;
    odom_pose_pub_msgs.header.frame_id = "odom";
    odom_pose_pub_msgs.pose.pose.position.x = curr_pos_.x();
    odom_pose_pub_msgs.pose.pose.position.y = curr_pos_.y();
    odom_pose_pub_msgs.pose.pose.position.z = 0.0;
    odom_pose_pub_msgs.pose.pose.orientation.x = q.x();
    odom_pose_pub_msgs.pose.pose.orientation.y = q.y();
    odom_pose_pub_msgs.pose.pose.orientation.z = q.z();
    odom_pose_pub_msgs.pose.pose.orientation.w = q.w();

    odom_pose_pub_msgs.twist.twist.linear.x = vel_x;
    odom_pose_pub_msgs.twist.twist.linear.y = 0.0;
    odom_pose_pub_msgs.twist.twist.linear.z = 0.0;
    odom_pose_pub_msgs.twist.twist.angular.x = 0.0;
    odom_pose_pub_msgs.twist.twist.angular.y = 0.0;
    odom_pose_pub_msgs.twist.twist.angular.z = angular_z;

    pub.publish(odom_pose_pub_msgs);

    tf::Transform transform;
    transform.setOrigin(
        tf::Vector3(curr_pos_.x(), curr_pos_.y(), 0.0));  // 平移
    transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));  // 旋转

    broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
  }

 private:
  ros::NodeHandle node;
  ros::Subscriber sub;
  ros::Publisher pub;
  tf::TransformBroadcaster broadcaster;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cvte_robot_base");
  tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped ts;
  ts.header.seq = 100;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = "base_link";
  ts.child_frame_id = "laser";
  ts.transform.translation.x = 0.078;
  ts.transform.translation.y = 0.0;
  ts.transform.translation.z = 0.0;
  tf2::Quaternion qtn;
  qtn.setRPY(0, 0, -M_PI_2);
  ts.transform.rotation.x = qtn.getX();
  ts.transform.rotation.y = qtn.getY();
  ts.transform.rotation.z = qtn.getZ();
  ts.transform.rotation.w = qtn.getW();

  broadcaster.sendTransform(ts);

  OdomPublisher odom_publisher;
  ros::spin();
}