#include "OdomImu2Ros.hpp"

#include <memory>

#include "engine/core/assert.hpp"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "messages/math.hpp"
#include "ros/callback_queue.h"
#include "ros/ros.h"

namespace isaac {
namespace rosbridge {

namespace {
class CallbackFunctor {
 public:
  explicit CallbackFunctor(OdomImu2Ros* bridge) {
    bridge_ = bridge;
  }
  CallbackFunctor(const CallbackFunctor&) = default;
  ~CallbackFunctor() = default;
 private:
  OdomImu2Ros* bridge_;
};
}  // namespace

struct OdomImu2Ros::RosOdomData {
  ros::NodeHandle odom_node;
  ros::Subscriber odom_sub;
  ros::Publisher odom_pub;
  ros::CallbackQueue odom_callbackQueue;
};
struct OdomImu2Ros::RosImuData {
  ros::NodeHandle imu_node;
  ros::Subscriber imu_sub;
  ros::Publisher imu_pub;
  ros::CallbackQueue imu_callbackQueue;
};

void OdomImu2Ros::start() {
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  odom_data_ = std::make_unique<RosOdomData>();
  odom_data_->odom_node.setCallbackQueue(&(odom_data_->odom_callbackQueue));
  odom_data_->odom_pub = odom_data_->odom_node.advertise<nav_msgs::Odometry>(
      get_odom_topic(), get_publisher_queue_size());

  imu_data_ = std::make_unique<RosImuData>();
  imu_data_->imu_node.setCallbackQueue(&(imu_data_->imu_callbackQueue));
  imu_data_->imu_pub = imu_data_->imu_node.advertise<sensor_msgs::Imu>(
      get_imu_topic(), get_publisher_queue_size());

  tickPeriodically();
}

void OdomImu2Ros::tick() {
  if (ros::ok()) {
    /*process incoming odom*/
    if(rx_in_odom().available()) {
      const auto odom_proto = rx_in_odom().getProto();
      auto odom_t_robot = FromProto(odom_proto.getOdomTRobot());
      auto linear_speed = FromProto(odom_proto.getSpeed());
      //SO2dProto --> rotation_matrix --> quaterniond
      Eigen::Matrix3d m;
      m(0, 0) = odom_proto.getOdomTRobot().getRotation().getQ().getX();
      m(0, 1) = -odom_proto.getOdomTRobot().getRotation().getQ().getY();
      m(1, 0) = odom_proto.getOdomTRobot().getRotation().getQ().getY();
      m(1, 1) = odom_proto.getOdomTRobot().getRotation().getQ().getX();
      m(2, 2) = 1; m(0, 2) = 0; m(1, 2) = 0; m(2, 0) = 0; m(2, 1) = 0;
      Eigen::Quaterniond q;
      q = Eigen::Quaterniond(m);
      //odom to ROS
      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = ros::Time::now();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_footprint";
      odom_msg.pose.pose.position.x = odom_t_robot.translation.x();
      odom_msg.pose.pose.position.y = odom_t_robot.translation.y();
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_msg.pose.pose.orientation.w = q.w();
      odom_msg.twist.twist.linear.x = linear_speed.x();
      odom_msg.twist.twist.linear.y = linear_speed.y();
      odom_msg.twist.twist.angular.z = odom_proto.getAngularSpeed();
      odom_data_->odom_pub.publish(odom_msg);
    }
    /*process incoming imu*/
    if(rx_in_imu().available())  {
      const auto imu_proto = rx_in_imu().getProto();
      //imu to ROS
      sensor_msgs::Imu imu_msg;
      imu_msg.header.stamp = ros::Time::now();
      imu_msg.header.frame_id = "gyro_link";
      imu_msg.angular_velocity.x = imu_proto.getAngularVelocityX();
      imu_msg.angular_velocity.y = imu_proto.getAngularVelocityY();
      imu_msg.angular_velocity.z = imu_proto.getAngularVelocityZ();
      imu_msg.linear_acceleration.x = imu_proto.getLinearAccelerationX();
      imu_msg.linear_acceleration.y = imu_proto.getLinearAccelerationY();
      imu_msg.linear_acceleration.z = -imu_proto.getLinearAccelerationZ();
      imu_data_->imu_pub.publish(imu_msg);
    }
    odom_data_->odom_callbackQueue.callAvailable();
    imu_data_->imu_callbackQueue.callAvailable();
  }
  else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void OdomImu2Ros::stop() {
  odom_data_->odom_pub.shutdown();
  odom_data_->odom_sub.shutdown();
  odom_data_ = nullptr;
  imu_data_->imu_pub.shutdown();
  imu_data_->imu_sub.shutdown();
  imu_data_ = nullptr;
}

OdomImu2Ros::~OdomImu2Ros() {
}

OdomImu2Ros::OdomImu2Ros() {
}

}  // namespace rosbridge
}  // namespace isaac
