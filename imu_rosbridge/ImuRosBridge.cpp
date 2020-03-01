#include "ImuRosBridge.hpp"
#include "engine/gems/sight/sight.hpp"
#include "sensor_msgs/Imu.h"
#include "messages/math.hpp"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace isaac {
namespace rosbridge {
namespace {
  class CallbackFunctor {
   public:
    explicit CallbackFunctor(ImuRosBridge* bridge) {
      bridge_ = bridge;
    }
    CallbackFunctor(const CallbackFunctor &) = default;
    ~CallbackFunctor() = default;
  
    void operator() (const sensor_msgs::Imu::ConstPtr &msg) {
      if (!msg) {
        LOG_WARNING("Empty Imu Message!");
        return;
      }
      auto _msg = bridge_->tx_imu().initProto();
      do {
        _msg.setLinearAccelerationX(msg->linear_acceleration.x);
        _msg.setLinearAccelerationY(msg->linear_acceleration.y);
        _msg.setLinearAccelerationZ(msg->linear_acceleration.z);
        _msg.setAngularVelocityX(msg->angular_velocity.x);
        _msg.setAngularVelocityY(msg->angular_velocity.y);
        _msg.setAngularVelocityZ(msg->angular_velocity.z);
      } while (0);
      bridge_->tx_imu().publish(ros::Time(msg->header.stamp).toNSec());
    }
   private:
    ImuRosBridge* bridge_;
  };
}  // namespace

struct ImuRosBridge::RosImuData {
  ros::NodeHandle node_;
  ros::Subscriber sub_;
  ros::CallbackQueue callbackQueue_;
};

ImuRosBridge::ImuRosBridge() {
}

ImuRosBridge::~ImuRosBridge() {
}

void ImuRosBridge::start() {
  ros::M_string _args;
  if (!ros::isInitialized()) {
    ros::init(_args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  imu_data_ = std::make_unique<RosImuData>();
  imu_data_->node_.setCallbackQueue(&(imu_data_->callbackQueue_));
  imu_data_->sub_ = imu_data_->node_.subscribe<sensor_msgs::Imu>(
      get_imu_topic(), get_subscriber_queue_size(),
      CallbackFunctor(this));
  tickPeriodically();
}

void ImuRosBridge::tick() {
  if (ros::ok()) {
    imu_data_->callbackQueue_.callAvailable();
  }
}

void ImuRosBridge::stop() {
  imu_data_->sub_.shutdown();
  imu_data_.reset();
}

} // namespace rosbridge
} // namespace isaac 
