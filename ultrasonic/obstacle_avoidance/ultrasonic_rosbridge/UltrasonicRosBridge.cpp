#include "UltrasonicRosBridge.hpp"
#include "engine/gems/sight/sight.hpp"
#include "sensor_msgs/Range.h"
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
  constexpr int kRangeSize = 1441;
  constexpr float min_range = 0.2;
  constexpr float max_range = 10;
  constexpr float robot_radius = 0.35;
  constexpr float angle_min_ = -3.141592654;
  constexpr float angle_increment_ = 0.00436332309619;
  
  class CallbackFunctor {
   public:
    explicit CallbackFunctor(UltrasonicRosBridge* bridge) {
      bridge_ = bridge;
    }
    CallbackFunctor(const CallbackFunctor &) = default;
    ~CallbackFunctor() = default;
  
    void operator() (const sensor_msgs::Range::ConstPtr &msg) {
      if (!msg) {
        std::cout << "Empty Ultrasonic Message!" << std::endl;
        return;
      }
      auto _msg = bridge_->tx_sonar_scan().initProto();
      do {
        _msg.setInvalidRangeThreshold(min_range);
        _msg.setOutOfRangeThreshold(max_range);
        auto _ranges = _msg.initRanges(kRangeSize);
        auto _angles = _msg.initAngles(kRangeSize);
        for(int i=0; i<kRangeSize; ++i){
          _ranges.set(i, 10);
  	  _angles.set(i, angle_min_+i*angle_increment_);
        }
  
        if(msg->header.frame_id=="sonar1") {
          for(int i=660; i<780; i++) {
            _ranges.set(i, msg->range+robot_radius);
          }
        }
        else if(msg->header.frame_id=="sonar2") {
          for(int i=420; i<540; i++) {
            _ranges.set(i, msg->range+robot_radius);
          }
        }
        else if(msg->header.frame_id=="sonar3") {
          for(int i=180; i<300; i++) {
            _ranges.set(i, msg->range+robot_radius);
          }
        }
        else if(msg->header.frame_id=="sonar4") {
          for(int i=0; i<60; i++) {
            _ranges.set(i, msg->range+robot_radius);
          }
          for(int i=1381; i<1441; i++) {
            _ranges.set(i, msg->range+robot_radius);
          }
        }
        else if(msg->header.frame_id=="sonar5") {
          for(int i=1140; i<1260; i++) {
            _ranges.set(i, msg->range+robot_radius);
          }
        }
        else if(msg->header.frame_id=="sonar6") {
          for(int i=900; i<1020; i++) {
            _ranges.set(i, msg->range+robot_radius);
          }
        }
      } while (0);
      bridge_->tx_sonar_scan().publish(ros::Time(msg->header.stamp).toNSec());
    }
   private:
    UltrasonicRosBridge* bridge_;
  };
}  // namespace

struct UltrasonicRosBridge::RosUltrasonicData {
  ros::NodeHandle node_;
  ros::Subscriber sub_[6];
  ros::CallbackQueue callbackQueue_;
};

UltrasonicRosBridge::UltrasonicRosBridge() {
}

UltrasonicRosBridge::~UltrasonicRosBridge() {
}

void UltrasonicRosBridge::start() {
  ros::M_string _args;
  if (!ros::isInitialized()) {
    ros::init(_args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  ultrasonic_data_ = std::make_unique<RosUltrasonicData>();
  ultrasonic_data_->node_.setCallbackQueue(&(ultrasonic_data_->callbackQueue_));
  ultrasonic_data_->sub_[0] = ultrasonic_data_->node_.subscribe<sensor_msgs::Range>(
      get_sonar_topic1(), get_subscriber_queue_size(),
      CallbackFunctor(this));
  ultrasonic_data_->sub_[1] = ultrasonic_data_->node_.subscribe<sensor_msgs::Range>(
      get_sonar_topic2(), get_subscriber_queue_size(),
      CallbackFunctor(this));
  ultrasonic_data_->sub_[2] = ultrasonic_data_->node_.subscribe<sensor_msgs::Range>(
      get_sonar_topic3(), get_subscriber_queue_size(),
      CallbackFunctor(this));
  ultrasonic_data_->sub_[3] = ultrasonic_data_->node_.subscribe<sensor_msgs::Range>(
      get_sonar_topic4(), get_subscriber_queue_size(),
      CallbackFunctor(this));
  ultrasonic_data_->sub_[4] = ultrasonic_data_->node_.subscribe<sensor_msgs::Range>(
      get_sonar_topic5(), get_subscriber_queue_size(),
      CallbackFunctor(this));
  ultrasonic_data_->sub_[5] = ultrasonic_data_->node_.subscribe<sensor_msgs::Range>(
      get_sonar_topic6(), get_subscriber_queue_size(),
      CallbackFunctor(this));

  tickPeriodically();
}

void UltrasonicRosBridge::tick() {
  if (ros::ok()) {
    ultrasonic_data_->callbackQueue_.callAvailable();
  }
}

void UltrasonicRosBridge::stop() {
  for(int i=0; i<6; i++) {
    ultrasonic_data_->sub_[i].shutdown();
  }
  ultrasonic_data_.reset();
}

} // namespace rosbridge
}  // namespace isaac
