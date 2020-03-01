#include "OdomRosBridge.hpp"

#include "messages/state/differential_base.hpp"
#include "engine/gems/sight/sight.hpp"
#include "engine/gems/state/io.hpp"
#include "nav_msgs/Odometry.h"
#include "messages/math.hpp"
#include "ros/callback_queue.h"
#include "ros/ros.h"

namespace isaac { namespace rosbridge {
namespace {

class CallbackFunctor {
 public:
  explicit CallbackFunctor(OdomRosBridge* bridge)
      : bridge_(bridge) {}
  CallbackFunctor(const CallbackFunctor &) = default;
  ~CallbackFunctor() = default;

  /**
     std_msgs/Header header
     uint32 seq
     time stamp
     string frame_id
   string child_frame_id
   geometry_msgs/PoseWithCovariance pose
     geometry_msgs/Pose pose
       geometry_msgs/Point position
         float64 x
         float64 y
         float64 z
       geometry_msgs/Quaternion orientation
         float64 x
         float64 y
         float64 z
         float64 w
     float64[36] covariance
   geometry_msgs/TwistWithCovariance twist
     geometry_msgs/Twist twist
       geometry_msgs/Vector3 linear
         float64 x
         float64 y
         float64 z
       geometry_msgs/Vector3 angular
         float64 x
         float64 y
         float64 z
     float64[36] covariance

     struct Odometry2Proto {
         # The pose of the "robot" relative to the reference odometric frame
         odomTRobot @0: Pose2dProto;
         speed @1: Vector2dProto;
         angularSpeed @2: Float64;
         acceleration @3: Vector2dProto;
         # Contains the name of the odometry frame and the robot frame.
         odometryFrame @4: Text;
         robotFrame @5: Text;
     }
   */
  void operator() (const nav_msgs::Odometry::ConstPtr &msg) {
    // auto _msg = bridge_->tx_odom().initProto();
    do {  // trans sensor_msgs::LaserScan to isaac FlatscanProto
      // _msg.setInvalidRangeThreshold(msg->angle_min);
      // _msg.setOutOfRangeThreshold(msg->angle_max);

      // ...

    } while (0);

    // publish scan data
    bridge_->tx_odom().publish(ros::Time(msg->header.stamp).toNSec());
  }

 private:
  OdomRosBridge* bridge_;
};
}  // namespace

struct OdomRosBridge::RosOdometryData {
  ros::NodeHandle node_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::CallbackQueue callbackQueue_;
};

OdomRosBridge::OdomRosBridge() {
}

OdomRosBridge::~OdomRosBridge() {
}

void OdomRosBridge::start() {
  ros::M_string _args;
  if (!ros::isInitialized()) {
    ros::init(_args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  odom_data_ = std::make_unique<RosOdometryData>();
  odom_data_->node_.setCallbackQueue(&(odom_data_->callbackQueue_));
  odom_data_->pub_ = odom_data_->node_.advertise<geometry_msgs::Twist>(
      get_publisher_channel_name(), get_publisher_queue_size());
  odom_data_->sub_ = odom_data_->node_.subscribe<nav_msgs::Odometry>(
      get_subscriber_channel_name(), get_subscriber_queue_size(),
      CallbackFunctor(this));

  tickPeriodically();
}

void OdomRosBridge::tick() {
  if (ros::ok()) {
    if (rx_cmd().available()) {
      isaac::messages::DifferentialBaseControl _cmd;
      FromProto(rx_cmd().getProto(), _cmd);

      geometry_msgs::Twist _msg;
      // setup message ...
      // tf::Transform _t;
      // tf::poseMsgToTF(, _t);
      odom_data_->pub_.publish(_msg);
    }

    /**
       geometry_msgs/Vector3 linear
         float64 x
         float64 y
         float64 z
       geometry_msgs/Vector3 angular
         float64 x
         float64 y
         float64 z
     */

    // show("scan",
    //      [&](sight::Sop& sop) {
    //        sop.style = sight::SopStyle{"red", "filled"};
    //        sop.add([&](sight::Sop& sop) { // Recursive call
    //                  sop.add(this->position_);
    //                });
    //      });

    odom_data_->callbackQueue_.callAvailable();  // corresponding to ros::spinOnce
  } else {
    LOG_ERROR("An error has occurred within ROS.");
  }
}

void OdomRosBridge::stop() {
  odom_data_->sub_.shutdown();
  odom_data_.reset();
}

} // namespace rosbridge
}  // namespace isaac