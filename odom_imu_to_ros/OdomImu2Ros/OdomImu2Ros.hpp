#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"
#include "Eigen/Eigen"

namespace isaac {
namespace rosbridge {

class OdomImu2Ros: public alice::Codelet {
 public:
  OdomImu2Ros();
  virtual ~OdomImu2Ros();

  void start() override;
  void tick() override;
  void stop() override;

  ISAAC_PROTO_RX(Odometry2Proto, in_odom);
  ISAAC_PROTO_RX(ImuProto, in_imu);

  ISAAC_POSE2(world, robot);

  ISAAC_PARAM(int, publisher_queue_size, 1000);
  ISAAC_PARAM(int, subscriber_queue_size, 1000);
  ISAAC_PARAM(std::string, odom_topic, "odom");
  ISAAC_PARAM(std::string, imu_topic, "imu");

 private:
  struct RosOdomData;
  struct RosImuData;
  std::unique_ptr<RosOdomData> odom_data_;
  std::unique_ptr<RosImuData> imu_data_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::OdomImu2Ros);
