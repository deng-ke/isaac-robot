#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "engine/alice/alice_codelet.hpp"
#include "messages/messages.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/constants.hpp"
#include "engine/core/logger.hpp"

namespace isaac { namespace rosbridge {

// replacement of DifferentialBaseOdometry
class OdomRosBridge : public alice::Codelet
{
 public:
  OdomRosBridge();
  virtual ~OdomRosBridge();

  void start () override;
  void tick  () override;
  void stop  () override;

  ISAAC_PROTO_RX(StateProto, cmd);

  ISAAC_PROTO_TX(Odometry2Proto, odom);

  ISAAC_PARAM(int, publisher_queue_size, 1000);
  ISAAC_PARAM(int, subscriber_queue_size, 1000);
  ISAAC_PARAM(std::string, publisher_channel_name, "cmd_vel");
  ISAAC_PARAM(std::string, subscriber_channel_name, "odom");

  ISAAC_POSE2(robot, odom);

 private:
  struct RosOdometryData;
  std::unique_ptr<RosOdometryData> odom_data_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::OdomRosBridge);