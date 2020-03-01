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

class HokuyoRosBridge : public alice::Codelet
{
 public:
  HokuyoRosBridge();
  virtual ~HokuyoRosBridge();

  void start () override;
  void tick  () override;
  void stop  () override;

  ISAAC_PROTO_TX(FlatscanProto, scan);

  // ROS subscriber queue depth
  ISAAC_PARAM(int, subscriber_queue_size, 1000);
  // ROS subscriber channel. Used to receive messagse from ROS
  ISAAC_PARAM(std::string, subscriber_channel_name, "scan");

  ISAAC_POSE2(robot, laser);

 public:
  std::vector<Vector2f> position_;

 private:
  struct RosHokuyoData;
  std::unique_ptr<RosHokuyoData> scan_data_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::HokuyoRosBridge);
