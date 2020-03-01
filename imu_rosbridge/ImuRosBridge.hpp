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

class ImuRosBridge : public alice::Codelet
{
 public:
  ImuRosBridge();
  virtual ~ImuRosBridge();

  void start () override;
  void tick  () override;
  void stop  () override;

  ISAAC_PROTO_TX(ImuProto, imu);
  // ROS subscriber queue depth
  ISAAC_PARAM(int, subscriber_queue_size, 1000);
  // ROS subscriber channel. Used to receive messagse from ROS
  ISAAC_PARAM(std::string, imu_topic, "camera/imu");

  ISAAC_POSE2(robot, imu);

 public:
  std::vector<Vector2f> position_;

 private:
  struct RosImuData;
  std::unique_ptr<RosImuData> imu_data_;
};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::ImuRosBridge);
