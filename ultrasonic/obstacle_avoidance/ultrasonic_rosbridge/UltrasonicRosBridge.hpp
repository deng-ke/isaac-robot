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

class UltrasonicRosBridge : public alice::Codelet
{
 public:
  UltrasonicRosBridge();
  virtual ~UltrasonicRosBridge();

  void start () override;
  void tick  () override;
  void stop  () override;

  ISAAC_PROTO_TX(FlatscanProto, sonar_scan);

  // ROS subscriber queue depth
  ISAAC_PARAM(int, subscriber_queue_size, 1000);
  // ROS subscriber channel. Used to receive messagse from ROS
  ISAAC_PARAM(std::string, sonar_topic1, "sonar1");
  ISAAC_PARAM(std::string, sonar_topic2, "sonar2");
  ISAAC_PARAM(std::string, sonar_topic3, "sonar3");
  ISAAC_PARAM(std::string, sonar_topic4, "sonar4");
  ISAAC_PARAM(std::string, sonar_topic5, "sonar5");
  ISAAC_PARAM(std::string, sonar_topic6, "sonar6");

  ISAAC_POSE2(robot, ultrasonic);

 public:
  std::vector<Vector2f> position_;

 private:
  struct RosUltrasonicData;
  std::unique_ptr<RosUltrasonicData> ultrasonic_data_;

};

}  // namespace rosbridge
}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::UltrasonicRosBridge);
