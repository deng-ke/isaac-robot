#pragma once

#include <memory>
#include <string>

#include "CYdLidar.h"

#include "engine/alice/alice_codelet.hpp"
#include "messages/messages.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/constants.hpp"
#include "engine/core/logger.hpp"

using namespace ydlidar;

namespace isaac {
class IsaacYdlidar : public isaac::alice::Codelet {
 public:
  IsaacYdlidar();
  virtual ~IsaacYdlidar();

  void start() override;
  void tick() override;
  void stop() override;

  ISAAC_PROTO_TX(FlatscanProto, scan);

  ISAAC_PARAM(std::string, port, "/dev/ydlidar");
  ISAAC_PARAM(std::string, frame_id, "laser");
  ISAAC_PARAM(std::string, ignore_list, "");

  ISAAC_PARAM(int, baudrate, 115200);
  ISAAC_PARAM(int, samp_rate, 9);
  ISAAC_PARAM(double, frequency, 7);

  ISAAC_PARAM(double, angle_max, 180);
  ISAAC_PARAM(double, angle_min, -180);

  ISAAC_PARAM(double, range_max, 20.0);
  ISAAC_PARAM(double, range_min, 0.08);

  ISAAC_PARAM(bool, resolution_fixed, true);
  ISAAC_PARAM(bool, intensity, false);
  ISAAC_PARAM(bool, low_exposure, false);
  ISAAC_PARAM(bool, auto_reconnect, true);
  ISAAC_PARAM(bool, reversion, false);

 private:
  // void initLaser();
  // int decodePacket();
  // void processDataBlock(
  //     const LaserScan &rays_raw,
  //     capnp::List<RangeScanProto::Ray>::Builder& rays);
  std::unique_ptr<CYdLidar> ydlidar_;

}; // class IsaacYdlidar

} // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::IsaacYdlidar);