#pragma once
#include <queue>
#include <string>
#include <iostream>
#include <ctime>
#include "engine/alice/alice_codelet.hpp"
#include "engine/core/optional.hpp"
#include "messages/messages.hpp"
#include "engine/gems/coms/serial.hpp"
#include "messages/math.hpp"
#include "engine/gems/state_machine/state_machine.hpp"
//#include "packages/navigation/RobotRemoteControl.hpp"


namespace isaac {
  class charge : public alice::Codelet {
    public:
      charge() {}
      virtual ~charge() {}
      void start() override;
      void tick() override;
      void stop() override;
      Vector3d getPilePose();
      void createStateMachine();
      void createMachine();

      /*infrared and battery*/
      inline void resetBuffer();
      inline bool isEqualData(const unsigned char* data);
      void readBatteryState();

      /*motion control*/
      inline void setStop();
      inline void goBack(double distance);
      inline void goAhead(double distance);
      void rotation(int direction, double degree);
      void rotation(int direction);
      double getDistance();
      // switch velocity channel to JoystickStateProto(ctrl)
      inline void switchToCtrl();
      // switch velocity channel to StateProto(cmd)
      inline void switchToCmd();

      ISAAC_POSE2(world, robot);

      ISAAC_PROTO_RX(FlatscanProto, laser_scan);
      ISAAC_PROTO_TX(JoystickStateProto, charge_ctrl);
      // receive form navigation_behavior
      ISAAC_PROTO_RX(PingProto, arrived_charge_pile);
      // while raybot is fully charged, send this message to navigation_behavior
      ISAAC_PROTO_TX(PingProto, finish_charge);
      // A switch for switching velocity channel between StateProto(cmd) and JoystickStateProto(ctrl)
      ISAAC_PROTO_TX(PingProto, channel_switch);

      ISAAC_PARAM(std::string, infrared_port, "/dev/ttyinfrared");
      ISAAC_PARAM(int, infrared_baud, 9600);
      ISAAC_PARAM(double, linear_speed, 0.3);
      ISAAC_PARAM(double, angular_speed, 0.5);
      ISAAC_PARAM(double, robot_radius, 0.12);
      //start state of state_machine
      ISAAC_PARAM(std::string, start_state, "kChargeMessage");
      //laser beam_lines in a certain ranges of angle(used to get distance)
      ISAAC_PARAM(int, angle_ranges, 80);
      //distance_threshold
      ISAAC_PARAM(double, distance_threshold, 0.3);
      //over handshake_time, handshake failed
      ISAAC_PARAM(int, handshake_time, 30000);

    private:
      using State = std::string;
      state_machine::StateMachine<State> machine_;
      isaac::Serial infrared_ = isaac::Serial("/dev/ttyTHS2", 9600);
      isaac::Serial battery_ = isaac::Serial("/dev/ttyUSB0", 9600);
      double distance_ = 0;
      Vector3d pile_pose_;
      bool isOKHandshake_ = 1;
      int handshake_times_ = 0;
  }; //class charge
} //namespace

ISAAC_ALICE_REGISTER_CODELET(isaac::charge);
