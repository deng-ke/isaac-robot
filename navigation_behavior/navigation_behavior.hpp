/*
  Dengke dengke1024@126.com
*/

#pragma once
#include <queue>
#include <string>
#include <iostream>
#include "engine/alice/alice_codelet.hpp"
#include "engine/core/optional.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/gems/state_machine/state_machine.hpp"
#include "messages/messages.hpp"
#include "messages/math.hpp"
#include "engine/core/assert.hpp"
#include <unistd.h>
#include "engine/gems/state/io.hpp"

namespace isaac {
  namespace navigation {
    class GroupSelectorBehavior;
    class SelectorBehavior;
  }
  class navigation_behavior : public alice::Codelet {
    public:
      void start() override;
      void tick() override;
      void stop() override;
      void createStateMachine();

      /*motion control*/
      void rotation(int direction);
      void setStop();
      // switch velocity channel to JoystickStateProto(ctrl)
      inline void switchToCtrl();
      // switch velocity channel to StateProto(cmd)
      inline void switchToCmd();

      /*navigation*/
      Vector5d getRobotPose();
      void driveToPose();
      void driveToWaypoint(const std::string& waypoint);
      void stopDrive();

      /*message*/
      bool isRequestCharge();
      //send a message to charge package
      void sendPingMessage(const std::string& text);

#if 1 /*used to read robot pose from posetree*/
      ISAAC_POSE2(world, robot);
#endif

      // pose as goal
      ISAAC_PROTO_RX(Goal2Proto, pose);
      // While raybot arrived charge-pile, send this message to charge-package
      ISAAC_PROTO_TX(PingProto, arrived_charge_pile);
      // receive form charge-package, to determine whether raybot is fully charged.
      ISAAC_PROTO_RX(PingProto, finish_charge);
      // send velocity to zhongling using cmd channel
      ISAAC_PROTO_TX(StateProto, relocalize_cmd);
      // send velocity to zhongling using ctrl channel
      ISAAC_PROTO_TX(JoystickStateProto, relocalize_ctrl);
      // A switch for switching velocity channel between StateProto(cmd) and JoystickStateProto(ctrl)
      ISAAC_PROTO_TX(PingProto, channel_switch);
      // The desired target waypoint where the robot wants to drive next
      ISAAC_PROTO_TX(GoalWaypointProto, target_waypoint);

      // Variable that indicates whether rayrobot arrived at the target
      ISAAC_PARAM(std::string, goto_has_arrived, "go_to/isaac.navigation.GoTo/has_arrived");
      // Variable that indicates whether rayrobot is stationary
      ISAAC_PARAM(std::string, goto_is_stationnary, "go_to/isaac.navigation.GoTo/is_stationary");
      // Variable that indicates whether rayrobot is stationary
      ISAAC_PARAM(std::string, goto_remaining_distance,"go_to/isaac.navigation.GoTo/remaining_delta_pose");
      // Variable that indicates whether rayrobot is stationary
      ISAAC_PARAM(std::string, localize_status, "localize/isaac.navigation.LocalizeBehavior/status");
      // Variable that indicates whether rayrobot is stationary
      ISAAC_PARAM(std::string, localize_status_desired, "localize/isaac.navigation.LocalizeBehavior/status_desired");
      // Variable that indicates whether rayrobot is stationary
      ISAAC_PARAM(std::string, start_state, "kChargeMessage");
      // Variable that indicates whether rayrobot is stationary
      ISAAC_PARAM(double, relocalize_angular_speed, 0.5);
    private:
      using State = std::string;
      bool want_to_stop_;

      state_machine::StateMachine<State> machine_;
      navigation::GroupSelectorBehavior* navigation_mode_;
      navigation::SelectorBehavior* goal_behavior_;
      bool ticked_state_machine_ = false;

      Goal2Proto::Reader pose_proto_old_;
      Goal2Proto::Reader pose_proto_new_;
      Pose2d pose_goal_old_;
      Pose2d pose_goal_new_;
      Vector5d robot_pose_;
      Vector5d pile_pose_;
  };
} //namespace}

ISAAC_ALICE_REGISTER_CODELET(isaac::navigation_behavior);
