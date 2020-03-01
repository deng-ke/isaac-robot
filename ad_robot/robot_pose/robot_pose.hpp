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
#include "engine/gems/coms/i2c.hpp"
#include "gtest/gtest.h"

namespace isaac {
  namespace rosbridge {
    class RobotPose : public alice::Codelet {
      public:
        void start() override;
        void tick() override;
        void stop() override;
        void createStateMachine();
        void driveToPose();
        void stopDrive();

        /*IIC*/
        void recGoalPose();
        void sendRobotPose();
        
        /*pose*/
        bool getRobotPose();

        ISAAC_POSE2(world, robot);
        ISAAC_PROTO_TX(Goal2Proto, pose);
        ISAAC_PARAM(std::string, goto_has_arrived, "go_to/isaac.navigation.GoTo/has_arrived");
        ISAAC_PARAM(std::string, goto_is_stationnary, "go_to/isaac.navigation.GoTo/is_stationary");
        ISAAC_PARAM(std::string, start_state, "kWaitForPose");
        ISAAC_PARAM(int, i2c_address, 0x69);
        ISAAC_PARAM(int, device_id, 2);
        ISAAC_PARAM(int, publisher_queue_size, 1000);
        ISAAC_PARAM(std::string, robot_pose_topic, "robot_pose");

      private:
        isaac::I2c i2c_arduino;
        Vector5d robot_pose_;
        using State = std::string;
        bool want_to_stop_;
        state_machine::StateMachine<State> machine_;
        bool ticked_state_machine_ = false;
        struct RobotPoseData;
        std::unique_ptr<RobotPoseData> robot_pose_data_;
    };
  }
} //namespace}

ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::RobotPose);
