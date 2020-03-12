#include "navigation_behavior.hpp"
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include "engine/alice/components/deprecated/GroupSelectorBehavior.hpp"
#include "engine/alice/components/deprecated/SelectorBehavior.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "messages/state/differential_base.hpp"

using namespace std;

namespace isaac {
  namespace {
    static int tick_num = 0;
    constexpr char kSleepHelper[] = "wait_for_goal_stopwatch";
    constexpr char kDistanceUpdateHelper[] = "distance-update-helper";
    constexpr double kWaitBeforeCheckForArrival = 3.0;

    // state for navigation
    constexpr char kStateReLocalize[] = "kReLocalize";
    constexpr char kStateWaitForPose[] = "kWaitForPose";
    constexpr char kStateDriveToPose[] = "kDriveToPose";
    constexpr char kStateDriveToPile[] = "kDriveToPile";
    constexpr char kStateChargeMessage[] = "kChargeMessage";
  } //namespace

  #if 1
  void navigation_behavior::rotation(int direction) {
    auto msg = tx_relocalize_ctrl().initProto();
    auto _axes = msg.initAxes(2);
    _axes[0].setY(0.0); // linear_speed
    if(direction == 0) {
      _axes[1].setX(get_relocalize_angular_speed()); //angular_speed
    }
    else {
      _axes[1].setX(get_relocalize_angular_speed()); //angular_speed
    }
    tx_relocalize_ctrl().publish();
  }
  #endif

  void navigation_behavior::setStop() {
    auto msg = tx_relocalize_ctrl().initProto();
    auto _axes = msg.initAxes(2);
    _axes[0].setY(0.0); // linear_speed
    _axes[1].setX(0.0); //angular_speed
    tx_relocalize_ctrl().publish();
  }

  inline void navigation_behavior::switchToCtrl() {
    auto msg = tx_channel_switch().initProto();
    msg.setMessage("ctrl");
    tx_channel_switch().publish();
  }

  inline void navigation_behavior::switchToCmd() {
    auto msg = tx_channel_switch().initProto();
    msg.setMessage("cmd");
    tx_channel_switch().publish();
  }

  Vector5d navigation_behavior::getRobotPose() {
    bool ok;
    Vector5d robot_pose;
    const Pose2d world_T_robot = get_world_T_robot(getTickTime(), ok);
    if(ok) {
     robot_pose[0] = world_T_robot.translation(0);
     robot_pose[1] = world_T_robot.translation(1);
     robot_pose[2] = world_T_robot.rotation.sin();
     robot_pose[3] = world_T_robot.rotation.cos();
     robot_pose[4] = world_T_robot.rotation.angle();
    }
    return robot_pose;
  }

  void navigation_behavior::driveToPose() {
    navigation_mode_->set_desired_behavior("navigate");
  }

  void navigation_behavior::driveToWaypoint(const std::string& waypoint) {
    navigation_mode_->set_desired_behavior("navigate");
    auto proto = tx_target_waypoint().initProto();
    proto.setWaypoint(waypoint);
    tx_target_waypoint().publish(node()->clock()->timestamp());
  }

  void navigation_behavior::stopDrive() {
    navigation_mode_->set_desired_behavior("stop");
  }

  void navigation_behavior::sendPingMessage(const std::string& text) {
    auto proto = tx_arrived_charge_pile().initProto();
    proto.setMessage(text);
    tx_arrived_charge_pile().publish();
  }

  void navigation_behavior::start() {
    navigation_mode_ = node()->app()->findComponentByName<navigation::GroupSelectorBehavior>(
        "navigation_mode/isaac.navigation.GroupSelectorBehavior");
    goal_behavior_ = node()->app()->findComponentByName<navigation::SelectorBehavior>(
        "goal_behavior/isaac.navigation.SelectorBhavior");
    ASSERT(navigation_mode_, "Could not find navigation mode");
    want_to_stop_ = false;
    createStateMachine();
    machine_.start(get_start_state());
    tickPeriodically();
  }

  void navigation_behavior::tick() {
    if (rx_pose().available()) {
      if (tick_num < 2 ) {  
        pose_proto_old_ = rx_pose().getProto();
        pose_goal_old_ = FromProto(pose_proto_old_.getGoal());
        tick_num ++;
      }
      machine_.tick();
      bool ticked_state_machine_ = true;
      if (!ticked_state_machine_) {
        std::cout << "retick machine..." << std::endl;
        machine_.tick();
      }
    }
  }

  void navigation_behavior::stop() {
    want_to_stop_ = true;
    machine_.tick();
    machine_.stop();
  }

  void navigation_behavior::createStateMachine() {
    const std::vector<State> all_states = {
      kStateReLocalize,
      kStateWaitForPose,
      kStateDriveToPose,
      kStateDriveToPile,
      kStateChargeMessage,
    };
    machine_.setToString([this] (const State& state) {return state;});
    machine_.addState(kStateReLocalize,
        [this] {
          switchToCtrl();
          std::cout << "start re-localize" << std::endl;
          rotation(0);
        },
        [this] {
          switchToCtrl();
          rotation(0);
        },
        [this] {
          setStop();
          std::cout << "complete re-localize" << std::endl;
        }
    );
    machine_.addState(kStateWaitForPose,
        [this] {
          switchToCmd();
          stopDrive();
          std::cout << "I am wating for a pose..." << std::endl;
          robot_pose_ = getRobotPose();
          std::cout << "robot_pose: " << std::endl;
          std::cout << "x:" << robot_pose_[0] << ", ";
          std::cout << "y:" << robot_pose_[1] << ", ";
          std::cout << "sin(a):" << robot_pose_[2] << ", ";
          std::cout << "cos(a):" << robot_pose_[3] << ", ";
          std::cout << "theta:" << robot_pose_[4] << ", ";
        },
        [this] {
          switchToCmd();
        }, 
        [] {}
    );
    machine_.addState(kStateDriveToPose,
        [this] {
          std::cout << "Driving to posepoint" << std::endl;
          stopwatch(kSleepHelper).start();
          switchToCmd();
          driveToPose();
        },
        [this] {
          switchToCmd();
        }, 
        [this] {
          switchToCmd();
          stopDrive();
          stopwatch(kSleepHelper).stop();
          std::cout << "Arrived at posepoint~" << std::endl;
        }
    );
    machine_.addState(kStateDriveToPile,
        [this] {
          std::cout << "Driving to charge-pile" << std::endl;
          stopwatch(kSleepHelper).start();
          driveToWaypoint("charge_pile");
        },
        [this] {
          switchToCmd();
        },
        [this] {
          switchToCmd();
          stopDrive();
          stopwatch(kSleepHelper).stop();
          std::cout << "Arrived at posepoint~" << std::endl;
        }
    );
    machine_.addState(kStateChargeMessage,
        [this] {
          switchToCmd();
          stopDrive();
          int i = 20;
          do {
            sendPingMessage("charge");
            usleep(500000);
          } while(i--);
        },
        [] {
        },
        [] {
        }
    );
    machine_.addTransition(kStateReLocalize, kStateWaitForPose,
        [this] {
  #if 0 //use status and status_desired to re-localize
          if (stopwatch(kSleepHelper).read() < kWaitBeforeCheckForArrival) {
            return false;
          }
          const auto localize_status = getVariable(get_localize_status());
          const auto localize_status_desired = getVariable(get_localize_status_desired());
          std::cout << *localize_status << std::endl;
          std::cout << *localize_status_desired << std::endl;
          if (!localize_status) {
            LOG_WARNING("could not read variable 'localize_status'");
            return false;
          }
          if (!localize_status_desired) {
            LOG_WARNING("could not read variable 'localize_status_desired'");
            return false;
          }
          if(localize_status>1.5 && localize_status_desired>1.5)
            return true;
          else
            return false;
  #endif
  #if 1 //rotation to re-localize
          sleep(10);
          return true;
  #endif
        },
        [] {}
    );
    machine_.addTransition(kStateDriveToPose, kStateWaitForPose,
        [this] {
          if (stopwatch(kSleepHelper).read() < kWaitBeforeCheckForArrival) {
            return false;
          }
          // Check if the robot has arrived
          const auto has_arrived = getVariable(get_goto_has_arrived());
          std::cout << "has_arrived" << *has_arrived << std::endl;
          if (!has_arrived) {
            LOG_WARNING("could not read variable 'goto_has_arrived'");
            return false;
          }
          return (has_arrived > 0.5);
        },
        [] {}
    );
    machine_.addTransition(kStateWaitForPose, kStateDriveToPose,
        [this] {
          pose_proto_new_ = rx_pose().getProto();
          pose_goal_new_ = FromProto(pose_proto_new_.getGoal());
          //
          if (pose_goal_new_.translation.x() == 0 &&
              pose_goal_new_.translation.y() == 0) {
            return false;
          }
          else {
            if (pose_goal_old_.translation.x() == pose_goal_new_.translation.x() &&
                pose_goal_old_.translation.y() == pose_goal_new_.translation.y()) {
              return false;
            }
            else {
              /*test*/
              std::cout << "true x_new: " << pose_goal_new_.translation.x() <<std::endl;
              std::cout << "true y_new: " << pose_goal_new_.translation.y() <<std::endl;
              std::cout << "true sin: " << pose_goal_new_.rotation.sin()<<std::endl;
              std::cout << "true cos: " << pose_goal_new_.rotation.cos()<<std::endl;
              std::cout << "true theta: " << pose_goal_new_.rotation.angle()<<std::endl;
              /*end test*/
              pose_goal_old_ = pose_goal_new_;
              return true;
            }
          } 
        },
        [] {}
    );
#if 0
    machine_.addTransition(kStateDriveToPose, kStateDriveToPile,
        [] {
        },
        [] {}
    );
    machine_.addTransition(kStateWaitForPose, kStateDriveToPile,
        [] {
        },
        [] {}
    );
#endif
    machine_.addTransition(kStateDriveToPile, kStateWaitForPose,
        [this] {
          const auto has_arrived = getVariable(get_goto_has_arrived());
          std::cout << "has_arrived" << *has_arrived << std::endl;
          if (!has_arrived) {
            LOG_WARNING("could not read variable 'goto_has_arrived'");
            return false;
          }
          return (has_arrived > 0.5);
        },
        [] {}
    );
    machine_.addTransition(kStateChargeMessage, kStateWaitForPose,
        [this] {
          return false;
        },
        [] {}
    );
  }

} //namespace isaac
