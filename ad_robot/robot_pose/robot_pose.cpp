#include "robot_pose.hpp"
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include "engine/alice/components/deprecated/GroupSelectorBehavior.hpp"
#include "engine/alice/components/deprecated/SelectorBehavior.hpp"
#include "engine/gems/algorithm/string_utils.hpp"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

using namespace std;

namespace isaac {
  namespace rosbridge {
    namespace {
      constexpr char kStateWaitForPose[] = "kWaitForPose";
      constexpr char kStateDriveToPose[] = "kDriveToPose";
      std::array<isaac::byte, 8> send_robot_pose = {'\0'};
      std::array<isaac::byte, 8> buffer = {'\0'};
      class CallbackFunctor {
        public:
          explicit CallbackFunctor(RobotPose* bridge) {
            bridge_ = bridge;
          }
          CallbackFunctor(const CallbackFunctor&) = default;
          ~CallbackFunctor() = default;
        private:
          RobotPose* bridge_;
      };
    } // namespace

    struct RobotPose::RobotPoseData {
      ros::NodeHandle robot_pose_node;
      ros::Subscriber robot_pose_sub;
      ros::Publisher robot_pose_pub;
      ros::CallbackQueue robot_pose_callbackQueue;
    };
  
    bool RobotPose::getRobotPose() {
       bool ok;
       const Pose2d world_T_robot = get_world_T_robot(getTickTime(), ok);
       if(ok) {
         //get robot pose form pose_tree
         robot_pose_[0] = world_T_robot.translation(0); //x
         robot_pose_[1] = world_T_robot.translation(1); //y
         robot_pose_[2] = world_T_robot.rotation.angle(); //theta
         robot_pose_[3] = world_T_robot.rotation.sin();
         robot_pose_[4] = world_T_robot.rotation.cos();
         return true;
       }
       else
         return false;
    }
  
    void RobotPose::recGoalPose() {
      
    }
  
    #if 1 //send robot pose by ROS
    void RobotPose::sendRobotPose() {
      bool ok = getRobotPose();
      if(ok) {
        geometry_msgs::Pose2D robot_pose_msg;
        robot_pose_msg.x = robot_pose_[0];
        robot_pose_msg.y = robot_pose_[1];
        robot_pose_msg.theta = robot_pose_[2];
        robot_pose_data_->robot_pose_pub.publish(robot_pose_msg);
      }
      else {
        LOG_WARNING("could not read the robot pose");
      }
    }
    #endif
  
    #if 0 //send robot pose by i2c
    void RobotPose::sendRobotPose() {
      bool ok = getRobotPose();
      if(ok) {
        //transform robot_pose_ from float to byte
        send_robot_pose[0] = floor(robot_pose_[0]/100);
        send_robot_pose[1] = floor(robot_pose_[0]) - 100*send_robot_pose[0];
        send_robot_pose[2] = floor(100*(robot_pose_[0] - (float)((int)robot_pose_[0])));
        send_robot_pose[3] = floor(robot_pose_[1]/100);
        send_robot_pose[4] = floor(robot_pose_[1]) - 100*send_robot_pose[3];
        send_robot_pose[5] = floor(100*(robot_pose_[1] - (float)((int)robot_pose_[1])));
        send_robot_pose[6] = floor(robot_pose_[2]);
        send_robot_pose[7] = floor(100*(robot_pose_[2] - send_robot_pose[6]));
      }
      else {
        LOG_WARNING("could not read the robot pose");
      }
      //send robot pose by i2c
      bool isOpen = i2c_arduino.open(1,get_i2c_address());
      EXPECT_TRUE(isOpen);
      i2c_arduino.write(0x00, 8, send_robot_pose.data());
      i2c_arduino.close();
    }
    #endif
  
    void RobotPose::start() {
      ros::M_string args;
      if (!ros::isInitialized()) {
        ros::init(args, "isaacRosBridge", ros::init_options::NoSigintHandler);
      }
      robot_pose_data_ = std::make_unique<RobotPoseData>();
      robot_pose_data_->robot_pose_node.setCallbackQueue(&(robot_pose_data_->robot_pose_callbackQueue));
      robot_pose_data_->robot_pose_pub = robot_pose_data_->robot_pose_node.advertise<geometry_msgs::Pose2D>(
          get_robot_pose_topic(), get_publisher_queue_size());
      tickPeriodically();
    }
  
    void RobotPose::tick() {
      if(ros::ok()) {
        sendRobotPose();
      }
      else {
        LOG_ERROR("An error has occurred within ROS!");
      }
    }
  
    void RobotPose::stop() {
      robot_pose_data_->robot_pose_pub.shutdown();
      robot_pose_data_->robot_pose_sub.shutdown();
      robot_pose_data_ = nullptr;
    }
  } // namespace rosbridge
} //namespace isaac
