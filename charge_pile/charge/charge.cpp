/*采用状态机*/
#include "charge.hpp"
#include <string>
#include <utility>
#include <vector>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
using namespace std;


/*machine-state and IR-data*/
namespace isaac {
  namespace {
    constexpr char kStateReqCharge[] = "kReqCharge";
    constexpr char kStateGetDistance[] = "kGetDistance";
    constexpr char kStateSearchPile[] = "kSearchPile";
    constexpr char kStateHandshake[] = "kHandShake";
    constexpr char kStateCharging[] = "kCharging";
    constexpr char kStateFinishCharge[] = "kFinishCharge";
    constexpr char kStateReSearch[] = "kStateReSearch";

    // used to request charge
    constexpr unsigned char kReqData[6] = {0xA1, 0xF1, 0x11, 0x11, 0x11};
    // used to search pile
    //constexpr unsigned char kSearchData[6] = {0xA1, 0xF1, 0x2A, 0x2B, 0x2C};
    constexpr unsigned char kSearchData[6] = {0xA1, 0xF1, 0x22, 0x22, 0x22};
    // SYN of handshake data
    constexpr unsigned char kSynData[6] = {0xA1, 0xF1, 0x33, 0x33, 0x33};
    // ACK of handshake data
    constexpr unsigned char kAckData[6] = {0xA1, 0xF1, 0x44, 0x44, 0x44};
    // FIN of handshake data(if no battery protocol)
    constexpr unsigned char kFinData[6] = {0xA1, 0xF1, 0x55, 0x55, 0x55};
    // indicates the end of charging
    constexpr unsigned char kEndData[6] = {0xA1, 0xF1, 0x66, 0x66, 0x66};

    unsigned char kBuffer[3] = {0x00};
    unsigned char kBatteryData[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
    unsigned char kBatteryBuffer[100] = {0x00};
  } //namespace


/*infrared and battery*/
  void charge::readBatteryState() {
    battery_.writeChars(kBatteryData, 7);
    usleep(100000);
    battery_.readChars(&kBatteryBuffer[0], 34);
  }
  inline void charge::resetBuffer() {
    for(int i=0; i<3; i++) {
      kBuffer[i] = 0x00;
    }
  }
  inline bool charge::isEqualData(const unsigned char* data) {
    if(kBuffer[0]==data[2] && kBuffer[1]==data[3] && kBuffer[2]==data[4])
      return 1;
    else
      return 0;
  }


/*motion control*/
  inline void charge::setStop() {
    auto msg = tx_charge_ctrl().initProto();
    auto _axes = msg.initAxes(2);
    _axes[0].setY(0); // linear_speed
    _axes[1].setX(0); //angular_speed
    tx_charge_ctrl().publish();
  }

  inline void charge::goBack(double distance) {
    auto msg = tx_charge_ctrl().initProto();
    auto _axes = msg.initAxes(2);
    _axes[0].setY(get_linear_speed());
				_axes[0].setX(0);
    tx_charge_ctrl().publish();
    sleep(distance/get_linear_speed());
    setStop();
  }

  inline void charge::goAhead(double distance) {
    auto msg = tx_charge_ctrl().initProto();
    auto _axes = msg.initAxes(2);
    _axes[0].setY(-get_linear_speed());
				_axes[0].setX(0);
    tx_charge_ctrl().publish();
    sleep(distance/get_linear_speed());
    setStop();
  }

  void charge::rotation(int direction, double degree) {
    auto msg = tx_charge_ctrl().initProto();
    auto _axes = msg.initAxes(2);
    _axes[0].setY(0); // linear_speed
    if(direction == 0) {
      _axes[1].setX(get_angular_speed()); //angular_speed
    }
    else {
      _axes[1].setX(-get_angular_speed()); //angular_speed
    }
    tx_charge_ctrl().publish();
    sleep((1.29*3.14*degree/180)/get_angular_speed());
    setStop();
  }

  void charge::rotation(int direction) {
    auto msg = tx_charge_ctrl().initProto();
    auto _axes = msg.initAxes(2);
    _axes[0].setY(0); // linear_speed
    if(direction == 0) {
      _axes[1].setX(get_angular_speed()); //angular_speed
    }
    else {
      _axes[1].setX(-get_angular_speed()); //angular_speed
    }
    tx_charge_ctrl().publish();
  }

#if 0
  double charge::getDistance() {
    sleep(3);
    if (rx_laser_scan().available()) {
      const auto scan_proto = rx_laser_scan().getProto();
      auto distance = scan_proto.getRanges();
      double sum = 0;
      for(int i=0; i<1081; i++) {
        sum += distance[i];
        std::cout << i << ": " << distance[i] << std::endl;
      }
      return sum/(1+get_angle_ranges());
    }
    return 0;
  }
#endif

#if 1
  double charge::getDistance() {
    sleep(3);
    if (rx_laser_scan().available()) {
      const auto scan_proto = rx_laser_scan().getProto();
      auto distance = scan_proto.getRanges();
      double sum = 0;
      for(int i=200; i<=800; i++) {
        //std::cout << i << std::endl;
          int j = 1;
          for(j=1; j<=get_angle_ranges(); j++) {
            if(distance[i+j] > 0.2) {
              sum += (distance[i+j] - distance[i]);
            }
            else {
              break;
            }
          }
          std::cout << j << std::endl;
          if(j==get_angle_ranges()+1 && distance[i]<0.8 && sum<(get_angle_ranges()+1)*get_distance_threshold()) {
            std::cout << i << std::endl;
            return distance[i]+sum/(get_angle_ranges()+1);
          }
          else {
            sum = 0;
          }
      }
      return 1;
    }
    return 0;
  }
#endif

  inline void navigation_behavior::switchToCtrl() {
    auto msg = tx_channel_switch().initProto();
    msg.setX(2);
    tx_channel_switch().publish();
  }

  inline void navigation_behavior::switchToCmd() {
    auto msg = tx_channel_switch().initProto();
    msg.setX(1);
    tx_channel_switch().publish();
  }


/*main*/
  void charge::start() {
    createStateMachine();
    machine_.start(get_start_state());
    tickPeriodically();
  }

  void charge::tick() {
#if 0 // test infrared
    for(int i=0; i<30; i++) {
    infrared_.writeChars(kReqData, 5);
    }
    sleep(1);
    for(int k=0; k<30; k++) {
      infrared_.readChars(&kBuffer[0], 3);
      for(int j=0; j<3; j++) {
        std::cout << hex << (unsigned int)(unsigned char)kBuffer[j] << std::endl;
      }
    }
    if(isEqualData(kSynData)) {
      for(int n=0; n<10; n++) {
        infrared_.writeChars(kAckData, 5);
        usleep(100000);
      }
    }
#endif
    //sleep(1);
    //machine_.tick();
    //readBatteryState();
    //for (int i=0; i<34; i++) {
    //  std::cout << hex << (unsigned int)(unsigned char)kBatteryBuffer[i] << std::endl;
    //}
    //sleep(1);
  }

  void charge::stop() {
    machine_.tick();
    machine_.stop();
  }

  Vector3d charge::getPilePose() {
    bool ok;
    Vector3d pile_pose;
    const Pose2d world_T_robot = get_world_T_robot(getTickTime(), ok);
    if(ok) {
      pile_pose[0] = world_T_robot.translation(0);
      pile_pose[1] = world_T_robot.translation(1);
      pile_pose[2] = world_T_robot.rotation.angle();
    }
    return pile_pose;
  }

  void charge::createStateMachine() {
    const std::vector<State> all_states = {
      kStateReqCharge,
      kStateGetDistance,
      kStateSearchPile,
      kStateHandshake,
      kStateCharging,
      kStateFinishCharge,
      kStateReSearch,
    };
    machine_.setToString([this] (const State& state) {return state;});
    machine_.addState(kStateReqCharge,
        [this] {
          std::cout << "Request charging" << std::endl;
          rotation(1);
        },
        [this] {
          infrared_.writeChars(kReqData, 5);
        },
        [] {
        }
    );
    machine_.addState(kStateGetDistance,
        [this] {
          std::cout << "start to get distance" << std::endl;
          rotation(0, 180);
          sleep(3);
          while(distance_<0.1) {
            distance_ = getDistance();
            //std::cout << "dengke: " << distance_ << std::endl;
          }
          rotation(1);
          // clear serial buffer
          tcflush(infrared_.fd_, TCIFLUSH);
          std::cout << "distance: " << distance_ << std::endl;
        },
        [] {},
        [] {
        }
    );
    machine_.addState(kStateSearchPile,
        [this] {
          tcflush(infrared_.fd_, TCIFLUSH);
          goBack(distance_ - get_robot_radius());
          std::cout << "Search Pile" << std::endl;
        },
        [] {},
        [this] {
          setStop();
          tcflush(infrared_.fd_, TCIFLUSH);
          std::cout << "arrived at pile" << std::endl;
        }
    );
    machine_.addState(kStateHandshake,
        [this] {
          tcflush(infrared_.fd_, TCIFLUSH);
          std::cout << "Handshake" << std::endl;
        },
        [this] {
          handshake_times_++;
        },
        [this] {
          handshake_times_ = 0;
        }
    );
    machine_.addState(kStateCharging,
        [this] {
          tcflush(infrared_.fd_, TCIFLUSH);
          std::cout << "Charging" << std::endl;
          pile_pose_ = getPilePose();
          std::cout << "pile_pose: " << std::endl;
          std::cout << "x:" << pile_pose_[0] << ", ";
          std::cout << "y:" << pile_pose_[1] << ", ";
          std::cout << "theta:" << pile_pose_[2] << std::endl;
        },
        [] {}, [] {}
    );
    machine_.addState(kStateFinishCharge,
        [this] {
          tcflush(infrared_.fd_, TCIFLUSH);
          std::cout << "Finished Charge" << std::endl;
          for(int i=0; i<800; i++){
            infrared_.writeChars(kEndData, 5);
            std::cout << "send EndData" << std::endl;
            sleep(1);
          }
        },
        [] {}, [] {}
    );
    machine_.addState(kStateReSearch,
        [this] {
          tcflush(infrared_.fd_, TCIFLUSH);
          goAhead(distance_ - get_robot_radius());
          std::cout << "Re-search Pile" << std::endl;
        },
        [] {}, [] {}
    );
    machine_.addTransition(kStateReqCharge, kStateGetDistance,
        [this] {
          //tcflush(infrared_.fd_, TCIFLUSH);
          infrared_.readChars(&kBuffer[0], 3);
          if (isEqualData(kSearchData)) {
            sleep(1);
            setStop();
            sleep(2);
            resetBuffer();
            return true;
          }
          else
            return false;
        },
        [] {}
    );
    machine_.addTransition(kStateGetDistance, kStateSearchPile,
        [this] {
          infrared_.readChars(&kBuffer[0], 3);
          std::cout << hex << (unsigned int)(unsigned char)kBuffer[0] << std::endl;
          if (isEqualData(kSearchData)) {
            //sleep(2);
            usleep(3000000);
            setStop();
            /*for (int i=0; i<3; i++) {
              kBuffer[i] = 0x00;
            }*/
            resetBuffer();
            sleep(2);
            return true;
          }
          else
            return false;
        },
        [] {}
    );
    machine_.addTransition(kStateSearchPile, kStateHandshake,
        [this] {
          return true;
        },
        [] {}
    );
    machine_.addTransition(kStateHandshake, kStateCharging,
        [this] {
          infrared_.readChars(&kBuffer[0], 3);
          /*for(int i=0; i<3; i++) {
          //std::cout << kBuffer[i] << std::endl;
          std::cout << hex << (unsigned int)(unsigned char)kBuffer[i] << std::endl;
          }*/
          if (isEqualData(kSynData)) {
            int i = 30;
            std::cout << "IR3" << std::endl;
            do {
               infrared_.writeChars(kAckData, 5);
               //infrared_.writeChars(kReqData, 5);
               usleep(10000);
               std::cout << "send IR4" << std::endl;
            } while(i--);
            //sleep(1);

            #if 0 /*if use battery protocol*/
            if (readBatteryState()) {
              return true;
            }
            #endif

            #if 1 /*if no battery protocol*/
            int j = 30;
            do {
              infrared_.readChars(&kBuffer[0], 3);
              if (isEqualData(kFinData)) {
                return true;
              }
            } while(j--);
            #endif
            return false;
          }
          else {
            isOKHandshake_ = 0;
            return false;  
          }
        },
        [] {}
    );
    machine_.addTransition(kStateCharging, kStateFinishCharge,
        [this] {
          //if(readBatteryState()) {
          if(1) {
            return true;
          }
          else
            return false;
        },
        [] {}
    );
#if 0
    machine_.addTransition(kStateHandshake, kStateReSearch,
        [this] {
          if(handshake_times_ == get_handshake_time()) {
            return true;
          }
          else
            return false;
        },
        [] {}
    );
#endif
    machine_.addTransition(kStateReSearch, kStateReqCharge,
        [this] {
          return true;
        },
        [] {}
    );
  }
} //namespace isaac
