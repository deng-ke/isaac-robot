#pragma once

#include <memory>
#include <string>
#include <cstddef>
#include <mutex>
#include <thread>
#include <functional>   // std::minus
#include <numeric>      // std::accumulate

#include <errno.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>  // 定义错误帧的符号位,用于设置掩码和容错

#include "engine/alice/alice_codelet.hpp"
#include "messages/messages.hpp"
#include "engine/gems/state_machine/state_machine.hpp"
#include "engine/core/math/pose2.hpp"
#include "messages/math.hpp"

namespace isaac {

template<class T> class CircularBuffer {
 public:
  typedef T& reference;
  typedef const T& const_reference;
  typedef T* pointer;
  typedef const T* const_pointer;

 public:
  CircularBuffer() {}
  CircularBuffer(const std::size_t &capacity) { buf_.resize(capacity); }
  CircularBuffer(const CircularBuffer<T> &other) {
    std::swap(buf_, other.buffer());
    head_ = other.head();
    tail_ = other.tail();
    size(other.size());
  }
  CircularBuffer<T>& operator=(const CircularBuffer<T> &other) {
    std::swap(buf_, other.buffer());
    head_ = other.head();
    tail_ = other.tail();
    size(other.size());
    return *this;
  }

  virtual ~CircularBuffer() { buf_.clear(); }

  bool isEmpty() const { return (0 == size()); }
  bool isFull() const { return (size() == capacity()); }

  std::vector<reference> buffer() { return buf_; }

  std::size_t head() const { return head_; }
  std::size_t tail() const { return tail_; }

  std::size_t size() const { return size_; }
  void size(const std::size_t &s) { size_ = s; }

  std::size_t capacity() const { return buf_.size(); }

  void resize(const std::size_t &s) {
    std::vector<T> _new(s);
    std::swap(buf_, _new);
    head_ = 0; tail_ = 0; size(0);
  }

  void push_back(const T& value) {
    if (isFull()) {
      head_ = (1 + head_) % capacity();
    } else {
      ++size_;
    }

    buf_[tail_] = value;
    tail_ = (1 + tail_) % capacity();
  }
  reference get(const std::size_t &index) {
    return buf_[(head() + index) % capacity()];
  }
  reference back() { return get(size() - 1); }
  reference operator[] (const std::size_t &index) { return get(index); }

 private:
  std::vector<T> buf_;
  std::size_t head_ = 0;
  std::size_t tail_ = 0;
  std::size_t size_ = 0;
};

namespace drivers {

typedef struct can_frame Frame_t;

class MotorDriver {
 public:
  struct RpmState {
    int flag_ = 0;  // 0: both; 1: right; 2: left
    float left_rpm_  = 0;
    float right_rpm_ = 0;
    std::int64_t timestamp_ = isaac::NowCount(); // timestamp in nanoseconds
  };

  typedef enum {
    kFnRequest   = 0xDA,
    kFnRequestOnce = 0xDC,
    kFnResponse  = 0xDB,
    kFnFeedback  = 0xFE,
    kFnException = 0xFF
  } FnCode_t;

  typedef enum {
    kAddrHead = 0,
    kAddrProcess = 0x10,
    kAddrObjVelocity = 0x11,
    kAddrAccTime4Pos = 0x12,
    kAddrAccTime4Vel = 0x13,
    kAddrTrapezoidVel = 0x14,
    kAddrWarningClear = 0x15,
    kAddrObjPosition = 0x16,
    kAddrPosition = 0x17,
    kAddrOriginPoint = 0x18,
    kAddrMode = 0x19,
    kAddrSetPosKP = 0x20,
    kAddrSetPosKD = 0x21,
    kAddrSetPosKI = 0x22,
    kAddrSetVelKP = 0x23,
    kAddrSetVelKD = 0x24,
    kAddrSetVelKI = 0x25,
    kAddrSetCurrentKP = 0x26,
    kAddrSetCurrentKI = 0x27,
    kAddrBraking = 0x30,
    kAddrVoltage = 0xE1,
    kAddrOutputCurrent = 0xE2,
    kAddrWarningState = 0xE3,
    kAddrRealTimeRMP = 0xE4,
    kAddrPositionSet = 0xE6,
    kAddrPositionFeedback = 0xE8,
    kAddrTail
  } AddrCode_t;

  typedef enum {
    kCtrlCodeHead     = 0,
    kCtrlCodeRelease  = 0x0F,
    kCtrlCodeEnable   = 0x1F,
    kCtrlCodeVelocityMode  = 0x2F,
    kCtrlCodePositionMode  = 0x3F,
    kCtrlCodeAbsPosProcess = 0x4F,
    kCtrlCodeRelPosProcess = 0x5F,
    kCtrlCodeOriginPoint   = 0x6F,
    kCtrlCodeWarningClear  = 0x7F,
    kCtrlCodeSetRMP = 0x55,
    kCtrlCodeTail
  } ControlCode_t;

  typedef enum {
    kValHead = 0,
    kValMode,
    kValAccPeriod,
    kValBraking,
    kValEnable,
    kValKp,
    kValKi,
    kValKd,
    kValTail
  } ValueType_t;

  typedef std::function<void(RpmState &)> RpmCallback_t;
  typedef std::function<void(int)> ExceptionCallBack_t;

 public:
  MotorDriver(const std::string &dev) : device_(dev) {}
  virtual ~MotorDriver() { ::close(fd_); }

  void open();
  void close() { ::close(fd_); }

  void setRmpFunc(RpmCallback_t rpm) { rpm_ = rpm; }
  void setExceptionFunc(ExceptionCallBack_t except) { except_ = except; }

  bool decode(const Frame_t &f); // 协议解析代码

  void enable();
  void disable();
  void braking();
  void clearWarning();
  void setAccel();
  void velocityMode();

  void process_error(const int &error);

  int readData(Frame_t &frame);
  int write(const Frame_t *fp, const std::size_t &size);

  int writeData(const Frame_t &f, const std::size_t &s) {
    int _count = 0; int _nbytes = 0; int _errno = 0;
    const int _len = static_cast<int>(s);
    do { _nbytes = ::write(fd_, &f + _count, _len - _count);
      _errno = errno;
      process_error(_errno);
      if (-1 == _nbytes) {
        break;
      } else {
        _count += _nbytes;
      }
    } while (_count < _len);

    // if (_nbytes == static_cast<int>(s)) {
    //   std::cout << "++++";
    // } else {
    //   std::cout << "write size: " << _nbytes << std::endl;
    // }
    isaac::Sleep(0);
    return _count;
  }
  void sendVelocity(const double &, const double &);
  void option() {
    velocityMode();
    clearWarning();
    //setAccel();
    enable();
  }

 private:
  int fd_ = -1;
  std::uint32_t left_id_ = 1;
  std::uint32_t right_id_ = 2;
  std::string device_ = "can0";

  RpmCallback_t rpm_ = std::nullptr_t();
  ExceptionCallBack_t except_ = std::nullptr_t();
};  // class MotorDriver
}  // namespace drivers
}  // namespace isaac

namespace isaac {
class ZhonglingRmpDriver : public isaac::alice::Codelet {
 public:
  struct Velocity_t {
    Velocity_t() {}
    Velocity_t(const double &lin, const double &ang, const std::int64_t &tm)
        : linear_(lin), angular_(ang), timestamp_(tm) {}
    Velocity_t(const Velocity_t &v) {
      linear_ = v.linear_; angular_ = v.angular_;
      lin_acc_ = v.lin_acc_;
      ang_acc_ = v.ang_acc_;
      timestamp_ = v.timestamp_;
    }
    Velocity_t& operator=(const Velocity_t &v) {
      linear_ = v.linear_; angular_ = v.angular_;
      lin_acc_ = v.lin_acc_;
      ang_acc_ = v.ang_acc_;
      timestamp_ = v.timestamp_;
      return *this;
    }

    double linear_  = 0.0;
    double angular_ = 0.0;
    double lin_acc_ = 0.0;
    double ang_acc_ = 0.0;
    std::int64_t timestamp_ = isaac::NowCount();
  };

 public:
  void start () override;
  void tick  () override;
  void stop  () override;

  bool update ();
  void smoothVelocity(double safe_linear, double safe_angular, double left, 
      double right, double wheel_separation, double last_linear_speed);
  void createStateMachine();
  Vector2i getChannelSwitch();
  void setState(const drivers::MotorDriver::RpmState &);
  drivers::MotorDriver::RpmState getState() const { return state_; }

  ISAAC_PROTO_RX (StateProto, zl_cmd);
  ISAAC_PROTO_RX (JoystickStateProto, zl_ctrl);
  ISAAC_PROTO_RX (Vector2iProto, channel_switch);
  ISAAC_PROTO_TX (DifferentialBaseStateProto, zl_state);

  ISAAC_PARAM(std::string, Device, "can0");
  // ISAAC_PARAM(int, baudrate, 460800);
  ISAAC_PARAM(int, LeftCanID, 1);
  ISAAC_PARAM(int, RightCanID, 2);
  ISAAC_PARAM(int, VelocityRollingWindowSize, 10);
  ISAAC_PARAM(double, SpeedLimitLinear, 0.7);
  ISAAC_PARAM(double, SpeedLimitAngular, 1.0);
  ISAAC_PARAM(double, MaxAccelerationLinear, 1.0);
  ISAAC_PARAM(double, MaxAccelerationangular, 1.0);

  ISAAC_PARAM(double, WheelSeparation, 0.37666);
  ISAAC_PARAM(double, WheelRadius, 0.085);
  ISAAC_PARAM(int, EncoderResolution, 4132);
  ISAAC_PARAM(double, LeftWheelMultiplier, -1.0);
  ISAAC_PARAM(double, RightWheelMultiplier, 1.0);
  ISAAC_PARAM(double, WheelSeparationMultiplier, 1.0);
  ISAAC_PARAM(double, Kp, 1.0);
  ISAAC_PARAM(double, Ki, 1.0);
  ISAAC_PARAM(double, Kd, 1.0);
  ISAAC_PARAM(double, Ko, 1.0);
  ISAAC_PARAM(std::string, start_state, "kCtrl");

 public:
  std::mutex lock_;

 private:
  using State = std::string;
  state_machine::StateMachine<State> machine_;
  std::unique_ptr<drivers::MotorDriver> motor_;
  alice::Failsafe* failsafe_;
  drivers::MotorDriver::RpmState state_;
  CircularBuffer<Velocity_t> states_;

  std::unique_ptr<std::thread> thread_;
  bool run_ = false;
  double last_linear_speed_;
  double last_linear_speed_cmd_;
  Vector2i channel_switch_;
};

} // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ZhonglingRmpDriver);
