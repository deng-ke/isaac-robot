#include <cmath>

#include "engine/alice/components/Failsafe.hpp"
#include "engine/core/constants.hpp"
#include "engine/gems/sight/sight.hpp"
#include "engine/gems/state/io.hpp"
#include "messages/state/differential_base.hpp"

#include "ZhonglingRmpDriver.hpp"

namespace isaac {
namespace {
constexpr float kTicksPerRad = 0x1024;
// 8192 * 60 / 3000 / 2 / M_PI
constexpr float kVelocityCoef = 26.08949044585987;
constexpr float kReadCoef = 2.0*M_PI*3000 / 60 / 8192;

constexpr inline std::uint32_t endian(const std::uint32_t &value) {
  return ((value & 0x000000FF) << 24) | ((value & 0x0000FF00) << 8) |
      ((value & 0x00FF0000) >> 8) | ((value & 0xFF000000) >> 24);
}
constexpr inline std::int32_t endian(const std::int32_t &value) {
  return ((value & 0x000000FF) << 24) | ((value & 0x0000FF00) << 8) |
      ((value & 0x00FF0000) >> 8) | ((value & 0xFF000000) >> 24);
}
constexpr inline std::uint16_t endian(const std::uint16_t &value) {
  return (((value & 0xff00) >> 8) | ((value & 0x00ff) << 8));
}
constexpr inline std::int16_t endian(const std::int16_t &value) {
  return (((value & 0xff00) >> 8) | ((value & 0x00ff) << 8));
}
constexpr char kStateCmd[] = "kCmd";
constexpr char kStateCtrl[] = "kCtrl";
} // namespace

namespace drivers {
bool MotorDriver::decode(const Frame_t &f) {
  auto _addr = static_cast<int>(f.data[3]);
  auto _cat  = static_cast<int>(f.data[1]);
  // if (kFnResponse == _cat) {
  //   printf("can_id %d :", f.can_id);
  //   for (auto i=0; i<8; ++i) {
  //     printf("0x%x ", f.data[i]);
  //   } printf("\n");
  // }
  RpmState _state; _state.timestamp_ = isaac::NowCount();
  bool _isleft = (f.can_id == left_id_);
  switch (_addr) {
    case kAddrProcess:
    case kAddrAccTime4Pos:
    case kAddrAccTime4Vel:
    case kAddrTrapezoidVel:
    case kAddrWarningClear:
    case kAddrObjPosition:
    case kAddrPosition:
    case kAddrOriginPoint:
    case kAddrMode:
    case kAddrSetPosKI:
    case kAddrSetVelKP:
    case kAddrSetVelKD:
    case kAddrSetVelKI:
    case kAddrSetCurrentKP:
    case kAddrSetCurrentKI:
    case kAddrBraking:
    case kAddrVoltage:
    case kAddrOutputCurrent:
    case kAddrRealTimeRMP:
    case kAddrPositionFeedback:
    case kAddrPositionSet:
      return true;
    case kAddrWarningState:
      return true;
    case kAddrSetPosKD: do { if (_cat != kFnFeedback) return true;
        std::int16_t _tick = 0; std::memcpy(&_tick, f.data + 6, sizeof(_tick));
        _tick = endian(_tick); // trans rad per minite to rad per second

        if (_isleft) { _state.flag_ = -1;
          _state.left_rpm_ = static_cast<double>(_tick)*kReadCoef;
        } else { _state.flag_ = 1; // _tick *= -1;
          _state.right_rpm_ = static_cast<double>(_tick)*kReadCoef;
        } if (rpm_) rpm_(_state);
      } while (0);
      return true;
    case kAddrSetPosKP:
    case kAddrObjVelocity:
      return true;
    default:
      return false;
  }
}

void MotorDriver::open() {
  struct sockaddr_can _addr;
  struct ifreq _ifr;

  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  std::strcpy(_ifr.ifr_name, device_.c_str());
  ioctl(fd_, SIOCGIFINDEX, &_ifr);
  _addr.can_family = AF_CAN;
  _addr.can_ifindex = _ifr.ifr_ifindex;
  bind(fd_, (struct sockaddr *)&_addr, sizeof(_addr));

  // struct can_filter _filter[2];
  // _filter[0].can_id = 0x01;
  // _filter[0].can_mask = CAN_SFF_MASK;
  // _filter[1].can_id = 0x10;
  // _filter[1].can_mask = CAN_SFF_MASK;
  // setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &_filter, sizeof(_filter));

  can_err_mask_t _mask = (CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF);
  setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &_mask, sizeof(_mask));
}

void MotorDriver::enable() { // 00 DA 00 10 00 00 00 1F
  static const std::uint8_t _enable[8] = {
    0x00, 0xDA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x1F};
  Frame_t _frame;
  _frame.can_id  = 0x1; _frame.can_dlc = 8;
  std::memcpy(_frame.data, _enable, 8);
  writeData(_frame, sizeof(Frame_t));

  _frame.can_id  = 0x2;
  writeData(_frame, sizeof(Frame_t));
}

void MotorDriver::disable() { // 00 DA 00 10 00 00 00 0F
  Frame_t _frame;
  _frame.can_id  = 0x1; _frame.can_dlc = 8;
  _frame.data[0] = 0x00;
  _frame.data[1] = kFnRequest;
  _frame.data[2] = 0x00;
  _frame.data[3] = kAddrProcess;
  _frame.data[4] = 0x00;
  _frame.data[5] = 0x00;
  _frame.data[6] = 0x00;
  _frame.data[7] = kCtrlCodeRelease;
  writeData(_frame, sizeof(Frame_t));

  _frame.can_id  = 0x2;
  writeData(_frame, sizeof(Frame_t));
}

void MotorDriver::braking() {  // 00 DA 00 30 00 00 00 1F
  Frame_t _frame;
  _frame.can_id  = 0x1; _frame.can_dlc = 8;
  _frame.data[0] = 0x00;
  _frame.data[1] = kFnRequest;
  _frame.data[2] = 0x00;
  _frame.data[3] = kAddrBraking;
  _frame.data[4] = 0x00;
  _frame.data[5] = 0x00;
  _frame.data[6] = 0x00;
  _frame.data[7] = kCtrlCodeEnable;
  writeData(_frame, sizeof(Frame_t));

  _frame.can_id  = 0x2;
  writeData(_frame, sizeof(Frame_t));
}

void MotorDriver::clearWarning() {  // 00 DA 00 15 00 00 00 7F
  Frame_t _frame;
  _frame.can_id  = 0x1; _frame.can_dlc = 8;
  _frame.data[0] = 0x00;
  _frame.data[1] = kFnRequest;
  _frame.data[2] = 0x00;
  _frame.data[3] = kAddrWarningClear;
  _frame.data[4] = 0x00;
  _frame.data[5] = 0x00;
  _frame.data[6] = 0x00;
  _frame.data[7] = kCtrlCodeWarningClear;
  writeData(_frame, sizeof(Frame_t));

  _frame.can_id  = 0x2;
  writeData(_frame, sizeof(Frame_t));
}

void MotorDriver::setAccel() {  // 00 DA 00 13 00 00 32 32
  Frame_t _frame;
  _frame.can_id  = 0x1; _frame.can_dlc = 8;
  _frame.data[0] = 0x00;
  _frame.data[1] = kFnRequest;
  _frame.data[2] = 0x00;
  _frame.data[3] = kAddrAccTime4Vel;
  _frame.data[4] = 0x00;
  _frame.data[5] = 0x00;
  _frame.data[6] = 0x0A;
  _frame.data[7] = 0x0A;
  writeData(_frame, sizeof(Frame_t));

  _frame.can_id  = 0x2;
  writeData(_frame, sizeof(Frame_t));
}

void MotorDriver::velocityMode() {  // 00 DA 00 19 00 00 00 2F
  static const std::uint8_t _vm[8] = {
    0x00, 0xDA, 0x00, 0x19, 0x00, 0x00, 0x00, 0x2F};
  Frame_t _frame;
  _frame.can_id  = 0x1; _frame.can_dlc = 8;
  std::memcpy(_frame.data, _vm, 8);
  writeData(_frame, sizeof(Frame_t));
  _frame.can_id  = 0x2;
  writeData(_frame, sizeof(Frame_t));
}

void MotorDriver::process_error(const int &error) {
  switch (error) {
    case EBADF:
    case ECONNRESET:
    case EFAULT:
    case EINTR:
    case ENOTCONN:
    case EINVAL:
      std::cout << "Error: " << error << std::endl;
      do { close(); open(); } while(0);
      break;
    default: break;
  }
}

int MotorDriver::readData(Frame_t &frame) {
#if 1
  int _count = 0; int _nbytes = 0; int _errno = 0;
  const int _len = static_cast<int>(sizeof(frame));
  do { _nbytes = ::read(fd_, (&frame) + _count, _len - _count);
    _errno = errno;
    process_error(_errno);
    if (-1 == _nbytes) {
      break;
    } else {
      _count += _nbytes;
    }
  } while (_count < _len);

  // std::cout << "can_id: " << frame.can_id << ":  ";
  // for (auto i=0; i<8; ++i) {
  //   printf("%d ", frame.data[i]);
  // } printf("\n");
  return _count;
#else
  auto _nbytes = ::read(fd_, &frame, sizeof(frame));
  int _errno = errno;
  process_error(_errno);
  if (-1 != _nbytes)
    std::cout << "CAN_ID: " << frame.can_id << "  >> ";
  return _nbytes;
#endif
}

int
MotorDriver::write(const Frame_t *fp, const std::size_t &size) {
  // trylock
  int _count = 0; int _nbytes = 0; int _errno = 0;
  const int _len = static_cast<int>(size);
  do { _nbytes = ::write(fd_, fp + _count, _len - _count);
    _errno = errno;
    process_error(_errno);
    if (-1 == _nbytes) {
      break;
    } else {
      _count += _nbytes;
    }
  } while (_count < _len);

  isaac::Sleep(0);
  return _count;
}

void MotorDriver::sendVelocity(const double &l, const double &r) {
  std::int16_t _l = static_cast<std::int16_t>(l * kVelocityCoef);
  std::int16_t _r = static_cast<std::int16_t>(r * kVelocityCoef);
  _l = endian(_l); _r = endian(_r);

  Frame_t _frame;
  _frame.can_id  = 0x1; _frame.can_dlc = 8;

  _frame.data[0] = 0x00;
  _frame.data[1] = kFnRequest;
  _frame.data[2] = 0x00;
  _frame.data[3] = kAddrObjVelocity;
  _frame.data[4] = 0x00;
  _frame.data[5] = 0x00;
  std::memcpy(_frame.data+6, &_l, sizeof(_l));
  writeData(_frame, sizeof(Frame_t));

  _frame.can_id  = 0x2;
  std::memcpy(_frame.data+6, &_r, sizeof(_r));
  writeData(_frame, sizeof(Frame_t));
}

}  // namespace drivers

void ZhonglingRmpDriver::setState(const drivers::MotorDriver::RpmState &state) {
  if (0 == state.flag_ || /*做必要的过滤*/
      isaac::ToSeconds(state.timestamp_-state_.timestamp_) < 0.01)
    return;
  if (0 == state_.flag_) {
    state_ = state;
    return;
  }

  if (state.flag_ == state_.flag_) {  // 破坏交替接收规则，时间差大于2个周期，忽略
    // state_ = state;
    return;
  } else {  // 检测到一次交替就归约为一个有效的状态
    state_.flag_ = 0;
    state_.timestamp_ = state.timestamp_;
    if (-1 == state.flag_) {
      state_.left_rpm_ = state.left_rpm_;
    } else if (1 == state.flag_) {
      state_.right_rpm_ = state.right_rpm_;
    }
  }
}

void ZhonglingRmpDriver::smoothVelocity(double safe_linear, double safe_angular, double left,
    double right, double wheel_separation, double last_linear_speed) {
  if(safe_linear-last_linear_speed<=-0.3 && last_linear_speed==0) {
    for(double i=1; i<=30; i++) {
      motor_->sendVelocity (  // persecond
          (safe_linear-safe_angular*wheel_separation/2.0)*left*(i/30),
          (safe_linear+safe_angular*wheel_separation/2.0)*right*(i/30));
      usleep(50000);
    }
  }
  else if(safe_linear-last_linear_speed>=0.3 && last_linear_speed!=0) {
    for(double i=30; i>=0; i--) {
      motor_->sendVelocity (  // persecond
          (last_linear_speed-safe_angular*wheel_separation/2.0)*left*(i/30),
          (last_linear_speed+safe_angular*wheel_separation/2.0)*right*(i/30));
      usleep(50000);
    }
  }
  else if(safe_linear-last_linear_speed>=0.3 && last_linear_speed==0) {
    for(double i=1; i<=30; i++) {
      motor_->sendVelocity (  // persecond
          (safe_linear-safe_angular*wheel_separation/2.0)*left*(i/30),
          (safe_linear+safe_angular*wheel_separation/2.0)*right*(i/30));
      usleep(50000);
    }
  }
  else if(safe_linear-last_linear_speed<=-0.3 && last_linear_speed!=0) {
    for(double i=30; i>=0; i--) {
      motor_->sendVelocity (  // persecond
          (last_linear_speed-safe_angular*wheel_separation/2.0)*left*(i/30),
          (last_linear_speed+safe_angular*wheel_separation/2.0)*right*(i/30));
      usleep(50000);
    }
  }
  else {
    motor_->sendVelocity (  // per second
        (safe_linear-safe_angular*wheel_separation/2.0)*left,
        (safe_linear+safe_angular*wheel_separation/2.0)*right);
  }
}


void ZhonglingRmpDriver::start() {
  createStateMachine();
  machine_.start(get_start_state());
  failsafe_ = node()->getComponent<alice::Failsafe>();
  tickPeriodically();

  states_.resize(get_VelocityRollingWindowSize());
  motor_ = std::make_unique<drivers::MotorDriver>(get_Device());
  motor_->open();
  motor_->clearWarning();
  motor_->option();
  motor_->setRmpFunc(
      std::bind(&ZhonglingRmpDriver::setState, this, std::placeholders::_1));

  run_ = true;
  thread_ = std::make_unique<std::thread> (
      [this]()->void {
        std::cout << "***------------------------------***" << std::endl;
        drivers::Frame_t _frame;
        while (run_) {
          auto _nbytes = this->motor_->readData(_frame);
          if (_nbytes > 0) {
            motor_->decode(_frame); update();
          } else {
            std::cout << "read data length: " << _nbytes << std::endl;
          }
        }
      });

  // stop robot if the failsafe is triggered
  if (!failsafe_->isAlive()) {
    // safe_speed = 0.0;
    // safe_turning_rate = 0.0;
  }
}

#if 0
void ZhonglingRmpDriver::tick() {
  std::unique_lock<std::mutex> _locker(lock_, std::defer_lock);
  if (!_locker.try_lock()) {
    std::cout << "ZhonglingRmpDriver::tick waiting for locker ..." << std::endl;
    return;
  }

  const double _wheel_separation = get_WheelSeparation();
  const double _wheel_radius = get_WheelRadius();
  const double _speed_limit_linear = get_SpeedLimitLinear();
  const double _speed_limit_angular = get_SpeedLimitAngular();

  const double _leftWheelMultiplier = get_LeftWheelMultiplier();
  const double _rightWheelMultiplier = get_RightWheelMultiplier();

  if (rx_zl_ctrl().available()) {
    auto _ctrl = rx_zl_ctrl().getProto();
    auto _axes = _ctrl.getAxes();
    auto _safe_linear = _axes[0].getY();
    auto _safe_angular = _axes[1].getX();
    //std::cout << "linear:" << _safe_linear << std::endl;
    //std::cout << "angular:" << _safe_angular << std::endl;
    if (std::abs(_safe_linear) > _speed_limit_linear) {
      _safe_linear = std::copysign(_speed_limit_linear, _safe_linear);
    }
    if (std::abs(_safe_angular) > _speed_limit_angular) {
      _safe_angular = std::copysign(_speed_limit_angular, _safe_angular);
    }
    /*if (!failsafe_->isAlive()) {
      _safe_linear = 0.0; _safe_angular = 0.0;
    }*/
    auto _left = _leftWheelMultiplier / _wheel_radius;
    auto _right = _rightWheelMultiplier / _wheel_radius;
    smoothVelocity(_safe_linear, _safe_angular, _left, _right, _wheel_separation, last_linear_speed_);
    last_linear_speed_ = _safe_linear;
  } //rx_zl_ctrl().available

  if (rx_zl_cmd().available()) {  // 下发流程
    messages::DifferentialBaseControl _cmd;
    FromProto(rx_zl_cmd().getProto(), _cmd);
    double _safe_linear = -_cmd.linear_speed();
    double _safe_angular = -_cmd.angular_speed();
    if (std::abs(_safe_linear) > _speed_limit_linear)
      //_safe_linear = std::copysign(_speed_limit_linear, _cmd.linear_speed());
      _safe_linear = 0;
    if (_safe_linear > 0 && _safe_linear != 0.5)
      _safe_linear = 0;
    if (std::abs(_safe_angular) > _speed_limit_angular) {
      _safe_angular = std::copysign(
          _speed_limit_angular, _cmd.angular_speed());
    }
    //std::cout << "linear:" << _safe_linear << std::endl;
    //std::cout << "angular:" << _safe_angular << std::endl;
    auto _left = _leftWheelMultiplier / _wheel_radius;
    auto _right = _rightWheelMultiplier / _wheel_radius;
    smoothVelocity(_safe_linear, _safe_angular, _left, _right, _wheel_separation, last_linear_speed_cmd_);
    last_linear_speed_cmd_ = _safe_linear;
  } // rx_zl_cmd().available
}
#endif

void ZhonglingRmpDriver::tick() {
  std::unique_lock<std::mutex> _locker(lock_, std::defer_lock);
  if (!_locker.try_lock()) {
    std::cout << "ZhonglingRmpDriver::tick waiting for locker ..." << std::endl;
    return;
  }
  machine_.tick();
}

void ZhonglingRmpDriver::stop() {
  run_ = false;
  motor_->close();
  motor_.reset();
  if (thread_) {
    thread_->detach();
    thread_.reset();
  }
}

void ZhonglingRmpDriver::createStateMachine() {
  const std::vector<State> all_states = {
    kStateCmd,
    kStateCtrl,
  };
  machine_.setToString([this] (const State& state) {return state;});
  machine_.addState(kStateCmd,
                    [this] {
                      std::cout << "enter Cmd channel" << std::endl;
                    },
                    [this] {
                      const double _wheel_separation = get_WheelSeparation();
                      const double _wheel_radius = get_WheelRadius();
                      const double _speed_limit_linear = get_SpeedLimitLinear();
                      const double _speed_limit_angular = get_SpeedLimitAngular();
                      const double _leftWheelMultiplier = get_LeftWheelMultiplier();
                      const double _rightWheelMultiplier = get_RightWheelMultiplier();
                      if (rx_zl_cmd().available()) {  // 下发流程
                        messages::DifferentialBaseControl _cmd;
                        FromProto(rx_zl_cmd().getProto(), _cmd);
                        double _safe_linear = -_cmd.linear_speed();
                        double _safe_angular = -_cmd.angular_speed();
                        if (std::abs(_safe_linear) > _speed_limit_linear)
                          _safe_linear = 0;
                        if (_safe_linear > 0 && _safe_linear != 0.5)
                          _safe_linear = 0;
                        if (std::abs(_safe_angular) > _speed_limit_angular) {
                          _safe_angular = std::copysign(
                              _speed_limit_angular, _cmd.angular_speed());
                        }
                        auto _left = _leftWheelMultiplier / _wheel_radius;
                        auto _right = _rightWheelMultiplier / _wheel_radius;
                        smoothVelocity(_safe_linear, _safe_angular, _left, 
                            _right, _wheel_separation, last_linear_speed_cmd_);
                        last_linear_speed_cmd_ = _safe_linear;
                      } // rx_zl_cmd().available
                    },
                    [this] {
                      std::cout << "switch to Ctrl channel" << std::endl;
                    }
  );
  machine_.addState(kStateCtrl,
                    [this] {
                      std::cout << "enter Ctrl channel" << std::endl;
                    },
                    [this] {
                      const double _wheel_separation = get_WheelSeparation();
                      const double _wheel_radius = get_WheelRadius();
                      const double _speed_limit_linear = get_SpeedLimitLinear();
                      const double _speed_limit_angular = get_SpeedLimitAngular();
                      const double _leftWheelMultiplier = get_LeftWheelMultiplier();
                      const double _rightWheelMultiplier = get_RightWheelMultiplier();
                      if (rx_zl_ctrl().available()) {
                        auto _ctrl = rx_zl_ctrl().getProto();
                        auto _axes = _ctrl.getAxes();
                        auto _safe_linear = _axes[0].getY();
                        auto _safe_angular = _axes[1].getX();
                        if (std::abs(_safe_linear) > _speed_limit_linear) {
                          _safe_linear = std::copysign(_speed_limit_linear, _safe_linear);
                        }
                        if (std::abs(_safe_angular) > _speed_limit_angular) {
                          _safe_angular = std::copysign(_speed_limit_angular, _safe_angular);
                        }
                        auto _left = _leftWheelMultiplier / _wheel_radius;
                        auto _right = _rightWheelMultiplier / _wheel_radius;
                        smoothVelocity(_safe_linear, _safe_angular, _left, 
                            _right, _wheel_separation, last_linear_speed_);
                        last_linear_speed_ = _safe_linear;
                      } //rx_zl_ctrl().available
                    }, 
                    [this] {
                      std::cout << "switch to Cmd channel" << std::endl;
                    }
  );
  machine_.addTransition(kStateCmd, kStateCtrl,
                         [this] {
                            if(rx_channel_switch().available()) {
                              const auto switch_proto = rx_channel_switch().getProto();
                              channel_switch_[0] = switch_proto.getX();
                            }
                            if(channel_switch_[0] == 2)
                              return true;
                            else
                              return false;
                         },
                         [] {}
  );
  machine_.addTransition(kStateCtrl, kStateCmd,
                         [this] {
                            if(rx_channel_switch().available()) {
                              const auto switch_proto = rx_channel_switch().getProto();
                              channel_switch_[0] = switch_proto.getX();
                            }
                            if(channel_switch_[0] == 1)
                              return true;
                            else
                              return false;
                         },
                         [] {}
  );
}

bool ZhonglingRmpDriver::update () {
  auto _state = getState();
  if (0 != _state.flag_) {  // 状态处理：0表示左右轮消息交替到达,归约
    //std::cout << "XXXXXXXXXXXXXXXX" << std::endl;
    return false;         // 无效状态，退出
  }

  const double _wheel_separation = get_WheelSeparation();
  const double _wheel_radius = get_WheelRadius();
  const double _leftWheelMultiplier = get_LeftWheelMultiplier();
  const double _rightWheelMultiplier = get_RightWheelMultiplier();

  const double _lp = _leftWheelMultiplier * _state.left_rpm_ * _wheel_radius;
  const double _rp = _rightWheelMultiplier * _state.right_rpm_ * _wheel_radius;

#if 0  // 正常情况:左轮前进时编码器值增大，右轮前进时编码器值减小
  const double _linear = (_lp + _rp) * 0.5;
  const double _angular = (_rp - _lp) / _wheel_separation;
#else  // 电机编码器方向与实际运动方向相反地情况
  const double _linear = -(_lp + _rp) * 0.5;
  const double _angular = -(_rp - _lp) / _wheel_separation;
#endif

  auto _velocity = Velocity_t(_linear, _angular, _state.timestamp_);
  if (!states_.isEmpty()) {
    auto _back = states_.back();
    const double _dt = isaac::ToSeconds(_state.timestamp_ - _back.timestamp_);
    // We cannot estimate the speed with very small time intervals
    if (_dt > 0.001) {
      _velocity.lin_acc_ = (_linear - _back.linear_) / _dt;
      _velocity.ang_acc_ = (_angular - _back.angular_) / _dt;
    }
  } states_.push_back(_velocity);
  if (!states_.isFull()) return false;

  auto _ret = Velocity_t();
  _ret.linear_ = _linear; _ret.angular_ = _angular;
  do {  // 积分求均值
    auto _s = states_.size();
    for (std::size_t i=1; i < _s; ++i) {
      auto __dt = states_[i].timestamp_ - states_[i-1].timestamp_;
      _ret.timestamp_ += __dt;
      _ret.lin_acc_ += states_[i].lin_acc_ * __dt;
      _ret.ang_acc_ += states_[i].ang_acc_ * __dt;
    }
    _ret.lin_acc_ /= _ret.timestamp_;
    _ret.ang_acc_ /= _ret.timestamp_;
  } while (0);

  do {  // publish
    auto _msg = tx_zl_state().initProto();
    _msg.setLinearSpeed(_ret.linear_);
    _msg.setLinearAcceleration(0.5 * _ret.lin_acc_);
    _msg.setAngularSpeed(_ret.angular_);
    _msg.setAngularAcceleration(_ret.ang_acc_);
    tx_zl_state().publish();
  } while(0);

  // visualization
  // show("linear", _ret.linear_); show("angular", _ret.angular_);

  return true;
}  // update

}  // namespace isaac
