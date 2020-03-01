#include "IsaacYdlidar.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

using namespace ydlidar;
namespace isaac {
    namespace {
        constexpr double kMaxSafeSpeedLimit = 2.0;
        constexpr double kMaxSafeTurningRateLimit = 2.0;

        std::vector<float> split(const std::string &s, char delim) {
            std::vector<float> elems;
            std::stringstream ss(s);
            std::string number;
            while (std::getline(ss, number, delim)) {
                elems.push_back(atof(number.c_str()));
            }
            return elems;
        }
    }  // namespace

    IsaacYdlidar::IsaacYdlidar() {}
    IsaacYdlidar::~IsaacYdlidar() {}

    void IsaacYdlidar::start() {
        printf("__   ______  _     ___ ____    _    ____  \n");
        printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
        printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
        printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
        printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
        printf("\n");
        fflush(stdout);

        // tickOnMessage(rx_trigger());

        ydlidar_.reset(new CYdLidar());

        const auto _frame = get_frame_id();
        const auto _ignore_list = get_ignore_list();
        auto _frequency = get_frequency();
        auto _angle_max = get_angle_max();
        auto _angle_min = get_angle_min();
        auto _range_max = get_range_max();
        auto _range_min = get_range_min();

        std::vector<float> _ignores = split(_ignore_list, ',');
        if (_ignores.size() % 2) {
            LOG_ERROR("ignore array is odd need be even");
        }

        for (auto _e : _ignores)
            if (_e < 180 || _e > 180)
                LOG_ERROR(
                    "ignore array should be between -180 and 180");

        if (_frequency < 5)
            _frequency = 7.0;
        if (_frequency > 12)
            _frequency = 12.0;

        if (_angle_max < _angle_min) {
            auto _temp = _angle_max;
            _angle_max = _angle_min;
            _angle_min = _temp;
        }

        if (_range_max < _range_min) {
            auto _temp = _range_max;
            _range_max = _range_min;
            _range_min = _temp;
        }

        ydlidar_->setSerialPort(get_port());
        ydlidar_->setSerialBaudrate(get_baudrate());
        ydlidar_->setMaxRange(_range_max);
        ydlidar_->setMinRange(_range_min);
        ydlidar_->setMaxAngle(_angle_max);
        ydlidar_->setMinAngle(_angle_min);
        ydlidar_->setScanFrequency(_frequency);
        ydlidar_->setSampleRate(get_samp_rate());
        ydlidar_->setIntensities(get_intensity());
        ydlidar_->setReversion(get_reversion());
        ydlidar_->setFixedResolution(get_resolution_fixed());
        ydlidar_->setAutoReconnect(get_auto_reconnect());
        ydlidar_->setExposure(get_low_exposure());
        ydlidar_->setIgnoreArray(_ignores);

        ydlidar_->initialize();

        tickBlocking();
    }

    void IsaacYdlidar::tick() {
        auto _msg = tx_scan().initProto();
        LaserScan _scanRaw;
        do {
            bool _hardError;
            if (!ydlidar_->doProcessSimple(_scanRaw, _hardError)) {
                return;
            }

            _msg.setInvalidRangeThreshold(ydlidar_->getMinRange());
            _msg.setOutOfRangeThreshold(ydlidar_->getMaxRange());
            // _msg.setDeltaTime(_scanRaw.config.time_increment);

            const int _raysNum = _scanRaw.ranges.size();
            _msg.initRanges(_raysNum);
            _msg.initAngles(_raysNum);

            for (auto i=0; i < _raysNum; ++i) {
                _msg.getRanges().set(i, _scanRaw.ranges[i]);
                auto _a = i * 0.5 /*_scanRaw.config.ang_increment*/ *
                    M_PI / 180.f - M_PI;
                _msg.getAngles().set(i, _a);
            }
        } while (0);

        // for (auto r : tx_scan.getRanges()) {
        //     printf("range: %f    ", r);
        // } printf("\n");
        tx_scan().publish();

#if 1
        show("scan",
             [&](sight::Sop& sop) {
                 // sop.transform = sight::SopTransform{wolrd_T_robot};
                 sop.style = sight::SopStyle{"red"};
                 sop.add(geometry::Circled({{0.0, 0.0}, 1.0}));
                 sop.add([&](sight::Sop& sop) { // Recursive call
                             sop.style = sight::SopStyle("#ff8c00");
                             auto _rang = _msg.getRanges();
                             auto _angl = _msg.getAngles();
                             for (std::size_t i = 0; i < _rang.size(); ++i) {
                                 auto _x = _rang[i] * std::cos(_angl[i]);
                                 auto _y = _rang[i] * std::sin(_angl[i]);
                                 sop.add(geometry::Circled({{_x, _y}, 0.1}));
                             }
                         });
             });
#endif
    } // tick()

    void IsaacYdlidar::stop() {
        ydlidar_->turnOff();
        printf("[YDLIDAR INFO] Now YDLIDAR is stopping .......\n");
        ydlidar_->disconnecting();
        ydlidar_.reset();
    }
} // namespace