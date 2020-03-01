/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "RealsenseCameraSimple.hpp"

#include <memory>
#include <string>
#include <utility>

#include "engine/gems/geometry/pinhole.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/utils.hpp"
#include "messages/camera.hpp"

namespace isaac {

namespace {

// Check the current firmware version vs. the recommended firmware version and
// log a warning if they do not match
void LogWarningIfNotRecommendedFirmwareVersion(const rs2::device& dev) {
  const std::string current = dev.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);
  const std::string recommended = dev.get_info(RS2_CAMERA_INFO_RECOMMENDED_FIRMWARE_VERSION);

  // if firmware version is different from recommended version, log recommended version
  if (current != recommended) {
    LOG_WARNING("Realsense recommended firmware version is %s, "
                "currently using firmware version %s",
                recommended.c_str(), current.c_str());
  }
}

// Set a sensor option
void SetSensorOption(const rs2_option& option, float value, rs2::sensor& sensor) {
  if (!sensor.supports(option)) {
    // we try to keep the sensors in sync, so it is normal that not all sensors
    // will support all options
    return;
  }

  try {
    // set the option
    sensor.set_option(option, value);
  } catch (const rs2::error& e) {
    // let the user know something went wrong
    const std::string sensorName = sensor.get_info(RS2_CAMERA_INFO_NAME);
    const std::string optionLabel = rs2_option_to_string(option);
    const rs2::option_range range = sensor.get_option_range(option);
    LOG_WARNING("Failed to set '%s' option '%s' to %f (min:%f, max:%f, default:%f, step:%f)",
        sensorName.c_str(), optionLabel.c_str(), value,
        range.min, range.max, range.def, range.step);
  }
}

}  // namespace

RealsenseCameraSimple::RealsenseCameraSimple() {}
RealsenseCameraSimple::~RealsenseCameraSimple() {}

// Stores the Realsense pipeline and alignment filter
struct RealsenseCameraSimple::Impl {
  rs2::pipeline pipe;
  rs2::align align_to = rs2::align(RS2_STREAM_COLOR);
};

#if 1 // added by qiaoyx
void RealsenseCameraSimple::depthimageToLaserscan (
    int64_t acqtime, Image1f& depth, const geometry::Pinhole<double>& pinhole) {
  auto _skipRow = get_skip_row();
  auto _skipCol = get_skip_column();
  auto _minDistance = get_min_distance();
  auto _maxDistance = get_max_distance();
  auto _minHeight = get_height_min();
  auto _maxHeight = get_height_max();
  auto _fovH = get_fov_h();

  auto _scan = tx_scan().initProto();
  const int _row = std::ceil(depth.rows() / _skipRow);
  const int _col = std::ceil(depth.cols() / _skipCol);
  /*
  std::cout << _row << ";" << _col << std::endl;
  180;320
  */
  auto _angles = _scan.initAngles(_col+1);
  auto _ranges = _scan.initRanges(_col+1);

  const auto _increment = DegToRad(_fovH) / _col;
  const auto _angle_max = DegToRad(_fovH) / 2.0;

  for (int i = 0; i < _col+1; ++i) {
    _angles.set(_col - i, _angle_max - i*_increment);
    double _range = NAN;
    for (int j = 0; j < _row; ++j) {
      double _x = depth(j*_skipRow, i*_skipCol);
      if (!(std::isnan(_x) || std::isinf(_x))) {
        auto _z = -(j*_skipRow - pinhole.center[0]) * _x / pinhole.focal[0];
        if (_z >= _minHeight && _z <= _maxHeight) {
          auto _y = - (i*_skipCol - pinhole.center[1]) * _x / pinhole.focal[1];
          auto _r = std::sqrt(_x*_x + _y*_y);
          if (_r >= _minDistance && _r <= _maxDistance) {
            if (!std::isnan(_range) && !std::isinf(_range)) {
              _range = std::min(_r, _range);
            } else {
              _range = _r;
            }
          }
        }
      }
    } _ranges.set(_col - i, _range);
  }

  /*std::cout << "**************************" << std::endl;
  for(int i=0; i<_col+1; i++) {
    std::cout << "range:" << _ranges[i] << std::endl;
    //std::cout << "angle:" << _angles[i] << std::endl;
    //std::cout << "theta:" << asin(0.1/_ranges[i]) << std::endl;
  }*/


#if 1 // added by dengke
  if(get_use_radius_filter()) {
    //第1层滤波
    for(int i=0; i<_col; i++) {
      if(_ranges[i] == 0) {
        _ranges.set(i, NAN);
      }
      else if(!std::isnan(_ranges[i])) {
        double _theta = asin(get_filter_radius()/_ranges[i]);
        int _search_range = round(_theta/_increment);
        //std::cout << "theta:" << _theta << std::endl;
        //std::cout << "search_range:" << _search_range << std::endl;
	if(i-_search_range>=0 && i+_search_range<=_col) {
          int count = 0;
          for(int j=i-_search_range; j<i+_search_range; j++) {
            double _alpha = abs(_angles[j] - _angles[i]);
            double _distance_i_j = std::sqrt(pow(_ranges[i],2) + pow(_ranges[j],2) - 2*cos(_alpha)*_ranges[i]*_ranges[j]);
	    if(_distance_i_j <= get_filter_radius()) {
              count++;
            }
          }
	  if(count <= get_filter_count()) {
            _ranges.set(i, NAN);
	  }
        }
      }
    }
    //第2层滤波
    for(int i=10; i<_col-10; i++) {
      int _count1 = 0;
      for(int j=i-10; j<i+10; j++) {
        if(abs(_ranges[j]-_ranges[i]) <= 0.05)
          _count1++;
      }
      if(_count1 <= 3)
        _ranges.set(i, NAN);
    }
  }
#endif // the end

  /*std::cout << "**************************" << std::endl;
  for(int i=0; i<_col+1; i++) {
    std::cout << "range:" << _ranges[i] << std::endl;
    //std::cout << "angle:" << _angles[i] << std::endl;
    //std::cout << "theta:" << asin(0.1/_ranges[i]) << std::endl;
  }*/

  _scan.setInvalidRangeThreshold(_minDistance);
  _scan.setOutOfRangeThreshold(_maxDistance);
  tx_scan().publish(acqtime);
}
#endif // the end

void RealsenseCameraSimple::start() {
  try {
    impl_ = std::make_unique<Impl>();

    // get a list of realsense devices connected
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();

    // are any devices connected?
    if (devices.size() == 0) {
      reportFailure("No device connected, please connect a RealSense device");
      return;
    }

    // Go through each connected device, check the firmware, and configure
    for (const rs2::device& dev : devices) {
      LogWarningIfNotRecommendedFirmwareVersion(dev);
      configureDevice(dev);
    }

    // configure the pipeline, enable depth and color stream
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, get_cols(), get_rows(), RS2_FORMAT_Z16, get_framerate());
    cfg.enable_stream(RS2_STREAM_COLOR, get_cols(), get_rows(), RS2_FORMAT_RGB8, get_framerate());

    // start the pipeline
    impl_->pipe.start(cfg);
  } catch(const rs2::error& e) {
    reportFailure("RealSense error calling %s(%s): %s", e.get_failed_function().c_str(),
                  e.get_failed_args().c_str(), e.what());
  }

  tickBlocking();
}

void RealsenseCameraSimple::tick() {
  const int64_t acqtime = node()->clock()->timestamp();
  try {
    rs2::frameset frames = impl_->pipe.wait_for_frames();
    if (get_align_to_color()) {
      frames = frames.apply_filter(impl_->align_to);
    }

    // color image
    rs2::video_frame color_frame = frames.get_color_frame();
    CpuAlignedBufferConstView color_buffer(color_frame.get_data(), color_frame.get_height(),
                                           color_frame.get_stride_in_bytes());
    ImageConstView3ub color_image_view(color_buffer, color_frame.get_height(),
                                       color_frame.get_width());
    Image3ub color_image(color_image_view.dimensions());
    Copy(color_image_view, color_image);
    auto color_intrinsics =
        color_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    geometry::Pinhole<double> color_pinhole{
      color_frame.get_height(), color_frame.get_width(),
      Vector2d{color_intrinsics.fy, color_intrinsics.fx},
      Vector2d{color_intrinsics.ppy, color_intrinsics.ppx}
    };
    auto proto_color = tx_color().initProto();
    proto_color.setColorSpace(ColorCameraProto::ColorSpace::RGB);
    ToProto(color_pinhole, proto_color.initPinhole());
    ToProto(std::move(color_image), proto_color.initImage(), tx_color().buffers());
    tx_color().publish(acqtime);

    // depth image
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    CpuAlignedBufferConstView depth_buffer(depth_frame.get_data(), depth_frame.get_height(),
                                           depth_frame.get_stride_in_bytes());
    ImageConstView1ui16 depth_image_view(depth_buffer, depth_frame.get_height(),
                                         depth_frame.get_width());
    Image1f depth_image(depth_frame.get_height(), depth_frame.get_width());
    ConvertUi16ToF32(depth_image_view, depth_image, 0.001);
    auto depth_intrinsics =
        depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    geometry::Pinhole<double> depth_pinhole{
      depth_frame.get_height(), depth_frame.get_width(),
      Vector2d{depth_intrinsics.fy, depth_intrinsics.fx},
      Vector2d{depth_intrinsics.ppy, depth_intrinsics.ppx}
    };
  //geometry::Pinhole<double> pinhole;
#if 1 // added by qiaoyx
  if (get_depthimage_to_laserscan()) {
    depthimageToLaserscan(acqtime, depth_image, depth_pinhole);
  }
#endif // the end
    auto proto_depth = tx_depth().initProto();
    ToProto(depth_pinhole, proto_depth.initPinhole());
    ToProto(std::move(depth_image), proto_depth.initDepthImage(), tx_depth().buffers());
    proto_depth.setMinDepth(0.0);
    proto_depth.setMaxDepth(10.0);
    tx_depth().publish(acqtime);
  } catch(const rs2::error& e) {
    reportFailure("RealSense error calling %s(%s): %s", e.get_failed_function().c_str(),
                  e.get_failed_args().c_str(), e.what());
  }
}

void RealsenseCameraSimple::stop() {
  try {
    impl_.reset();
  } catch(const rs2::error& e) {
    reportFailure("RealSense error calling %s(%s): %s", e.get_failed_function().c_str(),
                  e.get_failed_args().c_str(), e.what());
  }
}

// Initial configuration of the realsense devices
void RealsenseCameraSimple::configureDevice(const rs2::device& dev) {
  // NOTE: this method is called before the pipeline is started, and not all
  // options can be configured before the sensor starts.
  for (auto& sensor : dev.query_sensors()) {
    SetSensorOption(RS2_OPTION_FRAMES_QUEUE_SIZE, get_frame_queue_size(), sensor);
    SetSensorOption(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, get_auto_exposure_priority(), sensor);
  }
}


}  // namespace isaac
