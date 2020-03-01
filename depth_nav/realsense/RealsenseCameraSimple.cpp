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
