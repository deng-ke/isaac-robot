/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "engine/alice/alice_codelet.hpp"
#include "messages/messages.hpp"

namespace isaac {

// Available color modes for the Color sensor
enum ColorMode {
    kRgb,
    kGrayscale,
    kInvalid = -1
};

// Mapping between each ColorMode type and an identifying string.
// See nlohmann/json for detailed documentation:
// https://github.com/nlohmann/json#specializing-enum-conversion
NLOHMANN_JSON_SERIALIZE_ENUM(ColorMode, {
  { kInvalid, nullptr },
  { kRgb, "Rgb" },
  { kGrayscale, "Grayscale" }
});

// Isaac codelet for Realsense D435 camera, that comes with two sensors (color and depth)
// Please note the color camera supports more image formats (RGB8, Y16, BGRA8, RGBA8, BGR8, YUYV).
// In this codelet we only support RGB8 and Y16. Support for other formats can be added as
// necessary.
// These resolutions and settings have been tested on firmware version 05.10.03
// To update/downgrade the firmware version on the sensor, follow the steps from this page
// https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Linux-RealSense-D400-DFU-Guide.pdf
//
// Color sensor supported modes
// 1 : 1920x1080@ 30Hz, 15Hz, 6Hz
// 2 : 1280x720@ 30Hz, 15Hz, 6Hz
// 3 : 960x540@ 60Hz, 30Hz, 15Hz, 6Hz
// 4 : 848x480@ 60Hz, 30Hz, 15Hz, 6Hz
// 6 : 640x480@ 60Hz, 30Hz, 15Hz, 6Hz
// 7 : 640x360@ 60Hz, 30Hz, 15Hz, 6Hz
// 8 : 424x240@ 60Hz, 30Hz, 15Hz, 6Hz
// 9 : 320x240@ 60Hz, 30Hz, 15Hz, 6Hz
// 10 : 320x180@ 60Hz, 30Hz, 15Hz, 6Hz
//
// Ir (left and right) stream supported modes
// 1 : Infrared 1280x800@ 30Hz, 15Hz (Not supported when depth stream is active)
// 2 : Infrared 1280x720@ 30Hz, 15Hz, 6Hz
// 3 : Infrared 848x480@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
// 4 : Infrared 640x480@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
// 4 : Infrared 640x360@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
// 5 : Infrared 480x270@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
// 6 : Infrared 424x240@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
//
// Depth stream supported modes
// 1 : Depth 1280x720@ 30Hz, 15Hz, 6Hz
// 2 : Depth 848x480@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
// 3 : Depth 640x480@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
// 4 : Depth 640x360@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
// 5 : Depth 480x270@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
// 6 : Depth 424x240@ 90Hz, 60Hz, 30Hz, 15Hz, 6Hz
class RealsenseCamera : public alice::Codelet {
 public:
  RealsenseCamera();
  ~RealsenseCamera();

  void start() override;
  void tick() override;
  void stop() override;

  // Color camera image. This can be Image3ub(for color) or Image1ui16 (for grayscale)
  ISAAC_PROTO_TX(ColorCameraProto, color);
  // Left Ir camera image
  ISAAC_PROTO_TX(ColorCameraProto, ir_left);
  // Right Ir camera image
  ISAAC_PROTO_TX(ColorCameraProto, ir_right);
  // Depth image (in meters). This is in left Ir camera frame.
  ISAAC_PROTO_TX(DepthCameraProto, depth);

  // Extrinsics from ir_left to ir_right
  ISAAC_POSE3(ir_left, ir_right);
  // Extrinsics from ir_left to color
  ISAAC_POSE3(ir_left, color);

  // Index of the camera, when working with multiple realsense cameras.
  // Each realsense camera consists of an RGB sensor and two IR sensors, each of which get
  // enumerated as /dev/video*. So we cannot use /dev/video* (like ZedCamera and V4L2Camera) to
  // uniquely identify a d435 device. The index here identifies the index of the camera in a list
  // that is sorted by serial number of the camera.
  ISAAC_PARAM(int, index, 0);
  // Color camera mode. Cannot be changed after the device starts
  ISAAC_PARAM(ColorMode, color_mode, kRgb);
  // Color image size. Cannot be changed after the device starts
  ISAAC_PARAM(Vector2i, color_size, Vector2i(1080, 1920));
  // Color camera frame rate. Cannot be changed after the device starts
  ISAAC_PARAM(int, color_fps, 30);

  // Enable/disable depth stream. Cannot be changed after the device starts
  ISAAC_PARAM(bool, enable_depth, true);
  // Enable/disable left IR stream. Cannot be changed after the device starts
  ISAAC_PARAM(bool, enable_ir_left, false);
  // Enable/disable right IR stream. Cannot be changed after the device starts
  ISAAC_PARAM(bool, enable_ir_right, false);
  // Depth/IR image size . Cannot be changed after the device starts
  ISAAC_PARAM(Vector2i, depth_size, Vector2i(720, 1280));
  // Depth/IR frame rate. Cannot be changed after the device starts
  ISAAC_PARAM(int, depth_fps, 30);

  // The default values for all the below parameters have been captured by running the
  // rs_sensor_control app that comes with the realsense sdk and then querying for the default
  // values, ranges and step size for each of them.

  // Enable/Disable Color Backling Compensation.
  // Range [0, 1].
  ISAAC_PARAM(int, color_backlight_compensation, 0);
  // Color Image Brightness.
  // Range [-64, 64].
  ISAAC_PARAM(int, color_brightness, 0);
  // Color Image Contrast.
  // Range[0, 100]
  ISAAC_PARAM(int, color_contrast, 50);
  // Enable/disable color image auto-exposure.
  // Range [0, 1].
  ISAAC_PARAM(int, color_enable_auto_exposure, 1);
  // Controls exposure time of color camera. Setting any value will disable auto exposure
  // Range [41, 10000].
  ISAAC_PARAM(int, color_exposure, 166);
  // Color Image Gain.
  // Range [0, 128].
  ISAAC_PARAM(int, color_gain, 64);
  // Color Image gamma setting.
  // Range [100, 500].
  ISAAC_PARAM(int, color_gamma, 300);
  // Color Image hue.
  // Range [-180, 180].
  ISAAC_PARAM(int, color_hue, 0);
  // Color Image Saturation.
  // Range [0, 100].
  ISAAC_PARAM(int, color_saturation, 64);
  // Color Image Sharpness.
  // Range [0, 100].
  ISAAC_PARAM(int, color_sharpness, 50);
  // Controls white balance of color image. Setting any value will disable auto white balance
  // Range [2800, 6500]. Step size 10
  ISAAC_PARAM(int, color_white_balance, 4600);
  // Enable/Disable auto white balance.
  // Range [0, 1].
  ISAAC_PARAM(int, color_enable_auto_white_balance, 1);
  // Max number of frames you can hold at a given time. Increasing this number will reduce frame
  // drops but increase latency, and vice versa.
  // Range [0, 32].
  ISAAC_PARAM(int, color_frames_queue_size, 2);
  // Power Line Frequency control for anti-flickering Off/50Hz/60Hz/Auto.
  // Range [0, 2].
  ISAAC_PARAM(int, color_power_line_frequency, 3);
  // Allows sensor to dynamically ajust the frame rate depending on lighting conditions
  // Range [0, 1].
  ISAAC_PARAM(int, color_auto_exposure_priority, 0);

  // Depth sensor exposure time in microseconds. Setting any value will disable auto exposure
  // Range [0, 166000]. Step size 20
  ISAAC_PARAM(int, depth_exposure, 8500);
  // Enable/disable depth image auto-exposure.
  // Range [0, 1].
  ISAAC_PARAM(int, depth_enable_auto_exposure, 1);
  // Depth Image Gain.
  // Range [16, 248].
  ISAAC_PARAM(int, depth_gain, 16);
  // Provide access to several recommended sets of option presets for the depth camera
  // Range [0, 6].
  ISAAC_PARAM(int, depth_visual_preset, 0);
  // Manual laser power in mw. applicable only when laser power mode is set to Manual
  // Range [0, 360]. Step size 30
  ISAAC_PARAM(int, depth_laser_power, 150);
  // Power Control for D400 Projector, 0-off, 1-on, (2-deprecated)
  // Range [0, 1].
  ISAAC_PARAM(bool, depth_emitter_enabled, false);
  // Max number of frames you can hold at a given time. Increasing this number will reduce frame
  // drops but increase latency, and vice versa.
  // Range [0, 32].
  ISAAC_PARAM(int, depth_frames_queue_size, 2);
  // Enable / disable polling of camera internal errors
  // Range [0, 1].
  ISAAC_PARAM(int, depth_error_polling_enabled, 1);
  // Generate trigger from the camera to external device once per frame
  // Range [0, 1].
  ISAAC_PARAM(int, depth_output_trigger_enabled, 0);
  // Inter-camera synchronization mode: 0:Default, 1:Master, 2:Slave
  // Range [0, 2].
  ISAAC_PARAM(int, depth_inter_cam_sync_mode, 1);

#if 1 // added by dengke
  ISAAC_PARAM(bool, use_radius_filter, false);
  ISAAC_PARAM(double, filter_radius, 0.1);
  ISAAC_PARAM(double, filter_count, 10);
#endif // the end

#if 1 // added by qiaoyx
  ISAAC_PROTO_TX(FlatscanProto, scan);

  ISAAC_PARAM(bool, depthimage_to_laserscan, false);
  ISAAC_PARAM(int, skip_row, 4);
  ISAAC_PARAM(int, skip_column, 4);
  ISAAC_PARAM(double, min_distance, 0.2);
  ISAAC_PARAM(double, max_distance, 5.0);
  ISAAC_PARAM(double, height_min, -0.6);
  ISAAC_PARAM(double, height_max, 0.1);
  ISAAC_PARAM(double, fov_h, 90);
#endif // the end


 private:
  // Read the sensor options from config and update those on the device
  void updateSensorOptions();
  // Forward declaration for pimpl
  class RealsenseCameraImpl;
  std::unique_ptr<RealsenseCameraImpl> impl_;

#if 1 // added by qiaoyx
  void depthimageToLaserscan(int64_t, Image1f&, const geometry::Pinhole<double>&);
#endif // the end

};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::RealsenseCamera)
