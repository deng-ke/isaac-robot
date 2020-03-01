/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "RealsenseCamera.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <math.h>

#include "engine/core/singleton.hpp"
#include "engine/gems/geometry/pinhole.hpp"
#include "engine/gems/image/utils.hpp"
#include "librealsense2/h/rs_pipeline.h"
#include "librealsense2/rs.h"
#include "librealsense2/rs_advanced_mode.h"
#include "messages/camera.hpp"

namespace isaac {

namespace {
// Depth range for the realsense d435 sensor
constexpr float kMinDepthMeters = 0.2f;
constexpr float kMaxDepthMeters = 10.0f;
// Factor by which to scale images down when displayed in Sight
constexpr int kSightReduceSize = 2;
// Constant used to normalize UINT16 to UINT8
constexpr float kUint16ToUint8 = static_cast<float>(UINT8_MAX) / UINT16_MAX;
// Timeout for fetching new frames from the sensor
constexpr int kTimeoutMs = 5000;

using RealsenseOptionsMap = std::unordered_map<std::underlying_type<rs2_option>::type, float>;

// Deleters for various rs2 types
struct Rs2ContextDeleter {
  void operator()(rs2_context* b) { rs2_delete_context(b); }
};

struct Rs2DeviceDeleter {
  void operator()(rs2_device* b) { rs2_delete_device(b); }
};

struct Rs2DeviceListDeleter {
  void operator()(rs2_device_list* b) { rs2_delete_device_list(b); }
};

struct Rs2PipelineDeleter {
  void operator()(rs2_pipeline* b) { rs2_delete_pipeline(b); }
};

struct Rs2PipelineProfileDeleter {
  void operator()(rs2_pipeline_profile* b) { rs2_delete_pipeline_profile(b); }
};

struct Rs2SensorListDeleter {
  void operator()(rs2_sensor_list* b) { rs2_delete_sensor_list(b); }
};

struct Rs2StreamProfileListDeleter {
  void operator()(rs2_stream_profile_list* b) { rs2_delete_stream_profiles_list(b); }
};

struct Rs2SensorDeleter {
  void operator()(rs2_sensor* b) { rs2_delete_sensor(b); }
};

struct Rs2FrameDeleter {
  void operator()(rs2_frame* b) { rs2_release_frame(b); }
};

struct Rs2ConfigDeleter {
  void operator()(rs2_config* b) { rs2_delete_config(b); }
};

// Returns true if the error is fatal and false otherwise
bool IsFatal(const rs2_error* error) {
  switch (rs2_get_librealsense_exception_type(error)) {
    case RS2_EXCEPTION_TYPE_INVALID_VALUE:
    case RS2_EXCEPTION_TYPE_WRONG_API_CALL_SEQUENCE:
      return true;
    case RS2_EXCEPTION_TYPE_IO:
    case RS2_EXCEPTION_TYPE_CAMERA_DISCONNECTED:
    case RS2_EXCEPTION_TYPE_BACKEND:
    case RS2_EXCEPTION_TYPE_NOT_IMPLEMENTED:
    case RS2_EXCEPTION_TYPE_DEVICE_IN_RECOVERY_MODE:
    case RS2_EXCEPTION_TYPE_UNKNOWN:
    default:
      return false;
  }
}

// Logs the details from the rs2_error
inline void LogRs2Error(const rs2_error* err) {
  LOG_ERROR("Realsense Error: %s.", rs2_get_error_message(err));
  LOG_ERROR("Failed function %s, Args %s", rs2_get_failed_function(err), rs2_get_failed_args(err));
}

// Log if there is an error
template <typename... Args>
inline void LogIfError(const rs2_error* err, const char* s, Args... args) {
  if (err == nullptr) {
    return;
  }

  LogRs2Error(err);
  LOG_ERROR(s, args...);
}

// Log if there is an error and assert
template <typename... Args>
inline void LogIfErrorAndAssert(const rs2_error* err, Args... args) {
  if (err == nullptr) {
    return;
  }
  LogRs2Error(err);
  ASSERT(err == nullptr, args...);
}

// Log if there is an error and assert if the error is fatal
template <typename... Args>
inline void LogIfErrorAndAssertIfFatal(const rs2_error* err, Args... args) {
  if (err == nullptr) {
    return;
  }
  LogRs2Error(err);
  ASSERT(!IsFatal(err), args...);
}

// Enables advanced mode on the device. Advanced mode is necessary to access IR streams
void EnableAdvancedMode(rs2_device* device_handle) {
  ASSERT(device_handle != nullptr, "device cannot be null");
  rs2_error* err = nullptr;
  // Check if Advanced-Mode is enabled
  int is_advanced_mode_enabled = 0;
  rs2_is_enabled(device_handle, &is_advanced_mode_enabled, &err);
  LogIfErrorAndAssert(err, "Error checking advanced mode.");

  if (!is_advanced_mode_enabled) {
    // Enable advanced-mode
    rs2_toggle_advanced_mode(device_handle, 1, &err);
    LogIfErrorAndAssert(err, "Error enabling advanced mode.");
  }

  rs2_is_enabled(device_handle, &is_advanced_mode_enabled, &err);
  LogIfErrorAndAssert(err, "Error checking advanced mode.");
  ASSERT(is_advanced_mode_enabled != 0, "Advanced mode couldn't be enabled");
}

// Check if the setting/option is supported by the camera and if it is, update the local cache
// with the value
void ReadSensorOptions(const rs2_sensor* sensor, RealsenseOptionsMap& sensor_options) {
  ASSERT(sensor != nullptr, "sensor cannot be nullptr");
  const rs2_options* options = reinterpret_cast<const rs2_options*>(sensor);
  sensor_options.clear();
  rs2_error* err = nullptr;
  for (int op = 0; op < RS2_OPTION_COUNT; ++op) {
    rs2_option rs2_op = static_cast<rs2_option>(op);
    int supports = rs2_supports_option(options, rs2_op, &err);
    LogIfError(err, "Error checking if option %s is supported", rs2_option_to_string(rs2_op));
    if (err == nullptr && supports > 0) {
      float val = rs2_get_option(options, rs2_op, &err);
      LogIfError(err, "Error querying for option %s", rs2_option_to_string(rs2_op));
      sensor_options[rs2_op] = val;
    }
  }
}

// Checks if the given frame matches all the criteria from the required stream
bool IsFromMatchingStream(rs2_frame* frame, rs2_stream stream_type, int stream_index,
                          rs2_format stream_format, rs2_error*& err) {
  const rs2_stream_profile* profile = rs2_get_frame_stream_profile(frame, &err);
  if (err != nullptr) {
    return false;
  }

  rs2_stream st_type;
  rs2_format format;
  int index = 0;
  int unique_id = 0;
  int framerate = 0;
  rs2_get_stream_profile_data(profile, &st_type, &format, &index, &unique_id, &framerate, &err);
  if (err != nullptr) {
    return false;
  }

  bool is_video_frame = rs2_is_frame_extendable_to(frame, RS2_EXTENSION_VIDEO_FRAME, &err);
  if (err != nullptr) {
    return false;
  }
  ASSERT(is_video_frame, "Expected a video frame");
  return (stream_type == st_type && stream_index == index && stream_format == format);
}

// Helper function to publish a depth frame
void PublishDepthFrame(int64_t acqtime, Image1f& depth, const geometry::Pinhole<double>& pinhole,
                       isaac::alice::ProtoTx<DepthCameraProto>& proto) {
  auto proto_depth = proto.initProto();
  ToProto(pinhole, proto_depth.initPinhole());
  ToProto(std::move(depth), proto_depth.initDepthImage(), proto.buffers());
  proto_depth.setMinDepth(kMinDepthMeters);
  proto_depth.setMaxDepth(kMinDepthMeters);

  proto.publish(acqtime);
}

// Helper function to publish a video frame
template <ColorCameraProto::ColorSpace C, typename K, int N>
void PublishVideoFrame(int64_t acqtime, Image<K, N>& image,
                       const geometry::Pinhole<double>& pinhole,
                       const Vector5f& distortion_coefficients,
                       isaac::alice::ProtoTx<ColorCameraProto>& proto) {
  auto proto_color = proto.initProto();
  proto_color.setColorSpace(C);
  ToProto(pinhole, proto_color.initPinhole());

  auto distortion = proto_color.initDistortion();
  distortion.setModel(DistortionProto::DistortionModel::BROWN);
  isaac::ToProto(distortion_coefficients, distortion.getCoefficients());

  ToProto(std::move(image), proto_color.getImage(), proto.buffers());
  proto.publish(acqtime);
}
// converts rs2 intrinsics to isaac intrinsics
void ToIsaacIntrinsics(const rs2_intrinsics& rs2_ints, geometry::Pinhole<double>& isaac_intrinsics,
                       Vector5f& distortion) {
  isaac_intrinsics.rows = rs2_ints.height;
  isaac_intrinsics.cols = rs2_ints.width;
  isaac_intrinsics.focal = Vector2d(rs2_ints.fy, rs2_ints.fx);
  isaac_intrinsics.center = Vector2d(rs2_ints.ppy, rs2_ints.ppx);
  ASSERT(rs2_ints.model == RS2_DISTORTION_BROWN_CONRADY, "Unsupported distortion model");
  distortion << rs2_ints.coeffs[0], rs2_ints.coeffs[1], rs2_ints.coeffs[2], rs2_ints.coeffs[3],
      rs2_ints.coeffs[4];
}

// converts rs2 video frame to Isaac image
// S = typeof the source data in rs2_frame
// K = typeof the destination in the published image
// N = Number of channels
template <typename S, typename K, int N, typename F>
void ToIsaacImage(const rs2_frame* frame, F convert, Image<K, N>& out, rs2_error*& err) {
  ASSERT(frame != nullptr, "Frame cannot be null");
  const void* frame_data = rs2_get_frame_data(frame, &err);
  if (err != nullptr) {
    return;
  }

  int stride = rs2_get_frame_stride_in_bytes(frame, &err);
  if (err != nullptr) {
    return;
  }

  int width = rs2_get_frame_width(frame, &err);
  if (err != nullptr) {
    return;
  }

  int height = rs2_get_frame_height(frame, &err);
  if (err != nullptr) {
    return;
  }

  CpuAlignedBufferConstView mem_view(static_cast<const S*>(frame_data), static_cast<size_t>(height),
                                     static_cast<size_t>(stride));
  ImageConstView<S, N> img_view(mem_view, height, width);
  Convert(img_view, out, convert);
}

// Helper to convert rs2_extrinsics to Pose3d
Pose3d ToPose3d(const rs2_extrinsics& extrinsics) {
  Matrix3f rotation(extrinsics.rotation);
  Quaternion<float> quaternion(rotation);
  Pose3d pose;
  pose.rotation = SO3<double>::FromQuaternion(quaternion.cast<double>());
  const float* rs2_translation = extrinsics.translation;
  pose.translation[0] = static_cast<double>(rs2_translation[0]);
  pose.translation[1] = static_cast<double>(rs2_translation[1]);
  pose.translation[2] = static_cast<double>(rs2_translation[2]);
  return pose;
}

// Realsense hub class. This is a helper class that is responsible for device enumeration and
// pipeline creation, while hiding the device context from the callers
class RealsenseCameraHub {
 public:
  // create a pipeline using the shared device context
  rs2_pipeline* createPipe() {
    rs2_error* err = nullptr;
    rs2_pipeline* pipe = rs2_create_pipeline(realsense_ctx_.get(), &err);
    LogIfErrorAndAssertIfFatal(err, "Error creating rs2_pipeline");
    return pipe;
  }

  // Get the serial number of a device
  std::string getSerial(int index) {
    enumerateConnectedDevices();

    rs2_error* err = nullptr;

    ASSERT(index >= 0, "Index cannot be negative %d", index);
    ASSERT(static_cast<size_t>(index) < devices_.size(),
           "Invalid device index. known devices %d, device index %d", devices_.size(), index);

    rs2_device* dev = devices_[index].get();

    std::string serial_number(rs2_get_device_info(dev, RS2_CAMERA_INFO_SERIAL_NUMBER, &err));
    LogIfErrorAndAssertIfFatal(err, "Error querying for serial number");

    std::string camera_name(rs2_get_device_info(dev, RS2_CAMERA_INFO_NAME, &err));
    LogIfError(err, "Error getting camera name");

    std::string firmware_ver(rs2_get_device_info(dev, RS2_CAMERA_INFO_FIRMWARE_VERSION, &err));
    LogIfError(err, "Error getting camera firmware version");

    LOG_DEBUG("Camera Name : %s", camera_name.c_str());
    LOG_DEBUG("Camera Serial : %s", serial_number.c_str());
    LOG_DEBUG("Camera Firmware : %s", firmware_ver.c_str());

    return serial_number;
  }

 private:
  // enumerates the connected devices. If called from multiple threads/codelets, when multiple
  // realsense cameras are connected, we ensure that the devices get enumerated only once
  void enumerateConnectedDevices() {
    std::call_once(flag_,
                   [](std::unique_ptr<rs2_context, Rs2ContextDeleter>& ctx,
                      std::vector<std::unique_ptr<rs2_device, Rs2DeviceDeleter>>& devices) {
                     rs2_error* err = 0;
                     ctx.reset(rs2_create_context(RS2_API_VERSION, &err));
                     LogIfErrorAndAssertIfFatal(err, "Error creating context");

                     std::unique_ptr<rs2_device_list, Rs2DeviceListDeleter> device_list(
                         rs2_query_devices(ctx.get(), &err));
                     LogIfErrorAndAssertIfFatal(err, "Error querying for devices");

                     int device_count = rs2_get_device_count(device_list.get(), &err);
                     LogIfErrorAndAssertIfFatal(err, "Error getting device count");
                     devices.reserve(device_count);

                     for (int i = 0; i < device_count; ++i) {
                       std::unique_ptr<rs2_device, Rs2DeviceDeleter> dev(
                           rs2_create_device(device_list.get(), i, &err));
                       LogIfErrorAndAssertIfFatal(err, "Error creating realsense device");

                       devices.push_back(std::move(dev));
                     }
                   },
                   std::ref(realsense_ctx_), std::ref(devices_));
  }

  std::once_flag flag_;
  std::unique_ptr<rs2_context, Rs2ContextDeleter> realsense_ctx_;
  std::vector<std::unique_ptr<rs2_device, Rs2DeviceDeleter>> devices_;
};

}  // namespace

// We are using pimpl idiom here. The actual implementation of the camera class is inside this class
class RealsenseCamera::RealsenseCameraImpl {
 public:
  RealsenseCameraImpl() = default;

  ~RealsenseCameraImpl() {
    if (pipe_started_) {
      rs2_error* err = nullptr;
      rs2_pipeline_stop(pipe_.get(), &err);
      LogIfError(err, "Error stopping the pipe");
    }
    pipe_ = nullptr;
  }

  // Non copyable Non movable
  RealsenseCameraImpl(RealsenseCameraImpl&&) = delete;
  RealsenseCameraImpl& operator=(RealsenseCameraImpl&&) = delete;

  // Start the sensor using the config
  void start(rs2_config* config) {
    rs2_error* err = nullptr;
    pipe_.reset(Singleton<RealsenseCameraHub>::Get().createPipe());
    // Start streaming
    pipeline_profile_.reset(rs2_pipeline_start_with_config(pipe_.get(), config, &err));
    LogIfErrorAndAssert(err, "Error starting the pipe");
    pipe_started_ = true;

    std::unique_ptr<rs2_device, Rs2DeviceDeleter> device_handle(
        rs2_pipeline_profile_get_device(pipeline_profile_.get(), &err));
    // Turn on advanced mode as IR streams are available only in advanced mode.
    EnableAdvancedMode(device_handle.get());

    saveSensorHandles(device_handle.get());
    ir_left_stream_profile_ = getStreamProfile(depth_sensor_.get(), RS2_STREAM_INFRARED, 1);
    ASSERT(ir_left_stream_profile_ != nullptr, "ir left stream profile cannot be null");
    ir_right_stream_profile_ = getStreamProfile(depth_sensor_.get(), RS2_STREAM_INFRARED, 2);
    ASSERT(ir_right_stream_profile_ != nullptr, "ir right stream profile cannot be null");
    color_stream_profile_ = getStreamProfile(color_sensor_.get(), RS2_STREAM_COLOR, 0);
    ASSERT(color_stream_profile_ != nullptr, "color stream profile cannot be null");

    // In order to speed up the rs2_option to string conversion every tick we create a lookup
    // table. The lookup table associates each rs2_option with its string name. The
    // rs2_option_to_string function returns string in this form "Option Name". Our corresponding
    // config param is named as "color_option_name" or "depth_option_name" To simplify and speed
    // up the lookup we convert the string "Option Name" to "option_name" and associate that with
    // the rs2_option
    rs2_option_to_codelet_config_.clear();
    for (int op = 0; op < RS2_OPTION_COUNT; ++op) {
      rs2_option rs2_op = static_cast<rs2_option>(op);
      std::string option_name(rs2_option_to_string(rs2_op));
      std::replace(option_name.begin(), option_name.end(), ' ', '_');
      std::transform(option_name.begin(), option_name.end(), option_name.begin(), ::tolower);
      rs2_option_to_codelet_config_[rs2_op] = option_name;
    }
  }

  // Fetch next set of frames
  void fetchNewFrames(rs2_error*& err) {
    frameset_.reset(rs2_pipeline_wait_for_frames(pipe_.get(), kTimeoutMs, &err));
    LogIfError(err, "Error while getting new frames");
  }

  // Helper to get new gray color frame from the frame set
  bool getNewColorGrayFrame(Image1ub& gray, geometry::Pinhole<double>& pinhole,
                            Vector5f& distortion, rs2_error*& err) {
    return getNewFrame<uint16_t>(RS2_STREAM_COLOR, 0, RS2_FORMAT_Y16,
                                 [](PixelConstRef1ui16 p) { return p * kUint16ToUint8; }, gray,
                                 pinhole, distortion, last_color_gray_frame_number_, err);
  }

  // Helper to get new Color RGB frame from the frame set
  bool getNewColorRGBFrame(Image3ub& color, geometry::Pinhole<double>& pinhole,
                           Vector5f& distortion, rs2_error*& err) {
    return getNewFrame<uint8_t>(RS2_STREAM_COLOR, 0, RS2_FORMAT_RGB8,
                                [](PixelConstRef3ub p) { return p; }, color, pinhole, distortion,
                                last_color_rgb_frame_number_, err);
  }

  // Helper to get new depth frame from the frame set
  bool getNewDepthFrame(Image1f& depth, geometry::Pinhole<double>& pinhole, Vector5f& distortion,
                        rs2_error*& err) {
    return getNewFrame<uint16_t>(
        RS2_STREAM_DEPTH, 0, RS2_FORMAT_Z16,
        [scale = this->depth_scale_](PixelConstRef1ui16 p) { return p * scale; }, depth, pinhole,
        distortion, last_depth_frame_number_, err);
  }

  // Helper to get new ir left frame from the frame set
  bool getNewIrLeftFrame(Image1ub& ir, geometry::Pinhole<double>& pinhole, Vector5f& distortion,
                         rs2_error*& err) {
    return getNewFrame<uint8_t>(RS2_STREAM_INFRARED, 1, RS2_FORMAT_Y8,
                                [](PixelConstRef1ub p) { return p; }, ir, pinhole, distortion,
                                last_ir_left_frame_number_, err);
  }

  // Helper to get new ir right frame from the frame set
  bool getNewIrRightFrame(Image1ub& ir, geometry::Pinhole<double>& pinhole, Vector5f& distortion,
                          rs2_error*& err) {
    return getNewFrame<uint8_t>(RS2_STREAM_INFRARED, 2, RS2_FORMAT_Y8,
                                [](PixelConstRef1ub p) { return p; }, ir, pinhole, distortion,
                                last_ir_right_frame_number_, err);
  }

  // Getter for rs2_option_to_codelet_config
  const std::unordered_map<std::underlying_type<rs2_option>::type, std::string>&
  getRs2OptionToCodeletConfig() {
    return rs2_option_to_codelet_config_;
  }

  // Helper to update color and depth sensor options from config
  void updateSensorOptions(const RealsenseOptionsMap& color_sensor_options_from_config,
                           const RealsenseOptionsMap& depth_sensor_options_from_config) {
    updateSensorOptions(color_sensor_.get(), color_sensor_options_from_config,
                        color_sensor_options_);
    updateSensorOptions(depth_sensor_.get(), depth_sensor_options_from_config,
                        depth_sensor_options_);
  }

  // Get extrinsics for the color and right ir sensor
  void getExtrinsics(Pose3d& ir_left_T_color, Pose3d& ir_left_T_ir_right, rs2_error*& err) {
    rs2_extrinsics ext;
    rs2_get_extrinsics(color_stream_profile_, ir_left_stream_profile_, &ext, &err);
    if (err != nullptr) {
      return;
    }
    ir_left_T_color = ToPose3d(ext);

    rs2_get_extrinsics(ir_right_stream_profile_, ir_left_stream_profile_, &ext, &err);
    if (err != nullptr) {
      return;
    }
    ir_left_T_ir_right = ToPose3d(ext);
  }

 private:
  // Compare the config value of the option with the cached value and update the option value if
  // necessary
  void updateSensorOptions(const rs2_sensor* sensor,
                           const RealsenseOptionsMap& sensor_options_from_config,
                           RealsenseOptionsMap& sensor_options) {
    rs2_error* err = nullptr;
    const rs2_options* options = reinterpret_cast<const rs2_options*>(sensor);
    for (auto& pair : sensor_options) {
      const auto& itr_config = sensor_options_from_config.find(pair.first);
      if (itr_config != sensor_options_from_config.end() && pair.second != itr_config->second) {
        rs2_option rs2_op = static_cast<rs2_option>(pair.first);
        LOG_DEBUG("Setting Realsense option %s to %f",
            rs2_option_to_string(rs2_op), itr_config->second);
        rs2_set_option(options, rs2_op, itr_config->second, &err);
        LogIfError(err, "Failed to set sensor option");
        pair.second = rs2_get_option(options, rs2_op, &err);
        LogIfError(err, "Failed to get sensor option");

        ASSERT(pair.second == itr_config->second, "Failed to set the sensor option %s : %d",
               rs2_option_to_codelet_config_.find(rs2_op)->first, itr_config->second);
      }
    }
    rs2_free_error(err);
  }

  // Helper function to check if there is a new frame on a stream and extract image and intrinsics
  template <typename S, typename K, int N, typename F>
  bool getNewFrame(rs2_stream stream_type, int stream_index, rs2_format stream_format, F convert,
                   Image<K, N>& color, geometry::Pinhole<double>& pinhole, Vector5f& distortion,
                   uint64_t& last_frame_number, rs2_error*& err) {
    bool is_new = false;
    int num_of_frames = rs2_embedded_frames_count(frameset_.get(), &err);
    if (err != nullptr) {
      return is_new;
    }

    for (int f = 0; f < num_of_frames; ++f) {
      std::unique_ptr<rs2_frame, Rs2FrameDeleter> frame(
          rs2_extract_frame(frameset_.get(), f, &err));
      if (err != nullptr) {
        return is_new;
      }

      if (IsFromMatchingStream(frame.get(), stream_type, stream_index, stream_format, err)) {
        uint64_t frame_number = rs2_get_frame_number(frame.get(), &err);
        if (err != nullptr) {
          return is_new;
        }

        is_new = (last_frame_number < frame_number);
        if (is_new) {
          last_frame_number = frame_number;

          ToIsaacImage<S>(frame.get(), convert, color, err);
          if (err != nullptr) {
            return is_new;
          }

          const rs2_stream_profile* profile = rs2_get_frame_stream_profile(frame.get(), &err);
          if (err != nullptr) {
            return is_new;
          }

          rs2_intrinsics intrinisics;
          rs2_get_video_stream_intrinsics(profile, &intrinisics, &err);
          if (err != nullptr) {
            return is_new;
          }

          ToIsaacIntrinsics(intrinisics, pinhole, distortion);
        }
      }
    }
    return is_new;
  }

  // Helper function to cache some sensor handles
  void saveSensorHandles(const rs2_device* device_handle) {
    ASSERT(device_handle != nullptr, "device cannot be null");
    // Get depth and color sensor handles
    rs2_error* err = nullptr;
    std::unique_ptr<rs2_sensor_list, Rs2SensorListDeleter> sensors(
        rs2_query_sensors(device_handle, &err));
    LogIfErrorAndAssertIfFatal(err, "Failed querying sensors");

    int num_sensors = rs2_get_sensors_count(sensors.get(), &err);
    LogIfErrorAndAssertIfFatal(err, "Failed querying sensor count");
    ASSERT(num_sensors == 2, "Expected number of sensors is 2. Found %d", num_sensors);

    for (int s = 0; s < num_sensors; ++s) {
      std::unique_ptr<rs2_sensor, Rs2SensorDeleter> sensor(
          rs2_create_sensor(sensors.get(), s, &err));
      LogIfErrorAndAssertIfFatal(err, "Failed creating sensor");
      ASSERT(sensor != nullptr, "Failed to get sensor from sensor list");
      int is_depth_sensor = 0;
      // Check if the given sensor can be extended to depth sensor interface
      is_depth_sensor = rs2_is_sensor_extendable_to(sensor.get(), RS2_EXTENSION_DEPTH_SENSOR, &err);
      LogIfErrorAndAssertIfFatal(err, "Failed querying for depth sensor");
      if (is_depth_sensor == 1) {
        depth_sensor_ = std::move(sensor);
        depth_scale_ = rs2_get_option(reinterpret_cast<const rs2_options*>(depth_sensor_.get()),
                                      RS2_OPTION_DEPTH_UNITS, &err);
        LogIfErrorAndAssertIfFatal(err, "Error getting depth scale from the sensor");
        ReadSensorOptions(depth_sensor_.get(), depth_sensor_options_);
      } else {
        color_sensor_ = std::move(sensor);
        ReadSensorOptions(color_sensor_.get(), color_sensor_options_);
      }
    }
  }

  // Helper function to cache some stream handles
  const rs2_stream_profile* getStreamProfile(rs2_sensor* sensor, rs2_stream stream, int index) {
    rs2_error* err = nullptr;
    std::unique_ptr<rs2_stream_profile_list, Rs2StreamProfileListDeleter>
        depth_sensor_stream_profiles(rs2_get_stream_profiles(sensor, &err));
    LogIfErrorAndAssertIfFatal(err, "Error querying sensor profiles");

    int stream_profiles_count =
        rs2_get_stream_profiles_count(depth_sensor_stream_profiles.get(), &err);
    LogIfErrorAndAssertIfFatal(err, "Error querying sensor profiles count");

    for (int prof = 0; prof < stream_profiles_count; ++prof) {
      const rs2_stream_profile* stream_profile =
          rs2_get_stream_profile(depth_sensor_stream_profiles.get(), prof, &err);
      LogIfErrorAndAssertIfFatal(err, "Error querying sensor profile ");

      rs2_stream st;
      rs2_format format;
      int idx = 0;
      int unique_id = 0;
      int framerate = 0;
      rs2_get_stream_profile_data(stream_profile, &st, &format, &idx, &unique_id, &framerate, &err);
      LogIfErrorAndAssertIfFatal(err, "Error querying sensor profile data");
      if (st == stream && idx == index) {
        return stream_profile;
      }
    }
    return nullptr;
  }

  std::unique_ptr<rs2_frame, Rs2FrameDeleter> frameset_;
  RealsenseOptionsMap color_sensor_options_;
  RealsenseOptionsMap depth_sensor_options_;
  std::unique_ptr<rs2_pipeline, Rs2PipelineDeleter> pipe_;
  std::unique_ptr<rs2_pipeline_profile, Rs2PipelineProfileDeleter> pipeline_profile_;
  std::unique_ptr<rs2_sensor, Rs2SensorDeleter> depth_sensor_;
  std::unique_ptr<rs2_sensor, Rs2SensorDeleter> color_sensor_;
  const rs2_stream_profile* ir_left_stream_profile_ = nullptr;
  const rs2_stream_profile* ir_right_stream_profile_ = nullptr;
  const rs2_stream_profile* color_stream_profile_ = nullptr;
  float depth_scale_ = 0.0f;
  bool pipe_started_ = false;
  uint64_t last_color_rgb_frame_number_ = 0;
  uint64_t last_color_gray_frame_number_ = 0;
  uint64_t last_depth_frame_number_ = 0;
  uint64_t last_ir_left_frame_number_ = 0;
  uint64_t last_ir_right_frame_number_ = 0;
  std::unordered_map<std::underlying_type<rs2_option>::type, std::string>
      rs2_option_to_codelet_config_;
};

// These need to be defined here for pimpl patttern to work
RealsenseCamera::RealsenseCamera() = default;
RealsenseCamera::~RealsenseCamera() = default;


#if 1 // added by qiaoyx
void RealsenseCamera::depthimageToLaserscan (
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

void RealsenseCamera::start() {
  impl_ = std::make_unique<RealsenseCameraImpl>();

  std::string device_serial = Singleton<RealsenseCameraHub>::Get().getSerial(get_index());
  rs2_error* err = nullptr;
  std::unique_ptr<rs2_config, Rs2ConfigDeleter> cfg(rs2_create_config(&err));
  LogIfErrorAndAssertIfFatal(err, "Error creating rs2_config");

  rs2_config_enable_device(cfg.get(), device_serial.c_str(), &err);
  LogIfErrorAndAssertIfFatal(err, "Error enabling device in config");

  Vector2i color_size = get_color_size();
  if (get_color_mode() == kRgb) {  // RGB
    rs2_config_enable_stream(cfg.get(), RS2_STREAM_COLOR, 0, color_size[1], color_size[0],
                             RS2_FORMAT_RGB8, get_color_fps(), &err);
    LogIfErrorAndAssertIfFatal(err, "Error enabling rgb color stream in config");
  } else if (get_color_mode() == kGrayscale) {  // Grayscale
    rs2_config_enable_stream(cfg.get(), RS2_STREAM_COLOR, 0, color_size[1], color_size[0],
                             RS2_FORMAT_Y16, get_color_fps(), &err);
    LogIfErrorAndAssertIfFatal(err, "Error enabling rgb grayscale stream in config");
  }

  Vector2i depth_size = get_depth_size();
  if (get_enable_depth()) {
    rs2_config_enable_stream(cfg.get(), RS2_STREAM_DEPTH, 0, depth_size[1], depth_size[0],
                             RS2_FORMAT_Z16, get_depth_fps(), &err);
    LogIfErrorAndAssertIfFatal(err, "Error enabling depth stream in config");
  }

  if (get_enable_ir_left()) {
    rs2_config_enable_stream(cfg.get(), RS2_STREAM_INFRARED, 1, depth_size[1], depth_size[0],
                             RS2_FORMAT_Y8, get_depth_fps(), &err);
    LogIfErrorAndAssertIfFatal(err, "Error enabling left ir stream in config");
  }

  if (get_enable_ir_right()) {
    rs2_config_enable_stream(cfg.get(), RS2_STREAM_INFRARED, 2, depth_size[1], depth_size[0],
                             RS2_FORMAT_Y8, get_depth_fps(), &err);
    LogIfErrorAndAssertIfFatal(err, "Error enabling right ir stream in config");
  }

  impl_->start(cfg.get());
  tickBlocking();
}

void RealsenseCamera::tick() {
  // Check if the config was modified for the device in sight and update the settings
  // accordingly
  updateSensorOptions();

  int64_t acqtime = node()->clock()->timestamp();

  Image3ub color_rgb;
  rs2_error* err = nullptr;
  bool is_new = false;
  geometry::Pinhole<double> pinhole;
  Vector5f distortion_coefficients;

  impl_->fetchNewFrames(err);
  if (err != nullptr) {
    stop();
  }

  is_new = impl_->getNewColorRGBFrame(color_rgb, pinhole, distortion_coefficients, err);
  LogIfError(err, "Error querying for Color RGB frame");
  if (err != nullptr) {
    stop();
  }
  if (is_new) {
    PublishVideoFrame<ColorCameraProto::ColorSpace::RGB>(acqtime, color_rgb, pinhole,
                                                         distortion_coefficients, tx_color());
  }

  Image1ub color_gray;
  is_new = impl_->getNewColorGrayFrame(color_gray, pinhole, distortion_coefficients, err);
  LogIfError(err, "Error querying for Color Gray frame");
  if (err != nullptr) {
    stop();
  }
  if (is_new) {
    PublishVideoFrame<ColorCameraProto::ColorSpace::GREYSCALE>(acqtime, color_gray, pinhole,
                                                               distortion_coefficients, tx_color());
  }

  Image1ub ir_left;
  is_new = impl_->getNewIrLeftFrame(ir_left, pinhole, distortion_coefficients, err);
  LogIfError(err, "Error querying for IR left frame");
  if (err != nullptr) {
    stop();
  }
  if (is_new) {
    PublishVideoFrame<ColorCameraProto::ColorSpace::GREYSCALE>(
        acqtime, ir_left, pinhole, distortion_coefficients, tx_ir_left());
  }

  Image1ub ir_right;
  is_new = impl_->getNewIrRightFrame(ir_right, pinhole, distortion_coefficients, err);
  LogIfError(err, "Error querying for IR right frame");
  if (err != nullptr) {
    stop();
  }
  if (is_new) {
    PublishVideoFrame<ColorCameraProto::ColorSpace::GREYSCALE>(
        acqtime, ir_right, pinhole, distortion_coefficients, tx_ir_right());
  }

  Image1f depth;
  is_new = impl_->getNewDepthFrame(depth, pinhole, distortion_coefficients, err);
  LogIfError(err, "Error querying for depth frame");
  if (err != nullptr) {
    stop();
  }

#if 1 // added by qiaoyx
  if (get_depthimage_to_laserscan()) {
    depthimageToLaserscan(acqtime, depth, pinhole);
  }
#endif // the end

  if (is_new) {
    PublishDepthFrame(acqtime, depth, pinhole, tx_depth());
  }

  Pose3d ir_left_T_color, ir_left_T_ir_right;
  impl_->getExtrinsics(ir_left_T_color, ir_left_T_ir_right, err);
  LogIfError(err, "Error querying for extrinsics");
  if (err != nullptr) {
    stop();
  }
  set_ir_left_T_color(ir_left_T_color, acqtime);
  set_ir_left_T_ir_right(ir_left_T_ir_right, acqtime);
}

void RealsenseCamera::stop() {
  impl_ = nullptr;
}

void RealsenseCamera::updateSensorOptions() {
  const std::array<std::string, 2> sensor_strs{"color_", "depth_"};
  std::array<RealsenseOptionsMap, 2> sensor_options_from_config;
  for (size_t s = 0; s < sensor_strs.size(); ++s) {
    for (int i = 0; i < RS2_OPTION_COUNT; ++i) {
      std::string config_string(sensor_strs[s] +
                                impl_->getRs2OptionToCodeletConfig().find(i)->second);
      std::optional<int> option = node()->config().tryGet<int>(this, config_string);
      if (option) {
        sensor_options_from_config[s][static_cast<rs2_option>(i)] = *option;
      }
    }
  }
  impl_->updateSensorOptions(sensor_options_from_config[0], sensor_options_from_config[1]);
}

}  // namespace isaac
