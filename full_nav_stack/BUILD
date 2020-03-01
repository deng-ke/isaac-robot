'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_app", "isaac_component", "isaac_pkg")

isaac_app(
    name = "qiaoyx",
    app_json_file = "follow.app.json",
    modules = [
        "hokuyo_rosbridge:hokuyo_ros_bridge",
        "map",
        "imu",
        "navigation",
        "planner",
        "perception:april_tags",
        "perception",
        "viewers",
        "zhongling",
        "sensors:joystick",
        "realsense",
        "stereo_depth:coarse_to_fine",
	"navigation_behavior",
    ]
)
