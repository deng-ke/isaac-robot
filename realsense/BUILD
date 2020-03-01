'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component")

isaac_component(
    name = "realsense_camera",
    deps = [
        "//engine/gems/geometry",
        "@realsense",
    ],
)

isaac_component(
    name = "realsense_camera_simple",
    deps = [
        "//engine/gems/geometry",
        "@realsense",
    ],
)

isaac_cc_module(
    name = "realsense",
    deps = [
        "realsense_camera",
        "realsense_camera_simple",
    ],
)
