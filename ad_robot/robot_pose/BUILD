'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
load("//engine/build:isaac.bzl", "isaac_cc_module") 

isaac_cc_module(
    name = "robot_pose",
    visibility = ["//visibility:public"],
    srcs = ["robot_pose.cpp"],
    hdrs = ["robot_pose.hpp"],
    deps = [
        "//engine/alice",
        "//messages",
        "//engine/gems/algorithm:string_utils",
        "//engine/gems/state_machine",
        "//engine/gems/state:io",
        "//engine/core/math",
        "//engine/gems/coms:serial",
        "//engine/gems/coms:i2c",
        "@gtest//:main",
        "//third_party:ros",
    ],
)
