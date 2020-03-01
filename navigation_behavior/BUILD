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
    name = "navigation_behavior",
    visibility = ["//visibility:public"],
    srcs = ["navigation_behavior.cpp"],
    hdrs = ["navigation_behavior.hpp"],
    deps = [
        "//engine/alice",
        "//messages",
        "//engine/gems/algorithm:string_utils",
        "//engine/gems/state_machine",
        "//engine/gems/state:io",
        "//engine/core/math",
        "//messages/state:differential_base",
    ],
)
