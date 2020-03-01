"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module")

isaac_cc_module(
    name = "OdomImu2Ros",
    srcs = [
        "OdomImu2Ros.cpp",
    ],
    hdrs =
        ["OdomImu2Ros.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice",
        "//engine/core",
        "//engine/core/image",
        "//engine/core/math",
        "//engine/core/tensor",
        "//engine/gems/sight",
        "//messages",
        "//third_party:ros",
    ],
)
