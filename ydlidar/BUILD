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
    name = "isaac_ydlidar",
    # linkopts = [
    #     "-ldl",
    #     "-lgomp",
    #     "-lrt",
    # ],
    deps = [
        "//engine/gems/geometry",
        "//engine/gems/system:cuda_context",
        "@ydlidar",
    ],
)

isaac_cc_module(
    name = "ydlidar",
    deps = [":isaac_ydlidar"],
)


# filegroup(
#     name = "libydlidar_driver.so",
#     tags = ["manual"],
#     data = select({
#         "//engine/build:platform_x86_64": ["//packages_x86_64/ydlidar:libydlidar_driver.so_release"],
#     }),
#     visibility = ["//visibility:public"],
# )