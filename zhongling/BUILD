'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "zhongling",
    visibility = ["//visibility:public"],
    deps = [
        ":zhongling_rmp_driver",
    ],
)

isaac_component(
    name = "zhongling_rmp_driver",
    deps = [
        "//engine/core/math",
        "//engine/core/tensor",
        "//engine/gems/sight",
        "//engine/gems/geometry",
        "//engine/gems/system:cuda_context",
        "//engine/gems/state:io",
        "//messages/state:differential_base",
        "//engine/gems/state_machine",
    ],
)


# isaac_cc_module(
#     name = "zhongling",
#     srcs = ["ZhonglingRmpDriver.cpp",],
#     hdrs = ["ZhonglingRmpDriver.hpp",],
#     visibility = ["//visibility:public"],
#     deps = [
#         "//engine/alice",
#         "//engine/core",
#         "//engine/core/math",
#         "//engine/core/tensor",
#         "//engine/gems/sight",
#         "//engine/gems/geometry",
#         "//engine/gems/system:cuda_context",
#         "//engine/gems/state:io",
#         "//messages/state:differential_base",
#         "//messages",
#     ],
# )
