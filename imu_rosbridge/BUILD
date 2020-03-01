load("//engine/build:isaac.bzl", "isaac_cc_module")


isaac_cc_module(
    name = "imu_ros_bridge",
    srcs = [
            "ImuRosBridge.cpp",
    ],
    hdrs =
        ["ImuRosBridge.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/alice",
        "//engine/core",
        "//engine/core/image",
        "//engine/core/math",
        "//engine/core/tensor",
        "//engine/gems/sight",
        "//engine/gems/geometry",
        "//engine/gems/system:cuda_context",
        "//messages",
        "//third_party:ros",
    ],
)
