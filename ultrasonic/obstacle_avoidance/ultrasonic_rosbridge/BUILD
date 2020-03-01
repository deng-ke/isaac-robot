load("//engine/build:isaac.bzl", "isaac_cc_module")


isaac_cc_module(
    name = "ultrasonic_ros_bridge",
    srcs = [
            "UltrasonicRosBridge.cpp",
    ],
    hdrs =
        ["UltrasonicRosBridge.hpp"],
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
