load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "odom_ros_bridge",
    srcs = [
            "OdomRosBridge.cpp",
    ],
    hdrs =
        ["OdomRosBridge.hpp"],
    # tags = [
    #     "manual",
    # ],
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
        "//engine/gems/state:io",
        "//messages/state:differential_base",
        "//messages",
        "//third_party:ros",
    ],
)
