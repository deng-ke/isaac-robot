load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component")

# isaac_component(
#     name = "hokuyo_rosbridge",
#     # linkopts = [
#     #     "-ldl",
#     #     "-lgomp",
#     #     "-lrt",
#     # ],
#     srcs = [
#             "HokuyoRosBridge.cpp", "HokuyoRosBridge.hpp",
#             ],
#     deps = [
#         "//engine/gems/geometry",
#         "//engine/gems/system:cuda_context",
#         "//third_party:ros",
#     ],
#     visibility = ["//visibility:public"],
# )

# isaac_cc_module(
#     name = "hokuyo",
#     deps = [":hokuyo_rosbridge"],
# )

isaac_cc_module(
    name = "hokuyo_ros_bridge",
    srcs = [
            "HokuyoRosBridge.cpp",
    ],
    hdrs =
        ["HokuyoRosBridge.hpp"],
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
        "//messages",
        "//third_party:ros",
    ],
)
