"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""
load("@com_nvidia_isaac_engine//engine/build:cc_capnp_library.bzl", "cc_capnp_library")

# A list of all capnp files and their capnp dependencies
_protos = [
    ["actor_group",             ["math"]],
    ["alice",                   ["math", "uuid"]],
    ["audio_data",              []],
    ["audio_file_playback",     []],
    ["basic",                   []],
    ["camera",                  ["image", "math"]],
    ["chat_message",            []],
    ["collision",               ["math"]],
    ["composite",               ["element_type", "math", "tensor"]],
    ["detections",              ["geometry", "graph", "math", "tensor"]],
    ["dynamixel_motors",        []],
    ["element_type",            []],
    ["fiducial_list",           ["math"]],
    ["flatscan",                []],
    ["geometry",                ["math"]],
    ["graph",                   ["math"]],
    ["heatmap",                 ["image", "math"]],
    ["image",                   ["element_type"]],
    ["imu",                     []],
    ["joystick_state",          ["math"]],
    ["json",                    []],
    ["label",                   []],
    ["led_strip",               ["math"]],
    ["map",                     ["math"]],
    ["marker_list",             ["math"]],
    ["math",                    []],
    ["mission",                 ["uuid", "json"]],
    ["differential_base",       ["math"]],
    ["obstacles",               ["image", "math", "uuid"]],
    ["trajectory",              ["math"]],
    ["parking",                 ["geometry"]],
    ["ping",                    []],
    ["point_cloud",             ["tensor"]],
    ["pose_tree",               ["math"]],
    ["pwm_channel_set",         []],
    ["range_scan",              ["tensor"]],
    ["rigid_body_3_group",      ["math"]],
    ["robot_state",             ["math"]],
    ["segmentation_prediction", ["tensor"]],
    ["state",                   ["tensor"]],
    ["superpixels",             ["image", "math"]],
    ["tensor",                  ["element_type"]],
    ["uuid",                    []],
    ["voice_command_detection", []],
]

def _proto_library_name(x):
    ''' The library name for a proto '''
    return x + "_proto"

def create_message_proto_libraries():
    '''
    Creates one C++ library for every capnp file which is part of the message API
    '''
    for entry in _protos:
        cc_capnp_library(
            name = _proto_library_name(entry[0]),
            protos = [entry[0] + ".capnp"],
            deps = [_proto_library_name(x) for x in entry[1]])

def message_proto_library_names():
    '''
    Returns a list with all names of C++ library created by `create_message_proto_libraries`
    '''
    return [_proto_library_name(x[0]) for x in _protos]
