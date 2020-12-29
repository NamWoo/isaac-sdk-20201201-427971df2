#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x9896da5d9bb812a8;

# Message containing detected command id and list of timestamps of contributing keywords
struct VoiceCommandDetectionProto {
    # Detected command id
    commandId @0 : UInt8;
    # List of timestamps of contributing keywords to the detected command
    timestamps @1: List(UInt64);
}
