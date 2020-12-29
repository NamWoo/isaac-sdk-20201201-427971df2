#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xcec96e7061c21f3e;

# Message containing input to the audio playback module
struct AudioDataProto {
    # Audio Sample Rate (eg. 44100, 48000, 96000, 192000)
    sampleRate @0 : UInt32;
    # Number of channels in the Audio packet
    numChannels @1 : UInt8;
}
