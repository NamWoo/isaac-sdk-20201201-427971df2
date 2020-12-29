#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0x906fdaceb9c949b4;

# A chat message
struct ChatMessageProto {
  # The user name of the sender
  user @0: Text;
  # A channel on which the message was posted
  channel @1: Text;
  # The text sent by the user
  text @2: Text;
}
