#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xe0663498ed64eae4;

using import "math.capnp".Pose3dProto;
using import "uuid.capnp".UuidProto;

# A message header which is sent together with the actual message.
# It allows the remote to forward the message to the correct receiver and it also provides timing
# information based on the transmitter clock.
struct MessageHeaderProto {
  # Uniquely identifies a message across all systems
  uuid @0: UuidProto;
  # Uniquely identifies a proto type across all systems
  proto @1 :Int64;
  # A channel identifier which is used to route messages
  channel @2 :Text;
  # A (local) timestamp in nanoseconds which describes when data relevant for this message was
  # acquired by the hardware.
  acqtime @3 :Int64;
  # A (local) timestamp in nanoseconds which describes when the message was published.
  pubtime @4 :Int64;
  # Total length (in bytes) of message header and message payload
  messageLength @5: UInt32;
  # Lengths (in bytes) of proto segments
  segmentLengths @6: List(UInt32);
}

# A small header for every package we send over a UDP socket.
# It allows the remote to reassemble the message
struct UdpPackageHeaderProto {
  # Uniquely identifies a message across all systems
  uuid @0: UuidProto;
  # The total length of the message (header plus message payload) in bytes
  messageLength @1: UInt32;
  # The index of this package (assuming a fixed package size). The message header is always a
  # single package which comes first.
  packageIndex @2: UInt16;
}

# Header for logging messages
struct LogMessageProto {
  acqtime @0: Int64;
  pubtime @1: Int64;
  contentsUuid @2: UuidProto;
}

# An index containing all message channels in a log
struct MessageChannelIndexProto {
  struct MessageChannel {
    # UUID of the app component
    componentUuid @0: UuidProto;
    # Tag of the message channel (as specified in ISAAC_PROTO_TX)
    tag @1: Text;
    # Series UUID under which messages are stored in the log
    seriesUuid @2: UuidProto;
  }
  channels @0: List(MessageChannel);
}

# Information about the pose relation between two coordinate frames.
struct PoseTreeEdgeProto {
  # Name of "left" frame, i.e. `lhs` for a pose `lhs_T_rhs`
  lhs @0: Text;
  # Name of "right" frame, i.e. `rhs` for a pose `lhs_T_rhs`
  rhs @1: Text;
  # The pose between the two frames using the orientation `lhs_T_rhs`
  pose @2: Pose3dProto;
}
