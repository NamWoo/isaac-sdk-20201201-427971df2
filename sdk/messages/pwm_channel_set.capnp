#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xff45529b9755494f;

# Set a duty cycle for a given PWM channel
struct PwmChannelSetDutyCycleProto {
  channel @0 :Int32;      # PWM channel to set
  dutyCycle @1 :Float32;  # duty cycle, as a percentage (0.0 to 1.0)
  disable @2 :Bool;       # if set to true, duty cycle is ignored and power is set to 0
}

# Set a pulse length for a given PWM channel
struct PwmChannelSetPulseLengthProto {
  channel @0 :Int32;       # PWM channel to set
  pulseLength @1 :Float32; # length of pulse, as a percentage (0.0 to 1.0)
}
