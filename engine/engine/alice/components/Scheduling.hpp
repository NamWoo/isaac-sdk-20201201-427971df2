/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/component.hpp"

namespace isaac {
namespace alice {

// This component contains scheduling information for codelets. Parameters apply to all components
// in a node. If the component is not present, default parameters are used.
class Scheduling : public Component {
 public:
  // Controls the relative priority of a codelet task within a timeslice window
  // Used for periodic and event driven codelets.
  // Higher values have higher priority
  ISAAC_PARAM(int, priority, 0);

  // Controls how much variation in start time is allowed when executing a codelet
  // Used for periodic and event driven codelets.
  // The parameter unit is seconds
  ISAAC_PARAM(double, slack, 0);

  // Set the expected time that the codelet will take to complete processing.
  // If no value is specified periodic tasks will assume the period of the task
  // and other tasks will assume there is no deadline.
  // The parameter unit is seconds
  ISAAC_PARAM(double, deadline);

  // Sets the execution group for the codelet.
  // Users can define groups in the scheduler configuration. If an execution_group
  // is specified it overrides default behaviors.
  //
  // If no value is specified it will attempt to use the default configuration
  // The default configuration provided creates three groups
  //  -BlockingGroup -- Blocking threads run according to OS scheduling. Default for tickBlocking.
  //  -WorkerGroup -- One Worker thread per core execute tick functions for tickPeriodic/OnEvent.
  // Note: tickBlocking spawns a worker thread for the blocking task which if executed
  // in the WorkerGroup can interfere with worker thread
  // execution due to OS scheduling. Removing the default groups could lead to
  // instabilities if not careful.
  ISAAC_PARAM(std::string, execution_group, "");
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::Scheduling)
