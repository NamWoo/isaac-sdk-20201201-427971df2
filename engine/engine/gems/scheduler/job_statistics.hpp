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
#include <vector>

#include "engine/core/time.hpp"
#include "engine/gems/math/exponential_moving_average.hpp"
#include "engine/gems/math/fast_running_median.hpp"

namespace isaac {
namespace scheduler {

// Statistics about job execution
struct JobStatistics {
  JobDescriptor descriptor;
  // Number of times the job was executed.
  int num_executed = 0;
  // Number of times the job overran its execution window
  int num_overrun = 0;
  // Total time in nanoseconds spend executing this job
  int64_t total_time = 0;
  // Total time in nanoseconds the job was overran its execution schedule
  int64_t total_time_overrun = 0;
  // Total time the job was idel
  int64_t total_idle = 0;
  // Last time the job stopped
  int64_t last_stop_time = 0;
  // The current average load, i.e. how much time it spends running vs idle
  math::ExponentialMovingAverageRate<double> current_load;
  // Current average frequency of execution
  math::ExponentialMovingAverageRate<double> current_rate;
  // Moving average of the time spent executing this job
  math::ExponentialMovingAverage<double> exec_dt;
  // Moving average of the delay starting this job
  math::ExponentialMovingAverage<double> execution_delay;
  // Running medians of the execution time
  math::FastRunningMedianImpl<double, std::vector<double>> execution_time_median;

  JobStatistics(size_t num_median_samples = 128)
  : execution_time_median(std::vector<double>(num_median_samples)) {}

  // Relative percentage this job was behind execution schedule
  double getOverrunPercentage() const {
    if (num_executed == 0) {
      return 0.0;
    }
    return static_cast<double>(num_overrun) / static_cast<double>(num_executed) * 100.0;
  }

  // The average time this job spent per execution
  double getAverageExecutionTime() const {
    if (num_executed == 0) {
      return 0.0;
    }
    return ToSeconds(static_cast<double>(total_time) / static_cast<double>(num_executed));
  }

  // The average time the job overran by, 0 if it never had an overrun
  double getAverageOverrunTime() const {
    if (num_overrun == 0) {
      return 0.0;
    }
    return ToSeconds(static_cast<double>(total_time_overrun) / static_cast<double>(num_overrun));
  }

  // The relative amount of time this job is running
  double getLifetimeLoad() const {
    if (total_time + total_idle > 0) {
      return static_cast<double>(total_time) / static_cast<double>(total_time + total_idle);
    } else {
      return 0.0;
    }
  }
};

}  // namespace scheduler
}  // namespace isaac
