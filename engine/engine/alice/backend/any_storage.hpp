/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <shared_mutex>  // NOLINT
#include <string>
#include <utility>

#include "engine/core/any.hpp"
#include "engine/core/optional.hpp"

namespace isaac {
namespace alice {

// A thread-safe key-value storage which can store anything
class AnyStorage {
 public:
  // Tries to get the value for `key` types as `T`. Returns nullopt if no such key, or wrong type.
  std::optional<double> tryGet(const std::string& key) const {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    const auto it = storage_.find(key);
    if (it == storage_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  // Sets the value for `key`. Creates a new entry if it does not yet exist, otherwise overwrites
  // the existing one (even if type differs).
  void set(const std::string& key, double value) {
    std::unique_lock<std::shared_timed_mutex> lock(mutex_);
    storage_[key] = value;
  }

 private:
  mutable std::shared_timed_mutex mutex_;
  std::map<std::string, double> storage_;
};

}  // namespace alice
}  // namespace isaac
