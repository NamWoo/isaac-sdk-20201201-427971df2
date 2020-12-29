/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "sight.hpp"

#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "engine/core/logger.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/sight/sight_interface.hpp"

namespace isaac {
namespace sight {

namespace {
SightInterface* sight_;
std::mutex mutex_;
}  // namespace

void ResetSight(SightInterface* sight) {
  std::unique_lock<std::mutex> lock(mutex_);
  sight_ = sight;
}

void Plot(const std::string& name, double time, float value) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sight_) return;
  sight_->plotValue(name, time, value);
}
void Plot(const std::string& name, double time, double value) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sight_) return;
  sight_->plotValue(name, time, value);
}
void Plot(const std::string& name, double time, int value) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sight_) return;
  sight_->plotValue(name, time, value);
}
void Plot(const std::string& name, double time, int64_t value) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sight_) return;
  sight_->plotValue(name, time, value);
}

void Plot(const std::string& name, float value) {
  Plot(name, std::chrono::duration_cast<std::chrono::seconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count(), value);
}
void Plot(const std::string& name, double value) {
  Plot(name, std::chrono::duration_cast<std::chrono::seconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count(), value);
}
void Plot(const std::string& name, int value) {
  Plot(name, std::chrono::duration_cast<std::chrono::seconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count(), value);
}
void Plot(const std::string& name, int64_t value) {
  Plot(name, std::chrono::duration_cast<std::chrono::seconds>(
                 std::chrono::system_clock::now().time_since_epoch()).count(), value);
}

void Draw(const std::string& name, sight::Sop sop) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sight_) return;
  sight_->drawCanvas(name, std::move(sop));
}

void Draw(const std::string& name, const ImageConstView3ub& img) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sight_) return;
  sight::Sop sop;
  sop.add(img);
  sight_->drawCanvas(name, std::move(sop));
}

void Draw(const std::string& name, const ImageConstView4ub& img) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sight_) return;
  sight::Sop sop;
  sop.add(sight::SopImage::Png(img));
  sight_->drawCanvas(name, std::move(sop));
}

void Draw(const std::string& name, const ImageConstView1ub& img) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sight_) return;
  sight::Sop sop;
  sop.add(img);
  sight_->drawCanvas(name, std::move(sop));
}

}  // namespace sight
}  // namespace isaac
