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
#include <utility>

#include "engine/alice/backend/sight_backend.hpp"
#include "engine/alice/component.hpp"
#include "engine/core/image/image.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

// This component is a proxy to access and expose sight functionalities to components. This
// component is added to every node by default. It should not be added to a node manually.
class Sight : public Component {
 public:
  virtual ~Sight() {}

  void initialize() override;
  void deinitialize() override;

  // Show operation for variables
  template <typename T, std::enable_if_t<!sight::is_sop_callback_v<T>, int> = 0>
  void show(const Component* component, const std::string& name, double time, const T& value) {
    backend_->show(component, name, time, value);
  }
  // Show operation based on a callback
  void show(const Component* component, const std::string& name, double time,
            std::function<void(sight::Sop&)> callback) {
    backend_->show(component, name, time, std::move(callback));
  }
  // Show operation based on an existing SOP
  void show(const Component* component, const std::string& name, double time, sight::Sop sop) {
    backend_->show(component, name, time, std::move(sop));
  }
  // Show operation based on an existing JSON object
  void show(Component* component, Json json) {
    backend_->show(component, std::move(json));
  }
  // Specialized show operation for images
  template<typename K, int N>
  void show(const Component* component, const std::string& name, double time,
            const Image<K, N>& img) {
    backend_->show(component, name, time, [&](sight::Sop& sop) { sop.add(img); });
  }
  // Show operation for pose2
  template <typename K>
  void show(const Component* component, const std::string& name, double time,
            const Pose2<K>& value) {
    backend_->show(component, name + ".x", time, value.translation.x());
    backend_->show(component, name + ".y", time, value.translation.y());
    backend_->show(component, name + ".a", time, value.rotation.angle());
  }
  // Show operation for pose3
  template <typename K>
  void show(const Component* component, const std::string& name, double time,
            const Pose3<K>& value) {
    backend_->show(component, name + ".p.x", time, value.translation.x());
    backend_->show(component, name + ".p.y", time, value.translation.y());
    backend_->show(component, name + ".p.z", time, value.translation.z());
    backend_->show(component, name + ".q.w", time, value.rotation.quaternion().w());
    backend_->show(component, name + ".q.x", time, value.rotation.quaternion().x());
    backend_->show(component, name + ".q.y", time, value.rotation.quaternion().y());
    backend_->show(component, name + ".q.z", time, value.rotation.quaternion().z());
  }

 private:
  friend class SightBackend;
  SightBackend* backend_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::Sight)
