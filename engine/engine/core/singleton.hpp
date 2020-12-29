/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

namespace isaac {

// A singleton which supports creation at pre-execution time
// Usage: For a class `MyType` with a member function `myMember` you can just write:
//   Singleton<MyType>::Get().myMember();
// This will automatically create a singleton, i.e. return the same instance each time you call Get.
// The constructor of MyType will be called at pre-execution time, i.e. before entering main.
// If you use multiple singletons they will all be created at pre-execution time, but you should
// not rely on a particular instantiation order.
template <typename T>
struct Singleton {
 public:
  Singleton& operator=(Singleton&) = delete;

  // Get the singleton
  static T& Get() {
    static T singleton;
    Use(dummy_);
    return singleton;
  }

 private:
  // Helpers to force pre-execution time
  static void Use(const T&) {}
  static T& dummy_;
};

// Force instantiation of the singleton at pre-execution time
template <typename T>
T& Singleton<T>::dummy_ = Singleton<T>::Get();

}  // namespace isaac
