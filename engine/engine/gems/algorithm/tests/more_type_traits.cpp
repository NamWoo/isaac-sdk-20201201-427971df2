/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>
#include <tuple>
#include <type_traits>

#include "engine/gems/algorithm/more_type_traits.hpp"
#include "gtest/gtest.h"

namespace isaac {

template <typename...>
struct MyGroupType {};

TEST(Unique, UniqueStaticTest) {
  static_assert(std::is_same<TypelistUnique<std::tuple, float, int, std::string, float, std::string>,
                             std::tuple<float, int, std::string>>::value,
                "Unique<> did not produce expected type");


  static_assert(std::is_same<TypelistUnique<MyGroupType, std::vector<std::string>, int, int>,
                             MyGroupType<std::vector<std::string>, int>>::value,
                "Unique<> did not produce expected type");
}

} //namespace isaac