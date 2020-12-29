/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>
#include <random>
#include <string>

#include "engine/gems/algorithm/string_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(String, StartsEndsWithSome) {
  EXPECT_TRUE(StartsWith("", ""));
  EXPECT_TRUE(StartsWith("a", ""));
  EXPECT_FALSE(StartsWith("", "a"));
  EXPECT_TRUE(StartsWith("abab", "aba"));
  EXPECT_FALSE(StartsWith("abab", "bab"));
  EXPECT_TRUE(EndsWith("", ""));
  EXPECT_TRUE(EndsWith("a", ""));
  EXPECT_FALSE(EndsWith("", "a"));
  EXPECT_FALSE(EndsWith("abab", "aba"));
  EXPECT_TRUE(EndsWith("abab", "bab"));
}

TEST(String, StartsWith) {
  std::uniform_int_distribution<size_t> length(0, 79);
  std::default_random_engine rng(1337);
  for (size_t i = 0; i < 20; i++) {
    const std::string a = RandomAlphaNumeric(length(rng), rng);
    const std::string b = RandomAlphaNumeric(length(rng), rng);
    const std::string c = a + b;
    EXPECT_TRUE(StartsWith(c, a));
    if (c != b) {
      EXPECT_FALSE(StartsWith(c, b));  // It works with the seed above
    }
  }
}

TEST(String, EndsWith) {
  std::uniform_int_distribution<size_t> length(0, 79);
  std::default_random_engine rng(1337);
  for (size_t i = 0; i < 20; i++) {
    const std::string a = RandomAlphaNumeric(length(rng), rng);
    const std::string b = RandomAlphaNumeric(length(rng), rng);
    const std::string c = a + b;
    if (c != b) {
      EXPECT_FALSE(EndsWith(c, a));  // It works with the seed above
    }
    EXPECT_TRUE(EndsWith(c, b));
  }
}

TEST(String, ToLowerCase) {
  EXPECT_EQ("moar", ToLowerCase("mOaR"));
  EXPECT_EQ("mo!/ar", ToLowerCase("mO!/aR"));
}

TEST(String, TrimString) {
  EXPECT_EQ("test1", TrimString(" test1"));
  EXPECT_EQ("%test2", TrimString("%test2 "));
  EXPECT_EQ("test3", TrimString(" test3 "));
  EXPECT_EQ("#test-case4", TrimString("   #test-case4    "));
  EXPECT_EQ("5Test   case5", TrimString("    5Test   case5    "));
}

}  // namespace isaac
