/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>

#include "engine/gems/serialization/json.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace serialization {

TEST(Serialization, StandardReplace) {
  Json json_a = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_a.json");

  // Replace two keys with new values
  std::map<std::string, std::string> key_map = {{"element_0", "A"},
                                                {"element_1", "B"}};

  int num_keys_replaced = ReplaceJsonKeys(key_map, json_a);
  EXPECT_EQ(num_keys_replaced, 2);

  Json json_f = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_f.json");
  EXPECT_TRUE(json_a == json_f);
}

TEST(Serialization, KeyNotFound) {
  Json json_a = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_a.json");
  Json json_a_copy = json_a;

  // Map without any existing keys
  std::map<std::string, std::string> key_map = {{"element_743", "A"},
                                                {"element_297", "B"},
                                                {"element_123", "C"}};

  int num_keys_replaced = ReplaceJsonKeys(key_map, json_a);
  EXPECT_EQ(num_keys_replaced, 0);
  EXPECT_TRUE(json_a == json_a_copy);
}

TEST(Serialization, MapWithRepeatName) {
  Json json_a = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_a.json");
  Json json_a_copy = json_a;

  // Map existing name to the same name
  std::map<std::string, std::string> key_map = {{"element_0", "element_0"},
                                                {"element_1", "element_1"}};

  int num_keys_replaced = ReplaceJsonKeys(key_map, json_a);

  // Count replacements, but expect no changes to json_a
  EXPECT_EQ(num_keys_replaced, 2);
  EXPECT_TRUE(json_a == json_a_copy);
}

TEST(Serialization, KeySwap) {
  Json json_c = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_c.json");

  // Swap names between three keys
  std::map<std::string, std::string> key_map = {{"element_0", "element_2"},
                                                {"element_1", "element_0"},
                                                {"element_2", "element_1"}};

  int num_keys_replaced = ReplaceJsonKeys(key_map, json_c);
  EXPECT_EQ(num_keys_replaced, 3);

  Json json_g = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_g.json");
  EXPECT_TRUE(json_c == json_g);
}

}  // namespace serialization
}  // namespace isaac
