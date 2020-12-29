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
#include "engine/gems/sight/named_sop.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace sight {
namespace named_sop_details {

TEST(Sight, StandardTruncation) {
  Json json_h = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_a.json");

  TruncateJsonKeys(json_h);

  Json json_i = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_b.json");
  EXPECT_TRUE(json_h == json_i);
}

TEST(Sight, ArraysWithMoreThanTenElements) {
  // In the flattened JSON structure, array indices are included as part of the string
  // representation of the key. These values should not be truncated.
  Json json_j = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_c.json");

  TruncateJsonKeys(json_j);

  Json json_k = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_d.json");
  EXPECT_TRUE(json_j == json_k);
}

TEST(Serialization, KeysWithOnlyNumbers) {
  // In current version of TruncateJsonKeys(), keys that consist only of numbers will not be
  // truncated
  Json json_l = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_e.json");
  Json json_l_copy = json_l;

  TruncateJsonKeys(json_l);

  EXPECT_TRUE(json_l == json_l_copy);
}

TEST(Sight, KeysThatShareFirstLetter) {
  //  If two keys at the same level are truncated to the same letter, their contents will be
  // merged (i.e., the contents of the second object will append or overwrite the first)
  Json json_m = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_f.json");

  TruncateJsonKeys(json_m);

  Json json_n = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_g.json");
  EXPECT_TRUE(json_m == json_n);
}

TEST(Sight, KeysThatBeginWithNumbers) {
  // Truncation that results in changing a key to "0" will cause that key to be interpreted as a
  // array index, likely causing undesired behaviour. JSON containing the key "0" cannot be
  // reversibly flattened using nlohmann/json (i.e., my_json.flatten().unflatten() != my_json)

  // In the case that one key is truncated to "0" and another key is truncated to a letter, an
  // exception will be thrown:
  Json json_o = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_h.json");
  ASSERT_ANY_THROW(TruncateJsonKeys(json_o));

  // In the case that one key is truncated to "0" and another key is truncated to a higher number,
  // these keys will be reinterpreted as array indices:
  Json json_p = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_i.json");
  TruncateJsonKeys(json_p);
  Json json_q = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_j.json");
  EXPECT_TRUE(json_p == json_q);

  // In the case that one key are truncated to nonzero numbers, none of the keys will be
  // reinterpreted as array indices.
  Json json_r = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_k.json");
  TruncateJsonKeys(json_r);
  Json json_s = serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/sample_l.json");
  EXPECT_TRUE(json_s == json_s);
}

}  // namespace named_sop_details
}  // namespace sight
}  // namespace isaac
