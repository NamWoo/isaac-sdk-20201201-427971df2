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

TEST(Serialization, AddElement) {
  // Add element_2 from sample_b.json to the elements 0 and 1 in sample_a.json
  Json json_a_with_b = JsonMerger().withFile("engine/gems/serialization/tests/test_data/sample_a.json")
                                   .withFile("engine/gems/serialization/tests/test_data/sample_b.json");

  Json json_c = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_c.json");

  EXPECT_TRUE(json_a_with_b == json_c);
}

TEST(Serialization, EditAttributes) {
  // Use sample_d.json to edit the colors of elements in sample_c.json
  Json json_c_with_d = JsonMerger().withFile("engine/gems/serialization/tests/test_data/sample_c.json")
                                   .withFile("engine/gems/serialization/tests/test_data/sample_d.json");

  Json json_e = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_e.json");

  EXPECT_TRUE(json_c_with_d == json_e);
}

TEST(Serialization, WithJson) {
  Json json_a = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_a.json");
  Json json_b = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_b.json");
  Json json_d = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_d.json");
  Json json_e = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_e.json");


  // Test merging JSON objects rather than filenames
  Json json_a_with_b_with_d = JsonMerger().withJson(json_a)
                                          .withJson(json_b)
                                          .withJson(json_d);

  EXPECT_TRUE(json_a_with_b_with_d == json_e);
      //EXPECT_TRUE(false);
}

TEST(Serialization, SharedJsonMerger) {
  // Create JsonMerger object with contents of sample_a.json
  JsonMerger json_merger_a =
          JsonMerger().withFile("engine/gems/serialization/tests/test_data/sample_a.json");

  // Add contents of sample_b.json to sample_a.json
  // This will make a copy of the json_ from json_merger_a, leaving json_merger_a with the contents
  // of sample_a.json and sample_b.json
  Json json_a_with_b = json_merger_a.withFile("engine/gems/serialization/tests/test_data/sample_b.json");
  Json json_c = LoadJsonFromFile("engine/gems/serialization/tests/test_data/sample_c.json");
  EXPECT_TRUE(json_a_with_b == json_c);

  // Reuse json_merger_a to generate another JSON object with contents of sample_b.json added to
  // sample_a.json, copying json_ from json_merger_a
  Json new_json_a_with_b = json_merger_a;
  EXPECT_TRUE(new_json_a_with_b == json_a_with_b);

  // Reuse json_merger_a to generate another JSON object with contents of sample_b.json added to
  // sample_a.json, moving json_ from json_merger_a
  Json newer_json_a_with_b = std::move(json_merger_a);
  EXPECT_TRUE(newer_json_a_with_b == json_a_with_b);

  // Reuse json_merger_a to generate an empty JSON object, since the contents were explicitly moved
  Json newest_json_a_with_b = json_merger_a;
  Json empty_json;
  EXPECT_TRUE(newest_json_a_with_b == empty_json);
}

}  // namespace serialization
}  // namespace isaac
