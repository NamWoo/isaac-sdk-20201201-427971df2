/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/gems/serialization/json.hpp"

#include <string>

#include "engine/core/logger.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace serialization {

TEST(Json, MergeJsonObjectObject) {
  std::vector<int> v1{1, 2, 3};
  std::vector<std::string> v2{"a", "b", "c"};

  Json json1;
  json1["a"]["v1"] = v1;
  json1["a"]["1"] = nlohmann::json::array();
  json1["a"]["2"] = nlohmann::json::object();
  LOG_INFO("json1: %s", json1.dump(2).c_str());

  Json json2;
  json2["a"]["v2"] = v2;
  json2["a"]["3"] = 2;
  LOG_INFO("json2: %s", json2.dump(2).c_str());

  Json json3 = MergeJson(json1, json2);
  LOG_INFO("json3: %s", json3.dump(2).c_str());

  EXPECT_TRUE(json3["a"]["1"].is_array());
  EXPECT_TRUE(json3["a"]["2"].is_object());
  EXPECT_EQ(json3["a"]["v1"], v1);
  EXPECT_EQ(json3["a"]["v2"], v2);
  EXPECT_EQ(json3["a"]["3"], json2["a"]["3"]);
}

TEST(Json, MergeJsonNullObject) {
  Json json1;

  Json json2;
  json2["a"]["1"] = "a";
  json2["a"]["2"] = 2;
  LOG_INFO("json2: %s", json2.dump(2).c_str());

  Json json3 = MergeJson(json1, json2);
  LOG_INFO("json3: %s", json3.dump(2).c_str());

  EXPECT_EQ(json3, json2);
}

TEST(Json, MergeJsonObjectNull) {
  Json json1;
  json1["1"]["2"]["3"] = {};
  json1["a"] = {"1"};
  LOG_INFO("json1: %s", json1.dump(2).c_str());

  Json json2;

  Json json3 = MergeJson(json1, json2);
  LOG_INFO("json3: %s", json3.dump(2).c_str());

  EXPECT_EQ(json3, json1);
}

TEST(Json, MergeJsonArrayArray) {
  Json json1{1, 2, 3};
  LOG_INFO("json1: %s", json1.dump(2).c_str());

  Json json2{4, 5};
  LOG_INFO("json2: %s", json2.dump(2).c_str());

  Json json3 = MergeJson(json1, json2);
  LOG_INFO("json3: %s", json3.dump(2).c_str());

  EXPECT_TRUE(json1.is_array());
  EXPECT_TRUE(json2.is_array());
  EXPECT_TRUE(json3.is_array());
  EXPECT_EQ(json3, json2);
}

TEST(Json, MergeJsonEmptyArrayArray) {
  Json json1 = nlohmann::json::array();

  Json json2{4, 5};
  LOG_INFO("json2: %s", json2.dump(2).c_str());

  Json json3 = MergeJson(json1, json2);
  LOG_INFO("json3: %s", json3.dump(2).c_str());

  EXPECT_TRUE(json1.is_array());
  EXPECT_TRUE(json2.is_array());
  EXPECT_TRUE(json3.is_array());
  EXPECT_EQ(json3, json2);
}

TEST(Json, MergeJsonArrayEmptyArray) {
  Json json1{1, 2, 3};
  LOG_INFO("json1: %s", json1.dump(2).c_str());

  Json json2 = nlohmann::json::array();

  Json json3 = MergeJson(json1, json2);
  LOG_INFO("json3: %s", json3.dump(2).c_str());

  EXPECT_TRUE(json1.is_array());
  EXPECT_TRUE(json2.is_array());
  EXPECT_TRUE(json3.is_array());
  EXPECT_EQ(json3, json2);
}

TEST(Json, MergeJsonArrayObject) {
  Json json1{1, 2, 3};
  LOG_INFO("json1: %s", json1.dump(2).c_str());

  Json json2{{"a", 1}, {"b", true}, {"c", "3"}, {"d", {{"4", "5"}, {"6", 7}}}};
  LOG_INFO("json2: %s", json2.dump(2).c_str());

  Json json3 = MergeJson(json1, json2);
  LOG_INFO("json3: %s", json3.dump(2).c_str());

  EXPECT_TRUE(json1.is_array());
  EXPECT_TRUE(json2.is_object());
  EXPECT_TRUE(json3.is_object());
  EXPECT_EQ(json3, json2);
}

TEST(Json, MergeJsonObjectArray) {
  Json json1{{"a", 1}, {"b", true}, {"c", "3"}, {"d", {{"4", "5"}, {"6", 7}}}};
  LOG_INFO("json1: %s", json1.dump(2).c_str());

  Json json2{1, 2, 3};
  LOG_INFO("json2: %s", json2.dump(2).c_str());

  Json json3 = MergeJson(json1, json2);
  LOG_INFO("json3: %s", json3.dump(2).c_str());

  EXPECT_TRUE(json1.is_object());
  EXPECT_TRUE(json2.is_array());
  EXPECT_TRUE(json3.is_array());
  EXPECT_EQ(json3, json2);
}

TEST(Json, TryLoadJsonFromFile) {
  std::optional<Json> good =
      TryLoadJsonFromFile("engine/gems/serialization/tests/test_data/valid.json");
  EXPECT_TRUE(good != std::nullopt);

  std::optional<Json> bad =
      TryLoadJsonFromFile("engine/gems/serialization/tests/test_data/invalid.json");
  EXPECT_EQ(bad, std::nullopt);

  std::optional<Json> nonexist =
      TryLoadJsonFromFile("engine/gems/serialization/tests/test_data/nonexist.json");
  EXPECT_EQ(nonexist, std::nullopt);
}

}  // namespace serialization
}  // namespace isaac
