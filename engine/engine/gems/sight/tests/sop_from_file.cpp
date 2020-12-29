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
#include "engine/gems/sight/sop.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace sight{

class SopFromFile: public testing::Test {
 protected:
  const Json expected_consolidated_json =
    serialization::LoadJsonFromFile("engine/gems/sight/tests/test_data/consolidated-sop.json");
};

TEST_F(SopFromFile, AssignmentFromJsonMergerRvalueReference) {
  sight::Sop sop;
  sop = serialization::JsonMerger().withFile("engine/gems/sight/tests/test_data/named-sop.json");

  Json consolidated_json_from_sop = sop.moveJson();
  EXPECT_TRUE(consolidated_json_from_sop == expected_consolidated_json);
}

TEST_F(SopFromFile, ConstructionFromJsonMergerRvalueReference) {
  sight::Sop sop =
    serialization::JsonMerger().withFile("engine/gems/sight/tests/test_data/named-sop.json");

  Json consolidated_json_from_sop = sop.moveJson();
  EXPECT_TRUE(consolidated_json_from_sop == expected_consolidated_json);
}

TEST_F(SopFromFile, AssignmentFromJsonMerger) {
  serialization::JsonMerger json_merger =
    serialization::JsonMerger().withFile("engine/gems/sight/tests/test_data/named-sop.json");
  sight::Sop sop;
  sop = json_merger;

  Json consolidated_json_from_sop = sop.moveJson();
  EXPECT_TRUE(consolidated_json_from_sop == expected_consolidated_json);
}

TEST_F(SopFromFile, ConstructionFromJsonMerger) {
  serialization::JsonMerger json_merger =
    serialization::JsonMerger().withFile("engine/gems/sight/tests/test_data/named-sop.json");
  sight::Sop sop = json_merger;

  Json consolidated_json_from_sop = sop.moveJson();
  EXPECT_TRUE(consolidated_json_from_sop == expected_consolidated_json);
}

TEST_F(SopFromFile, AssignmentFromJson) {
  Json json = serialization::JsonMerger().withFile("engine/gems/sight/tests/test_data/named-sop.json");
  sight::Sop sop;
  sop = json;

  Json consolidated_json_from_sop = sop.moveJson();
  EXPECT_TRUE(consolidated_json_from_sop == expected_consolidated_json);
}

TEST_F(SopFromFile, ConstructionFromJson) {
  Json json = serialization::JsonMerger().withFile("engine/gems/sight/tests/test_data/named-sop.json");
  sight::Sop sop = json;

  Json consolidated_json_from_sop = sop.moveJson();
  EXPECT_TRUE(consolidated_json_from_sop == expected_consolidated_json);
}

}  // namespace serialization
}  // namespace isaac
