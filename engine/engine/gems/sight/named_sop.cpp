/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "named_sop.hpp"

#include <map>
#include <string>
#include <utility>

namespace isaac {
namespace sight {
namespace named_sop_details {

namespace {

// If the key consists of only numbers, return a copy of the key, else only return the first
// character
std::string TruncateIfNotNumber(const std::string& key) {
  if (std::all_of(key.begin(), key.end(), ::isdigit)) {
    return key;
  } else {
    return std::string(1, key.front());
  }
}

}  // namespace

void TruncateJsonKeys(Json& json) {
  // Flatten json for iteration
  Json flattened_json = json.flatten();

  // Delimiter used to separate individual Json keys in flattened structure
  std::string delim = "/";

  // Store map of keys to be replaced. Here keys refer to keys in the flattened json, where
  // each key provides the full path through the Json hierarchy
  //   First value: original long key.
  //   Second value: abbreviated key (with single letter per level).
  std::map<std::string, std::string> truncation_map;

  for (const auto& it : flattened_json.items()) {
    // Copy of original key, with leading delimiter stripped
    std::string original_key = it.key().substr(delim.length(), it.key().length());

    // Build up truncated key while parsing original, initialize with leading delimiter
    std::string truncated_key = delim;

    size_t position = 0;
    while ((position = original_key.find(delim)) != std::string::npos) {
      std::string key = original_key.substr(0, position);
      truncated_key += TruncateIfNotNumber(key) + delim;
      original_key.erase(0, position + delim.length());
    }
    truncated_key += TruncateIfNotNumber(original_key);
    truncation_map.insert(std::make_pair(it.key(), truncated_key));
  }

  serialization::ReplaceJsonKeys(truncation_map, flattened_json);
  json = flattened_json.unflatten();
}

}  // namespace named_sop_details

void ConvertFromNamedSop(const Json& named_sop, Json& sop) {
  // Delete any existing data from Sop
  sop.clear();
  sop["type"] = "sop";

  // Iterate through each independent named Sop and add contents to consolidated Sop
  for (auto it : named_sop.items()) {
    Json temp;
    temp["type"] = "sop";
    temp["data"].push_back(it.value());
    sop["data"].push_back(temp);
  }

  // Convert all keys to single letters (e.g., "pose" to "p")
  named_sop_details::TruncateJsonKeys(sop);
}

}  // namespace sight
}  // namespace isaac
