/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "json.hpp"

#include <fstream>
#include <map>
#include <set>
#include <string>
#include <utility>

#include "engine/core/assert.hpp"

namespace isaac {
namespace serialization {

std::optional<Json> TryLoadJsonFromFile(const std::string& filename) {
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    return std::nullopt;
  }
  Json json = Json::parse(ifs, nullptr, false);
  if (json.is_discarded()) {
    return std::nullopt;
  }
  return json;
}

Json LoadJsonFromFile(const std::string& filename) {
  std::ifstream ifs(filename);
  ASSERT(ifs.is_open(), "Error opening file '%s'", filename.c_str());
  try {
    return Json::parse(ifs, nullptr, true);
  }
  catch (Json::parse_error &error) {
    PANIC("Error parsing JSON from file: %s\n%s", filename.c_str(), error.what());
  }
}

Json LoadJsonFromText(const std::string& text) {
  try {
    return Json::parse(text, nullptr, true);
  }
  catch (Json::parse_error &error) {
    PANIC("Error parsing JSON from text:\n%s\n%s", text.c_str(), error.what());
  }
}

bool WriteJsonToFile(const std::string& filename, const Json& json) {
  std::ofstream ofs(filename);
  if (!ofs) {
    return false;
  }
  ofs << json.dump(2);
  return true;
}

Json MergeJson(const Json& a, const Json& b) {
  if (a.is_null() || b.is_null()) {
    return a.is_null() ? b : a;
  }
  // If a or b is not an object, pick b to overwrite values
  if (!a.is_object() || !b.is_object()) {
    return b;
  }
  Json result = a;
  for (auto it = b.begin(); it != b.end(); ++it) {
    if (result.find(it.key()) == result.end()) {
      result[it.key()] = it.value();
    } else {
      result[it.key()] = MergeJson(result[it.key()], it.value());
    }
  }
  return result;
}

std::optional<Json> ParseJson(const std::string& text) {
  Json json = Json::parse(text, nullptr, false);
  if (json.is_discarded()) {
    return std::nullopt;
  }
  return json;
}

int ReplaceJsonKeys(const std::map<std::string, std::string>& key_map, Json& json) {
  int num_keys_replaced = 0;

  // If one of the new keys provided is an existing key, a temporary extension is appended and
  // later removed. This allows ReplaceJsonKeys() the be used to swap keys.
  std::string temp_extension = "_TEMPORARY_EXTENSION";
  std::set<std::string> renamed_needed;

  // Iterate over the provided map, renaming a key if found
  for (const auto& map_it : key_map) {
    auto json_it = json.find(map_it.first);
    // If old key is found, perform replacement
    if (json_it != json.end()) {
      // If a repeat value is used in map pair, do NOT change json, but still increase replacement
      // count.
      if (map_it.first != map_it.second) {
        // If new key is an existing key, append temporary extension and set flag to do clean up
        // of temporary extensions. Else, perform standard swap.
        if (json.find(map_it.second) != json.end()) {
          renamed_needed.insert(map_it.second);
          json[map_it.second + temp_extension] = std::move(json[map_it.first]);
        } else {
          json[map_it.second] = std::move(json[map_it.first]);
        }
        // Erase json with old key
        json.erase(map_it.first);
      }
      num_keys_replaced++;
    }
  }

  // If a temporary extension was needed, perform cleanup
  for (const auto& name : renamed_needed) {
    const std::string full_name = name + temp_extension;
    json[name] = std::move(json[full_name]);
    json.erase(json.find(full_name));
  }

  return num_keys_replaced;
}

}  // namespace serialization
}  // namespace isaac
