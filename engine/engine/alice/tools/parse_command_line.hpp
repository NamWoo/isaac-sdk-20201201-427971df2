/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>

#include "engine/alice/backend/application_json_loader.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

// Parses gflags command line parameter to get the desired application JSON file. Run the
// application with --help to see all available command line parameters.
ApplicationJsonLoader ParseApplicationCommandLine();

// Same as ParseApplicationCommandLine(string) but the given name is used to overwrite the name
// of the application JSON file given via command line, or it is used if no application JSON object
// is specified.
ApplicationJsonLoader ParseApplicationCommandLine(const std::string& name);

}  // namespace alice
}  // namespace isaac
