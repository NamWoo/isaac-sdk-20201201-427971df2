/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/logger.hpp"

#include <fstream>
#include <sstream>
#include <regex>

#include "gtest/gtest.h"

namespace {

// Check if the contents of a file matches a regular expression
void ExpectFileMatches(const char* filename, const std::string& expression) {
  std::ifstream file(filename);
  std::stringstream buffer;
  buffer << file.rdbuf();

  const bool ok = std::regex_search(buffer.str(), std::regex(expression));
  if (!ok) {
    EXPECT_STREQ(buffer.str().c_str(), expression.c_str());
  }
}

// Check if the contents of a file are empty
void ExpectFileEmpty(const char* filename) {
  std::ifstream file(filename);
  std::stringstream buffer;
  buffer << file.rdbuf();

  EXPECT_STREQ(buffer.str().c_str(), "");
}

// Executes a log test
void RunLoggerTest(const std::function<void()> callback, const std::string& expression) {
  const char* filename = "out";
  std::FILE* fp = std::fopen(filename, "w");
  isaac::logger::Redirect(fp);
  callback();
  isaac::logger::Redirect(stdout);
  std::fclose(fp);
  if (expression == "") {
    ExpectFileEmpty(filename);
  } else {
    ExpectFileMatches(filename, expression);
  }
}


}  // namespace

TEST(Logger, Debug) {
  RunLoggerTest(
    [] { LOG_DEBUG("hello world"); },
    ".*DEBUG.*engine\\/core\\/tests\\/logger\\.cpp\\@.*hello world.*"
  );
}

TEST(Logger, Info) {
  RunLoggerTest(
    [] { LOG_INFO("you actually don't care"); },
    ".*INFO.*engine\\/core\\/tests\\/logger\\.cpp\\@.*you actually don't care.*"
  );
}

TEST(Logger, Warning) {
  RunLoggerTest(
    [] { LOG_WARNING("it is coming"); },
    ".*WARN.*engine\\/core\\/tests\\/logger\\.cpp\\@.*it is coming.*"
  );
}

TEST(Logger, Error) {
  RunLoggerTest(
    [] { LOG_ERROR("code review failure"); },
    ".*ERROR.*engine\\/core\\/tests\\/logger\\.cpp\\@.*code review failure.*"
  );
}

TEST(Logger, AtMostWarnings) {
  isaac::logger::SetSeverity(isaac::logger::Severity::WARNING);
  RunLoggerTest(
    [] { LOG_ERROR("code review failure"); },
    ".*ERROR.*engine\\/core\\/tests\\/logger\\.cpp\\@.*failure.*"
  );
  RunLoggerTest(
    [] { LOG_WARNING("it is coming"); },
    ".*WARN.*engine\\/core\\/tests\\/logger\\.cpp\\@.*it is coming.*"
  );
  RunLoggerTest([] { LOG_INFO("you actually don't care"); }, "");
  RunLoggerTest([] { LOG_DEBUG("hello world"); }, "");
  isaac::logger::SetSeverity(isaac::logger::Severity::ALL);
}
