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

namespace isaac {

// Class wrapper for serial port com under linux
class Serial {
 public:
  Serial(const std::string& dev_port, int baudrate);
  ~Serial();

  // Writes characters to the serial port
  void writeChars(const unsigned char* c, size_t size);
  // Reads characters from the serial port
  int readChars(unsigned char* buffer, size_t buffer_size);
  // Sets DTR (https://en.wikipedia.org/wiki/Data_Terminal_Ready)
  void setDTR(bool level);

 private:
  int fd_;
  std::string port_name_;
};

}  // namespace isaac
