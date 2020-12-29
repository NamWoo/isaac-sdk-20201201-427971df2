/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "network.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <string>

#include "engine/core/logger.hpp"

namespace isaac {
namespace system {

bool GetIpForAdapter(const std::string& adapter, std::string& ip) {
  ifaddrs *if_address, *ifa;
  int success;
  char host[64];

  if (getifaddrs(&if_address) == -1) {
    LOG_ERROR("No network interfaces is available on this machine");
  }

  for (ifa = if_address; ifa != nullptr; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == nullptr) {
      continue;
    }
    success = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, sizeof(NI_MAXSERV), NULL,
                          0, NI_NUMERICHOST);

    if ((std::strcmp(ifa->ifa_name, adapter.c_str()) == 0) &&
        (ifa->ifa_addr->sa_family == AF_INET)) {
      if (success != 0) {
        LOG_ERROR("getnameinfo() failed: %s\n", gai_strerror(success));
        return false;
      }
      ip = ifa->ifa_name;
    }
  }
  freeifaddrs(if_address);
  return true;
}
}  // namespace system
}  // namespace isaac
