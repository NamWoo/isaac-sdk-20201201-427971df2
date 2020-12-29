/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "serial_baudrate.hpp"

#include <asm/termbits.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

#include "engine/core/logger.hpp"
#include "engine/core/time.hpp"

namespace isaac {

void SetSerialBaudrate(int fd, uint64_t baudrate) {
  // Termios cannot set non-standard baudrates. In addition, termios.h and asm/termbits.h have
  // conflicting definitions. So we break out setting the baudrate into its own compilation unit.

  // Device driver writers are then free to use the termios functions for the rest of their
  // settings, instead of the undocumented TC*S2 family ioctl methods.

  struct termios2 tio;

  if (ioctl(fd, TCGETS2, &tio) < 0) {
    LOG_ERROR("Error setting bitrate: %s [errno %d]", strerror(errno), errno);
    return;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag |= BOTHER;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;

  if (ioctl(fd, TCSETS2, &tio) < 0) {
    LOG_ERROR("Error setting bitrate: %s [errno %d]", strerror(errno), errno);
    return;
  }

  if (ioctl(fd, TCGETS2, &tio) < 0) {
    LOG_ERROR("Error setting bitrate: %s [errno %d]", strerror(errno), errno);
    return;
  }

  // Give time for device to update
  Sleep(SecondsToNano(0.1));
}

}  // namespace isaac
