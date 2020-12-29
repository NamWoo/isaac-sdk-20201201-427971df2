# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from checksumdir import dirhash
import subprocess

PACKAGE_DIR = "packages/sensor_certification/apps/"


# Add color format codes to the given text
def colorize(text, style):
    formats = {
        "GOOD": "\033[92m{}\033[0m",
        "BAD": "\033[91m{}\033[0m",
        "HEADER": "\033[1m{}\033[0m",
        "WARN": "\033[33m{}\033[0m",
    }
    return formats[style].format(text)


# Computes checksum on the directory present in input_file
def checksum(input_file, directory):
    dir = ""
    chksum = ""
    file = open(input_file, "r")
    lines = file.readlines()
    file.close()
    for line in lines:
        line = ''.join(line.split())
        if (line.find(directory) != -1):
            dir = line[len(directory):].strip('"')
            chksum = dirhash(dir, 'md5')
            return chksum
    return 0


# Returns platform info such as target platform, jetpack sdk version
def get_platform_info(platform, search_info):
    platform = PACKAGE_DIR + platform
    command = ['bash', '-c', 'source {} && env'.format(platform)]
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, universal_newlines=True)

    for line in proc.stdout:
        (key, _, value) = str(line).partition("=")
        if (key == search_info):
            return value.strip()
    return ""


# Construct and return a function that will check if a value is in a given range
def range_checker(minimum, maximum):
    def check(value):
        if not (value <= maximum and value >= minimum):
            raise ValueError("Value {} is outside allowed range [{}, {}]".format(
                value, minimum, maximum))

    return check
