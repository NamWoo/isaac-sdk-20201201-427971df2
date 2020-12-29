#!/usr/bin/python3
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from packages.pyalice import Application
from os import path, mkdir
from TestRunners import TestRunnerFactory
from Utils import colorize
import argparse
import json
import jsonschema
import time
import signal
import sys


# Parse the sensor specification and generate a list of modes
class SpecParser:
    def __init__(self, spec_file, report_dir, verbose):
        SCHEMA = {
            "type": "object",
            "properties": {
                "codelet": {
                    "type": "string"
                },
                "module_name": {
                    "type": "string"
                },
                "sensor_name": {
                    "type": "string"
                },
                "sw_version": {
                    "type": "string"
                },
                "sensor_modes": {
                    "type": "array",
                    "minItems": 1,
                    "items": {
                        "type": "object",
                        "properties": {
                            "type": {
                                "type": "string"
                            },
                            "sensor_spec": {
                                "type": "object"
                            }
                        },
                        "required": ["type", "sensor_spec"]
                    }
                },
            },
            "required": ["codelet", "module_name", "sensor_modes", "sensor_name"]
        }
        spec = json.load(spec_file)
        jsonschema.validate(instance=spec, schema=SCHEMA)

        module_spec = {
            "codelet": spec["codelet"],
            "module_name": spec["module_name"],
            "sensor_name": spec["sensor_name"],
            "sw_version": spec["sw_version"]
        }

        self.test_runners = [
            TestRunnerFactory.create_from_json(module_spec, mode, report_dir, verbose)
            for mode in spec["sensor_modes"]
        ]

    def get_test_runners(self):
        return self.test_runners


def write_report():
    report_file = '{}/report.txt'.format(args.report_dir)

    with open(report_file, 'w') as f:
        f.write(test_runner.to_report())

    print("--- Test Summary ---")
    print(test_runner.print_summary())
    print("\nReport was written to - {}".format(colorize(args.report_dir, "HEADER")))


def keyboard_interrupt_signal_handler(sig, frame):
    write_report()
    sys.exit(0)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        usage='bazel run sensor_certification -- <sensor_spec_file> [-h] [-q] [-r REPORT_FILE]'
              '[-v VERBOSE_OPTION]',
        description='Verify your sensor driver with ISAAC sensor certification framework to be \
        certified as an ecosystem partner')
    parser.add_argument('sensor_spec_file',
                        type=argparse.FileType('r'),
                        help="Sensor specification JSON file (provide absolute path)")
    parser.add_argument('-q',
                        '--query',
                        action="store_true",
                        help="list all the testcases for the sensor device")
    parser.add_argument(
        '-r',
        '--report_dir',
        help="Absolute path of test report directory (default: /tmp/sensor_certification)",
        default="/tmp/sensor_certification")
    parser.add_argument('-v',
                        '--verbose',
                        action="store_true",
                        help="Option to print out all ISAAC SDK log messages")

    args = parser.parse_args()
    timestr = time.strftime("_%m-%d-%Y_%I-%M-%S_%p")
    args.report_dir += timestr

    test_runners = SpecParser(args.sensor_spec_file,
                              args.report_dir,
                              args.verbose).get_test_runners()

    if args.query:
        for runner in test_runners:
            print(runner.get_definition_str())

    else:
        signal.signal(signal.SIGINT, keyboard_interrupt_signal_handler)

        for test_runner in test_runners:
            mkdir(args.report_dir)
            test_runner.prompt()
            test_runner.run_tests()
            write_report()
