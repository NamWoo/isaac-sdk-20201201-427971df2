# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from checksumdir import dirhash
from datetime import datetime
from Tests import *
from Utils import colorize, checksum, get_platform_info
import jsonschema
import json
import os

SUPPORTED_COLOR_SPACES = ["RGB", "GRAYSCALE", "BGR", "YUV", "RGBA"]

REPORT_FORMAT = """
---- ISAAC Sensor Certification Test Report ----
Sensor Name         - {sensor}
Sensor SW Version   - {version}
Sensor SW Checksum  - {sensor_chksum}
ISAAC SDK Version   - 2020.2
ISAAC SW Checksum   - {isaac_chksum}
TARGET_PLATFORM     - {platform}
JETPACK SDK Version - {jetpack}
Report Generation   - {date}

--- Summary ---
{summary}
--- Detailed Test Runs ---
{tests}
"""


# A class to run all the tests for a given sensor type
class TestRunner:
    def __init__(self, module_spec, sensor_spec, sensor_drv_config, report_dir, verbose):
        self.tests = [
            test_type(module_spec, sensor_spec, sensor_drv_config, report_dir, verbose)
            for test_type in self.get_test_types()
        ]

    # Print the definition of all tests in the test suite
    def get_definition_str(self):
        string = "{} is selected with mode:\n{}\nThe following ({}) tests will be run:\n".format(
            self.__class__.__name__, json.dumps(self.sensor_spec, indent=4), len(self.tests))

        for test in self.tests:
            string += (test.get_definition_str() + "\n")
        return string

    # Prompt the user to accept the tests being run
    def prompt(self):
        print("=" * 80)
        print("Running for sensor \"{}\" in mode:".format(self.module_spec["codelet"]))
        print(json.dumps(self.sensor_spec, indent=4))
        print("The following tests have been identified:")
        for index, test in enumerate(self.tests, start=1):
            print("{}. {}".format(index, test.name))

        input("Press ENTER to continue...")

    # Print the status of the tests that have been run so far
    def get_progress_str(self, color=True):
        progress_str = ""
        for index, test in enumerate(self.tests):
            progress_str += test.get_list_entry_str(color, index + 1)

        return progress_str

    # Run all of the tests in the test suite
    def run_tests(self):
        for test in self.tests:
            # Print test information and prompt user for confirmation
            start_string = "\nStarting test \"{}\"".format(test.name)
            print(colorize(start_string, "HEADER"))
            print(test.get_definition_str())
            test.run()

            # Print test results
            print(colorize("\nTest result", "HEADER"))
            print(test.get_result_str(True))
            input("Press ENTER to continue...\n")
            print(colorize("\nTest suite status", "HEADER"))
            print(self.get_progress_str())

    # Generate a full report from the tests that have been run
    def to_report(self):
        tests = {
            "module_spec": self.module_spec,
            "sensor_spec": self.sensor_spec,
            "sensor_drv_config": self.sensor_drv_config,
            "tests": [test.to_report() for test in self.tests if test.status != "NOT_RUN"]
        }

        return REPORT_FORMAT.format(sensor=self.module_spec["sensor_name"],
                                    version=self.module_spec["sw_version"],
                                    sensor_chksum="NA",
                                    isaac_chksum="NA",
                                    platform=get_platform_info("scripts/platform.sh",
                                                               "TARGET_PLATFORM"),
                                    jetpack=get_platform_info("scripts/platform.sh",
                                                              "JETPACK_VERSION"),
                                    date=str(datetime.now()),
                                    summary=self.print_summary(color=False),
                                    tests=json.dumps(tests, indent=4))

    # Generate a summarized report from the tests that have been run
    def print_summary(self, color=True):
        summary = ""
        total_tests = len(self.tests)
        passed_tests = len([test for test in self.tests if test.status == "PASS"])
        failed_tests = len([test for test in self.tests if test.status == "FAIL"])
        skipped_tests = len([test for test in self.tests if test.status == "SKIP"])
        failed_test_list = [test for test in self.tests if test.status == "FAIL"]

        if color:
            overall_status = colorize("PASS", "GOOD") if (
                failed_tests == 0 and skipped_tests == 0) else colorize("FAIL", "BAD")
        else:
            overall_status = "PASS" if (failed_tests == 0 and skipped_tests == 0) else "FAIL"
        summary += ("Overall status: {status}\n"
                    "\n"
                    "Passed:  {passed}/{total}\n"
                    "Failed:  {failed}/{total}\n"
                    "Skipped: {skipped}/{total}\n").format(status=overall_status,
                                                           passed=passed_tests,
                                                           failed=failed_tests,
                                                           skipped=skipped_tests,
                                                           total=total_tests)

        if failed_test_list:
            summary += "The following tests failed:\n"
            for test in failed_test_list:
                summary += " - {}\n".format(test.name)

        summary += "\n"
        summary += self.get_progress_str(color)
        return summary


# A depth sensor represents a sensor with a depth and color channel that have the same framerate
class DepthSensorTestRunner(TestRunner):
    def __init__(self, module_spec, sensor_spec, sensor_drv_config, report_dir, verbose):
        super().__init__(module_spec, sensor_spec, sensor_drv_config, report_dir, verbose)
        SCHEMA = {
            "type":
            "object",
            "properties": {
                "color_tx": {
                    "type": "string"
                },
                "color_intrinsics_tx": {
                    "type": "string"
                },
                "depth_tx": {
                    "type": "string"
                },
                "depth_intrinsics_tx": {
                    "type": "string"
                },
                "color_focal_length_x": {
                    "type": "number"
                },
                "color_focal_length_y": {
                    "type": "number"
                },
                "depth_focal_length_x": {
                    "type": "number"
                },
                "depth_focal_length_y": {
                    "type": "number"
                },
                "color_channels": {
                    "type": "number"
                },
                "fps": {
                    "type": "number"
                },
                "mode": {
                    "type": "string"
                },
                "min_depth": {
                    "type": "number"
                },
                "max_depth": {
                    "type": "number"
                }
            },
            "required": [
                "color_tx", "color_intrinsics_tx", "depth_tx", "depth_intrinsics_tx", "fps",
                "mode", "color_focal_length_x", "color_focal_length_y", "depth_focal_length_x",
                "depth_focal_length_y", "color_channels", "min_depth", "max_depth"
            ]
        }
        jsonschema.validate(instance=sensor_spec, schema=SCHEMA)
        self.sensor_spec = sensor_spec
        self.module_spec = module_spec
        self.sensor_drv_config = sensor_drv_config

    def get_test_types(self):
        return [
            DepthSanityTest, ColorSanityTest, ColorFrameRateTest, DepthFrameRateTest, MinDepthTest,
            MaxDepthTest, RangeDepthTest, ColorTest, ColorFrameRateStressTest,
            DepthFrameRateStressTest, AprilTagsTest
        ]


# Select the correct test based on the given sensor type
class TestRunnerFactory:
    TYPE_DICT = {"depth_camera": DepthSensorTestRunner}

    @staticmethod
    def create_from_json(module_spec, mode, report_dir, verbose):
        sensor_type = mode["type"]
        runner = TestRunnerFactory.TYPE_DICT[sensor_type]
        return runner(module_spec, mode["sensor_spec"], mode.get("sensor_drv_config", {}),
                      report_dir, verbose)
