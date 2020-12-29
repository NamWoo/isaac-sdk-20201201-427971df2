# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from packages.pyalice import Application
from packages.pyalice.bindings import set_severity, severity
from packages.pyalice.bindings import Status as Status
from textwrap import TextWrapper
from abc import ABCMeta, abstractmethod
from Utils import colorize, range_checker
from math import ceil
from texttable import Texttable
from enum import Enum
import getch
import os, sys
import time
import multiprocessing as mp
import logging


# A class describing a configurable tolerance value for a testcase
class Tolerance:
    def __init__(self, name, description, default, validate=range_checker(0, 100)):
        self.name = name
        self.description = description
        self.tolerance = default
        self.validate = validate

    def set(self, tolerance):
        tolerance = float(tolerance)
        self.validate(tolerance)
        self.tolerance = tolerance

    def get_definition_str(self):
        return "{} = {}: {}".format(self.name, self.tolerance, self.description)


# A class describing a testcase
class Test(metaclass=ABCMeta):
    def __init__(self):
        self.status = "NOT_RUN"
        self.tolerances = self.get_tolerances()

    # Prompt the user to enter custom values for all of the tolerances
    def override_tolerances(self):
        for tolerance in self.tolerances:
            print(colorize("\nOverriding tolerance " + tolerance.name, "HEADER"))
            print(tolerance.get_definition_str())
            valid = False
            while not valid:
                value = input("Enter new value: ")
                try:
                    tolerance.set(value)
                    valid = True
                except BaseException as e:
                    print("Error setting tolerance: ", str(e))

    # Run the test, allowing the user to override tolerances or skip the test if they choose. Saves
    # the new test status to self.status and any detailed test feedback from self.detailed_report
    def run(self):
        value = ""
        while value not in ['c', 'e', 's']:
            value = input("Press 'c' to run test, 'e' to enter tolerances or 's' to skip test:")
        if value == 'e':
            self.override_tolerances()
        if value == 's':
            self.status = "SKIP"
            self.detailed_report = "Test was skipped"
            return

        if self.execute():
            self.status = "PASS"
        else:
            self.status = "FAIL"

    # Prints this test's line in the test progress table
    def get_list_entry_str(self, color, num):
        status = "    "
        if self.status != "NOT_RUN":
            status = self.status
            if color:
                if status == "PASS":
                    status = colorize(status, "GOOD")
                if status == "FAIL":
                    status = colorize(status, "BAD")
                if status == "SKIP":
                    status = colorize(status, "WARN")

        return "[{status}] {num}. {name}\n".format(status=status, num=num, name=self.name)

    # Print the name, description, and tolerances of the test
    def get_definition_str(self):
        list_wrapper = TextWrapper(width=57, initial_indent=" " * 0, subsequent_indent=" " * 2)
        table = Texttable()
        table.add_rows([
            ["Name", self.name],
            ["Description", self.get_description()],
        ],
                       header=False)
        string = "NA"
        tolerances = self.tolerances
        if tolerances:
            string = ""
            for tolerance in tolerances:
                string += '\n'.join(list_wrapper.wrap("- " + tolerance.get_definition_str())) + '\n'
        table.add_row(["Error Tolerances", string])

        return table.draw() + "\n"

    # Print the detailed result of the test including status, tolerances that were overridden,
    # and detailed report data
    def get_result_str(self, use_color=False):
        wrapper = TextWrapper(width=80, initial_indent=" " * 4, subsequent_indent=" " * 6)

        if use_color:
            status_string = colorize(self.status, "GOOD") if self.status == "PASS" else colorize(
                self.status, "BAD")
        else:
            status_string = self.status

        string = "{} [{}]\n".format(self.name, status_string)
        tolerances = self.tolerances
        if tolerances:
            string += "Tolerances:\n"
            for tolerance in tolerances:
                string += '\n'.join(wrapper.wrap("- " + tolerance.get_definition_str())) + "\n"
        if self.detailed_report:
            string += "Detailed Report:\n"
            string += '\n'.join(['    ' + line for line in self.detailed_report.split('\n')])

        return string

    # Return the test results in json format
    def to_report(self):
        report = {
            "name": self.name,
            "status": self.status,
        }

        tolerances = self.tolerances
        if tolerances:
            report["tolerances"] = {tolerance.name: tolerance.tolerance for tolerance in tolerances}
        if self.detailed_report:
            report["detailed_report"] = self.detailed_report.split('\n')

        return report

    # Get the name of the child class. This is used as the test name.
    @property
    def name(self):
        return self.__class__.__name__

    # Execute the test, returning True for pass or False for failure
    # This function must also set the following parameters:
    #   self.detailed_report: (optional) A detailed report on the testcase execution
    @abstractmethod
    def execute(self):
        return

    # Return a string describing the test
    @staticmethod
    @abstractmethod
    def get_description():
        return

    # Return a list of Tolerances for the test that can be overidden
    @abstractmethod
    def get_tolerances(self):
        return


# A base class for a test that instantiates the DUT codelet and connects it to one
# evaluator codelet
class SimpleEvaluatorTest(Test, metaclass=ABCMeta):
    def __init__(self, module_spec, sensor_spec, sensor_drv_config, report_dir, verbose):
        super().__init__()
        self.sensor_spec = sensor_spec
        self.module_spec = module_spec
        self.sensor_drv_config = sensor_drv_config
        self.report_dir = report_dir + "/" + self.name
        self.verbose = verbose

    # Prompt the user to setup the test, by default there is no setup
    def do_setup(self, app):
        pass

    # Any extra app setup to be done before starting the app other than creating the DUT/evaluator
    def extra_app_setup(self, app):
        pass

    # Configuration to set on the isaac websight node
    def get_websight_config(self):
        return {
            "webroot": "packages/sight/webroot",
            "port": 3000,
        }

    # To protect against segmentation faults or other errors that may occur in isaac due to the DUT,
    # this create a child process and starts isaac in it. The actual implementation of test
    # execution is in execute_impl
    def execute(self):
        queue = mp.Queue()
        process = mp.Process(target=self.execute_process, args=(queue, sys.stdin.fileno()))
        process.start()

        while process.exitcode is None:
            time.sleep(1)
        if process.exitcode != 0:
            self.detailed_report = "ISAAC process terminated with non-zero error code {}".format(
                process.exitcode)
            return False
        else:
            result = queue.get(False)
            self.detailed_report = result[1]
            return result[0]

    def execute_process(self, queue, stdin):
        sys.stdin = os.fdopen(stdin)
        status = self.execute_impl()
        queue.put([status, self.detailed_report])

    def execute_impl(self):
        if not self.verbose:
            set_severity(severity.ERROR)

        app = Application(name="sensor_cert")

        if not self.verbose:
            app.logger.setLevel(logging.ERROR)

        # need to explicitly load websight node
        app.load_module("packages/sight")

        app.load_module("packages/viewers")

        for k, v in self.get_websight_config().items():
            app.nodes["websight"].components["WebsightServer"].config[k] = v

        app.load_module("packages/sensor_certification/evaluators")

        # Load design under test (dut)
        app.load_module(self.module_spec["module_name"])
        app.add("dut", [self.module_spec["codelet"]])

        # Load dut specific config
        component_name = self.module_spec["codelet"].split("::")[-1]
        for key, value in self.sensor_drv_config.items():
            app.nodes["dut"].components[component_name].config[key] = value

        # Load evaluator
        app.add("evaluator", [self.get_evaluator()])
        evaluator_name = self.get_evaluator().split("::")[-1]

        # Connect dut to evaluator
        for src, dest in self.get_links():
            app.connect(app.nodes["dut"].components[component_name], src,
                        app.nodes["evaluator"].components[evaluator_name], dest)

        # Configure evaluator config to match parameters
        eval_config = app.nodes["evaluator"].components[evaluator_name].config
        for key, value in self.get_evaluator_config().items():
            eval_config[key] = value

         # Add any extra setup to the app
        self.extra_app_setup(app)

        # Create report directory
        try:
            os.mkdir(self.report_dir)
        except FileExistsError:
            pass
        eval_config["report_directory"] = self.report_dir

        # Load additional evaluator config from the tolerances
        for tolerance in self.tolerances:
            config_name = self.get_tolerance_map()[tolerance.name]
            eval_config[config_name] = tolerance.tolerance

        try:
            app.start()
            self.do_setup(app)
            app.nodes["evaluator"].components[evaluator_name].config["setup_done"] = True
            status = app.wait_for_node("evaluator")
        finally:
            app.stop()

        self.detailed_report = app.nodes["evaluator"].components[evaluator_name].config["report"]

        if status == Status.Success:
            return True
        else:
            return False

    # Return the name of an isaac codelet to instantiate as the evaluator
    # The codelet will reportFailure to indicate the test failed or
    # reportSuccess to indicate the test passed
    @abstractmethod
    def get_evaluator(self):
        return

    # Return a list of (source, target) tuples indicating
    # which output channels form the design under test to connect to the
    # evaluator
    @abstractmethod
    def get_links(self):
        return

    # Return a dictionary of configs to set on the evaluator
    @abstractmethod
    def get_evaluator_config(self):
        return

    # return a map of tolerance names to config names
    @abstractmethod
    def get_tolerance_map(self):
        return


class FrameRateTest(SimpleEvaluatorTest):
    def __init__(self, *args):
        super().__init__(*args)

    def get_evaluator(self):
        return "isaac::evaluators::FrameRateEvaluator"

    def get_tolerances(self):
        return [
            Tolerance("FpsPercentError", ("The maximum allowed error in fps, calulated as 100 * "
                                          "abs(actual_fps - expected_fps) / (expected_fps)"), 5.00)
        ]

    def get_tolerance_map(self):
        return {"FpsPercentError": "fps_tolerance"}


class ColorFrameRateTest(FrameRateTest):
    def __init__(self, *args):
        super().__init__(*args)
        self.test_duration = 5.0

    def get_links(self):
        return [
            (self.sensor_spec["color_tx"], "color_listener")
        ]

    def get_evaluator_config(self):
        return {"target_fps": self.sensor_spec["fps"],
                "test_duration": self.test_duration, "tick_period": "5ms"}

    @staticmethod
    def get_description():
        return ("Capture frames from the color channel for 5s. Verify that "
                "the framerate is within the tolerance.")


class DepthFrameRateTest(FrameRateTest):
    def __init__(self, *args):
        super().__init__(*args)
        self.test_duration = 5.0

    def get_links(self):
        return [
            (self.sensor_spec["depth_tx"], "depth_listener")
        ]

    def get_evaluator_config(self):
        return {"target_fps": self.sensor_spec["fps"], "test_depth": True,
                "test_duration": self.test_duration, "tick_period": "5ms"}

    @staticmethod
    def get_description():
        return ("Capture frames from the depth channel for 5s. Verify that "
                "the framerate is within the tolerance.")


class ColorFrameRateStressTest(ColorFrameRateTest):
    def __init__(self, *args):
        super().__init__(*args)
        self.test_duration = 60.0

    def get_evaluator_config(self):
        return {"target_fps": self.sensor_spec["fps"],
                "test_duration": self.test_duration, "tick_period": "5ms"}

    @staticmethod
    def get_description():
        return ("A longer duration frame rate test. "
                "Captures frames from the color channel for 60s. Verifies that "
                "the framerate is within the tolerance.")


class DepthFrameRateStressTest(DepthFrameRateTest):
    def __init__(self, *args):
        super().__init__(*args)
        self.test_duration = 60.0

    def get_evaluator_config(self):
        return {"target_fps": self.sensor_spec["fps"], "test_depth": True,
                "test_duration": self.test_duration, "tick_period": "5ms"}

    @staticmethod
    def get_description():
        return ("A longer duration frame rate test. "
                "Captures frames from the depth channel for 60s. Verifies that "
                "the framerate is within the tolerance.")


class ColorSanityTest(SimpleEvaluatorTest):
    def __init__(self, *args):
        super().__init__(*args)

    def get_evaluator(self):
        return "isaac::evaluators::ColorSanityEvaluator"

    def get_links(self):
        return [(self.sensor_spec["color_tx"], "color_listener"),
                (self.sensor_spec["color_intrinsics_tx"], "color_intrinsics_listener")]

    def get_evaluator_config(self):
        split_mode = self.sensor_spec["mode"].split('x')
        cols = int(split_mode[0])
        rows = int(split_mode[1])
        channels = self.sensor_spec["color_channels"]
        return {
            "rows": rows,
            "cols": cols,
            "channels": channels,
            "num_frames": self.sensor_spec["fps"],
            "focal_x": self.sensor_spec["color_focal_length_x"],
            "focal_y": self.sensor_spec["color_focal_length_y"],
            "tick_period": "5ms"
        }

    @staticmethod
    def get_description():
        return ("Capture frames for 1s from the color channel. Verify they match the expected"
                " dimensions and colorspace.")

    def get_tolerances(self):
        return []

    def get_tolerance_map(self):
        return {}


class DepthSanityTest(SimpleEvaluatorTest):
    def __init__(self, *args):
        super().__init__(*args)

    def get_evaluator(self):
        return "isaac::evaluators::DepthSanityEvaluator"

    def get_links(self):
        return [(self.sensor_spec["depth_tx"], "depth_listener"),
                (self.sensor_spec["depth_intrinsics_tx"], "depth_intrinsics_listener")]

    def get_evaluator_config(self):
        split_mode = self.sensor_spec["mode"].split('x')
        cols = int(split_mode[0])
        rows = int(split_mode[1])
        return {
            "rows": rows,
            "cols": cols,
            "focal_x": self.sensor_spec["depth_focal_length_x"],
            "focal_y": self.sensor_spec["depth_focal_length_y"],
            "num_frames": self.sensor_spec["fps"],
            "tick_period": "5ms"
        }

    @staticmethod
    def get_description():
        return ("Capture frames for 1s from the depth channel. Verify they match the expected"
                " dimensions")

    def get_tolerances(self):
        return []

    def get_tolerance_map(self):
        return {}


class DepthTest(SimpleEvaluatorTest):
    def __init__(self, *args):
        super().__init__(*args)

    def do_setup(self, app):
        time.sleep(10)
        print(colorize("\nTest setup:", "HEADER"))
        # Get image dimensions
        split_mode = self.sensor_spec["mode"].split('x')
        cols = int(split_mode[0])
        rows = int(split_mode[1])

        # Get sensor focal length
        fx = self.sensor_spec["depth_focal_length_x"]
        fy = self.sensor_spec["depth_focal_length_y"]

        # How much of the image the target should consume
        width_ratio = 1 / 15
        height_ratio = 1 / 15

        # How big should the target appear in the camera (in pixels)
        target_height_pixels = int(rows * height_ratio)
        target_width_pixels = int(cols * width_ratio)

        # The target depth in meters
        depth = self.sensor_spec["max_depth"]

        # How big must the target be (in meters) to have the desired apparent size (in pixels)
        target_width = depth * target_width_pixels / fx
        target_height = depth * target_height_pixels / fy

        # Lego 2x4 block dimentions in meters
        lego_width = 0.032
        lego_height = 0.01

        # How many legos are needed to contruct the target?
        target_width_in_legos = int(max(ceil(target_width / lego_width), 2))
        target_height_in_legos = int(max(ceil(target_height / lego_height), 2))

        table = Texttable()
        table.add_rows(
            [["Target", "Description"], ["Target Choice", "Classic Lego brick 2x4"],
             [
                 "Target Size",
                 ("Based on FOV & sensor mode, a target of a minimum size of {width:.2f}cm wide by "
                  "{height:.2f}cm high is required for this testcase. Please see the documentation "
                  "on how to build a lego target that is {lego_w} bricks wide by {lego_h} bricks "
                  "high.").format(width=target_width * 100.0,
                                  height=target_height * 100.0,
                                  lego_w=target_width_in_legos,
                                  lego_h=target_height_in_legos)
             ],
             [
                 "Target Location", "Place the target in the field of view of the camera at a "
                 "distance of {:.3f}m".format(self.get_depth())
             ]])
        print(table.draw())

        print("\nPlease navigate to http://<Target_IP>:3000 in your web browser to see "
              "camera & depth viewers. Place the target in the line of sight of camera "
              "at a depth of {:.3f}m.".format(self.get_depth()))
        print("Adjust the position of the target while maintaining the target depth such "
              "that the green box (with the cross mark) on the depth viewer lands on the target.")
        print("Refer to 'Setup Test Scenes' section in the user guide for example.\n")

        target_cmd = ""
        print("Use 'w','a','s','d' to move target up, left, down, right or 'c' to run the test")
        while target_cmd not in ['c']:
            target_cmd = getch.getch()
            if target_cmd == 'w':
                app.nodes["evaluator"].components["DepthEvaluator"].config["target_cmd"] = 1
            elif target_cmd == 's':
                app.nodes["evaluator"].components["DepthEvaluator"].config["target_cmd"] = 2
            elif target_cmd == 'a':
                app.nodes["evaluator"].components["DepthEvaluator"].config["target_cmd"] = 3
            elif target_cmd == 'd':
                app.nodes["evaluator"].components["DepthEvaluator"].config["target_cmd"] = 4
            elif target_cmd == 'c':
                app.nodes["evaluator"].components["DepthEvaluator"].config["target_cmd"] = 5

    def get_websight_config(self):
        return {
            "webroot": "packages/sight/webroot",
            "port": 3000,
            "ui_config": {
                "windows": {
                    "Depth View: Line up target with green box": {
                        "renderer":
                        "2d",
                        "dims": {
                            "width": 640,
                            "height": 360
                        },
                        "channels": [{
                            "name": "sensor_cert/viewer/DepthCameraViewer/Depth"
                        }, {
                            "name": "sensor_cert/evaluator/DepthEvaluator/Region"
                        }, {
                            "name": "sensor_cert/evaluator/DepthEvaluator/Marker"
                        }],
                        "markers": [{
                            "name": "depthTarget"
                        }]
                    },
                    "Color View: Line up target with green box": {
                        "renderer":
                        "2d",
                        "dims": {
                            "width": 640,
                            "height": 360
                        },
                        "channels": [{
                            "name": "sensor_cert/viewer/ImageViewer/image"
                        }, {
                            "name": "sensor_cert/evaluator/DepthEvaluator/Region"
                        }, {
                            "name": "sensor_cert/evaluator/DepthEvaluator/Marker"
                        }],
                        "markers": [{
                            "name": "depthTarget"
                        }]
                    }
                }
            }
        }

    def extra_app_setup(self, app):
        component_name = self.module_spec["codelet"].split("::")[-1]
        app.add("viewer", ["isaac::viewers::DepthCameraViewer", "isaac::viewers::ImageViewer"])

        app.nodes["viewer"].components["DepthCameraViewer"].config["colormap"] = [[128, 0, 0],
                                                                                  [255, 0, 0],
                                                                                  [255, 255, 0],
                                                                                  [0, 255, 255],
                                                                                  [0, 0, 255],
                                                                                  [0, 0, 128]]
        app.nodes["viewer"].components["DepthCameraViewer"].config[
            "max_visualization_depth"] = 2 * self.sensor_spec["max_depth"]

        # Connect dut to viewer
        app.connect(app.nodes["dut"].components[component_name], self.sensor_spec["depth_tx"],
                    app.nodes["viewer"].components["DepthCameraViewer"], "depth")
        app.connect(app.nodes["dut"].components[component_name],
                    self.sensor_spec["depth_intrinsics_tx"],
                    app.nodes["viewer"].components["DepthCameraViewer"], "intrinsics")

        app.connect(app.nodes["dut"].components[component_name], self.sensor_spec["color_tx"],
                    app.nodes["viewer"].components["ImageViewer"], "image")
        app.connect(app.nodes["dut"].components[component_name],
                    self.sensor_spec["color_intrinsics_tx"],
                    app.nodes["viewer"].components["ImageViewer"], "intrinsics")

    def get_evaluator(self):
        return "isaac::evaluators::DepthEvaluator"

    def get_links(self):
        return [(self.sensor_spec["color_tx"], "color_listener"),
                (self.sensor_spec["depth_tx"], "depth_listener")]

    def get_evaluator_config(self):
        split_mode = self.sensor_spec["mode"].split('x')
        cols = int(split_mode[0])
        rows = int(split_mode[1])
        fx = self.sensor_spec["depth_focal_length_x"]
        fy = self.sensor_spec["depth_focal_length_y"]

        split_mode = self.sensor_spec["mode"].split('x')
        cols = int(split_mode[0])
        rows = int(split_mode[1])

        # How much of the image the target should consume
        width_ratio = 1 / 30
        height_ratio = 1 / 30

        # The target depth in meters
        depth = self.get_depth()

        target_height_pixels = int(rows * height_ratio)
        target_width_pixels = int(cols * width_ratio)

        return {
            "image_height": rows,
            "image_width": cols,
            "target_height": target_height_pixels,
            "target_width": target_width_pixels,
            "target_depth": depth,
            "frame_rate": self.sensor_spec["fps"],
            "tick_period": "5ms"
        }

    @abstractmethod
    def get_depth(self):
        pass

    @staticmethod
    def get_description():
        return ("A depth test evaluates the ability for the depth camera to detect a flat target "
                "at a fixed depth. The target is expected to take up at least 1/15th the width of "
                "the image and 1/15th the height of the image.\n\n"
                "The test captures 1s worth of frames and evaluates the average depth and "
                "coefficent of variation of pixels in the target region, which must be within "
                "the specified tolerances")

    def get_tolerances(self):
        return [
            Tolerance("AverageDepthPercentError",
                      ("The maximum allowed error in average depth, calculated as 100 * "
                       "abs(actual_depth - target_depth) / (target_depth)"), 10.00),
            Tolerance("MaxCoefficientOfVariation",
                      ("The maximum allowed coefficient of variation (standard deviation / mean) "
                       "of the target region. Setting this to 0.05 means most points in the "
                       "target region must be within 5 percent of the mean."), 0.05)
        ]

    def get_tolerance_map(self):
        return {
            "AverageDepthPercentError": "mean_tolerance",
            "MaxCoefficientOfVariation": "cov_tolerance"
        }

    def execute(self):
        while True:
            passed = super().execute()
            if passed:
                return passed
            print(self.detailed_report)
            print("Test failed")
            answer = ""
            while answer not in ["r", "c"]:
                answer = input("Press r to retry or c to continue: ")
            if answer == 'c':
                return passed


class MinDepthTest(DepthTest):
    def get_depth(self):
        return self.sensor_spec["min_depth"]

    @staticmethod
    def get_description():
        string = "Run a depth test for the minimum specified sensor depth.\n\n"
        string += super(MinDepthTest, MinDepthTest).get_description()
        return string


class MaxDepthTest(DepthTest):
    def get_depth(self):
        return self.sensor_spec["max_depth"]

    @staticmethod
    def get_description():
        string = "Runs a depth test for the maximum specified sensor depth.\n\n"
        string += super(MaxDepthTest, MaxDepthTest).get_description()
        return string


class RangeDepthTest(DepthTest):
    @staticmethod
    def get_description():
        string = ("Runs a series of depth tests for a range of depths between the minimum and "
                  "maximum specified sensor depth.\n\n")
        string += super(RangeDepthTest, RangeDepthTest).get_description()
        return string

    def prompt_depth_list(self):
        acknowledged = False
        while acknowledged != True:
            print("Current depths to try: " +
                  ', '.join([str(depth) + "m" for depth in self.depth_list]))
            value = input(
                "Press ENTER to accept these depths or enter a space separated list of depths: ")
            if value == "":
                acknowledged = True
            else:
                try:
                    depths = []
                    for item in value.split(' '):
                        depth = float(item)
                        if depth < self.sensor_spec["min_depth"] or depth > self.sensor_spec[
                                "max_depth"]:
                            raise ValueError("Depth {} is outside of range [{},{}]".format(
                                depth, self.sensor_spec["min_depth"],
                                self.sensor_spec["max_depth"]))
                        depths.append(depth)
                    if len(depths) == 0:
                        raise ValueError("Must enter at least one depth value")
                    self.depth_list = depths

                except BaseException as e:
                    print("Error with input depths: " + str(e))

    def execute(self):
        self.detailed_report = ""
        depth_range = self.sensor_spec["max_depth"] - self.sensor_spec["min_depth"]
        self.depth_list = [(1 / 4) * depth_range + self.sensor_spec["min_depth"],
                           (2 / 4) * depth_range + self.sensor_spec["min_depth"],
                           (3 / 4) * depth_range + self.sensor_spec["min_depth"]]
        self.prompt_depth_list()

        base_report_dir = self.report_dir
        report = ""
        total_status = True
        for depth in self.depth_list:
            print(colorize("Running range test for range {}m".format(depth), "HEADER"))
            input("Press ENTER to continue...")
            report += "Depth test at {}m =============\n".format(depth)
            self.current_depth = depth
            self.report_dir = base_report_dir + "_{}m".format(depth)
            status = super().execute()
            report += self.detailed_report
            report += "Result: {}\n".format("PASS" if status else "FAIL")

            print(colorize("Status for range test at {}m".format(depth), "HEADER"))
            print("Status: {}".format(
                colorize("PASS", "GOOD") if status else colorize("FAIL", "BAD")))
            print(self.detailed_report)

            total_status = total_status and status

        self.detailed_report = report
        return total_status

    def get_depth(self):
        return self.current_depth

class ColorTest(SimpleEvaluatorTest):
    class Color(Enum):
        RED = 0
        GREEN = 1
        BLUE = 2

    def __init__(self, *args):
        super().__init__(*args)
        self.color_list = [type(self).Color.RED, type(self).Color.GREEN, type(self).Color.BLUE]

    def do_setup(self, app):
        time.sleep(10)
        print(colorize("\nTest setup:", "HEADER"))

        split_mode = self.sensor_spec["mode"].split('x')
        cols = int(split_mode[0])
        rows = int(split_mode[1])

        # How much of the image the target should consume
        # as specified for the user
        width_ratio = 1 / 15
        height_ratio = 1 / 15

        target_height_pixels = int(rows * height_ratio)
        target_width_pixels = int(cols * width_ratio)

        # Get sensor focal length
        fx = self.sensor_spec["depth_focal_length_x"]
        fy = self.sensor_spec["depth_focal_length_y"]

        # Distance for color test
        depth = 0.5

        # Target size (in meters) to have the desired apparent size (in pixels)
        target_width = depth * target_width_pixels / fx
        target_height = depth * target_height_pixels / fy

        # Lego 2x4 block dimentions in meters
        lego_width = 0.032
        lego_height = 0.01

        target_width_in_legos = int(max(ceil(target_width / lego_width), 2))
        target_height_in_legos = int(max(ceil(target_height / lego_height), 2))

        table = Texttable()
        table.add_rows(
            [["Target", "Description"], ["Target Choice", "Classic Lego brick 2x4"],
             [
                 "Target Size",
                 ("Based on FOV & sensor mode, a solid {color} colored target of a minimum "
                  "size of {width:.2f}cm wide by {height:.2f}cm high is required for this "
                  "testcase. Please see the documentation on how to build a lego target "
                  "that is {lego_w} bricks wide by {lego_h} bricks "
                  "high.").format(color=self.current_color.name,
                                  width=target_width * 100.0,
                                  height=target_height * 100.0,
                                  lego_w=target_width_in_legos,
                                  lego_h=target_height_in_legos)
             ],
             [
                 "Target Location", "Place the target in the field of view of the camera at a "
                 "distance of {:.3f}m from the camera".format(depth)
             ]])
        print(table.draw())

        print("\nPlease navigate to http://<Target_IP>:3000 in your web browser to see "
              "the image viewer. Place the {} colored target in the line of sight of camera "
              "at a distance of {:.3f}m.".format(self.current_color.name, depth))
        print("Adjust the position of the lego target or the green box (with the cross mark) "
              "in the viewer while maintaining the lego target's distance such that on the image "
              "viewer the green box lands on "
              "the {} colored lego target.".format(self.current_color.name))
        print("Refer to 'Setup Test Scenes' section in the user guide for example.\n")

        target_cmd = ""
        print("Use 'w','a','s','d' to move green box up, left, down, right or 'c' to run the test")
        while target_cmd not in ['c']:
            target_cmd = getch.getch()
            if target_cmd == 'w':
                app.nodes["evaluator"].components["ColorEvaluator"].config["target_cmd"] = 1
            elif target_cmd == 's':
                app.nodes["evaluator"].components["ColorEvaluator"].config["target_cmd"] = 2
            elif target_cmd == 'a':
                app.nodes["evaluator"].components["ColorEvaluator"].config["target_cmd"] = 3
            elif target_cmd == 'd':
                app.nodes["evaluator"].components["ColorEvaluator"].config["target_cmd"] = 4
            elif target_cmd == 'c':
                app.nodes["evaluator"].components["ColorEvaluator"].config["target_cmd"] = 5

    def get_websight_config(self):
        return {
            "webroot": "packages/sight/webroot",
            "port": 3000,
            "ui_config": {
                "windows": {
                    "Color View: Line up specified color with green box": {
                        "renderer":
                        "2d",
                        "dims": {
                            "width": 640,
                            "height": 360
                        },
                        "channels": [{
                            "name": "sensor_cert/viewer/ImageViewer/image"
                        }, {
                            "name": "sensor_cert/evaluator/ColorEvaluator/Region"
                        }, {
                            "name": "sensor_cert/evaluator/ColorEvaluator/Marker"
                        }],
                        "markers": [{
                            "name": "colorTarget"
                        }]
                    }
                }
            }
        }

    def extra_app_setup(self, app):
        component_name = self.module_spec["codelet"].split("::")[-1]
        app.add("viewer", ["isaac::viewers::ImageViewer"])
        app.add("cv_viewer", ["isaac::viewers::ImageViewer"])

        # Connect dut to viewer
        app.connect(app.nodes["dut"].components[component_name], self.sensor_spec["color_tx"],
                    app.nodes["viewer"].components["ImageViewer"], "image")
        app.connect(app.nodes["dut"].components[component_name],
                    self.sensor_spec["color_intrinsics_tx"],
                    app.nodes["viewer"].components["ImageViewer"], "intrinsics")

    def get_evaluator(self):
        return "isaac::evaluators::ColorEvaluator"

    def get_links(self):
        return [(self.sensor_spec["color_tx"], "color_listener")]

    def get_evaluator_config(self):
        # Get image dimensions
        split_mode = self.sensor_spec["mode"].split('x')
        cols = int(split_mode[0])
        rows = int(split_mode[1])

        # How much of the image the target should actually consume
        # for the test to run
        width_ratio = 1 / 30
        height_ratio = 1 / 30

        # The color to run the test with
        color = self.current_color.value

        # How big should the target appear in the camera (in pixels)
        target_height_pixels = int(rows * height_ratio)
        target_width_pixels = int(cols * width_ratio)

        return {
            "image_height": rows,
            "image_width": cols,
            "target_height": target_height_pixels,
            "target_width": target_width_pixels,
            "target_color": color,
            "frame_rate": self.sensor_spec["fps"],
            "tick_period": "5ms"
        }

    @staticmethod
    def get_description():
        return ("Runs a series of color detection tests for the three specified "
                "lego colors (red, green, blue.)\n\n"
                "A color detection test evaluates the ability of a camera to "
                "perceive the correct colors. "
                "The target is expected to take up at least 1/15th of the width and height "
                "of the image. \n\n"
                "The test captures 1s worth of frames and evaluates the average RGB values "
                "within the specified boxed region of the target which must consist "
                "of expected values.")

    def get_tolerances(self):
        return []

    def get_tolerance_map(self):
        return {}

    def execute_single_color(self):
        while True:
            passed = super().execute()

            # If test passed then confirm result with user to prevent false-positives
            if passed:
                answer = ""
                while answer not in ["r", "c"]:
                    print(self.detailed_report)
                    answer = input("Is this detection correct?\n"
                                   "If correct press c to continue otherwise press r to retry: ")
                    if answer == 'c':
                        return passed
                continue

            print(self.detailed_report)
            print("Test failed")
            answer = ""
            while answer not in ["r", "c"]:
                answer = input("Press r to retry or c to continue: ")
            if answer == 'c':
                return passed

    def execute(self):
        base_report_dir = self.report_dir
        report = ""
        total_status = True

        for color in self.color_list:
            print(colorize("Running color test for {}".format(color.name), "HEADER"))
            input("Press ENTER to continue...")
            report += "Color test for {} =============\n".format(color.name)
            self.current_color = color
            self.report_dir = base_report_dir + "_{}".format(color.name)
            status = self.execute_single_color()
            report += self.detailed_report
            report += "Result: {}\n".format("PASS" if status else "FAIL")

            print(colorize("Status for color test at {}".format(color.name), "HEADER"))
            print("Status: {}\n".format(
                colorize("PASS", "GOOD") if status else colorize("FAIL", "BAD")))

            total_status = total_status and status

        self.detailed_report = report
        return total_status


# A SimpleEvaluatorTest class to run the april tags computer vision test. It overrides all
# required abstract functions to launch and connect the AprilTagsDetection codelet to the device
# under test. Then it defines the logic to prompt the user through the april tags test scene setup
# and in launching the AprilTagsEvaluator to verify the fiducial detections.
class AprilTagsTest(SimpleEvaluatorTest):
    def __init__(self, *args):
        super().__init__(*args)
        self.test_duration = 1.0

    def do_setup(self, app):
        time.sleep(10)

        # Scene setup instructions for the user
        print(colorize("\nTest setup:", "HEADER"))
        table = Texttable()
        table.add_rows(
            [["Target", "Description"], ["Target Choice", "April Tags Fiducial Marker"],
             [
                "Target Details",
                "A single april tag marker in the tag36h11 family is required for this testcase. "
                "The ID of the tag does not matter. PDF copies of tags of the correct family "
                "can be found at the link provided in the 'Test Scene Materials' section of the "
                "user guide and can be printed out and used in the test."
             ],
             [
                "Target Location", "Place the target in the target region in the field of "
                "view of the camera. The target region is shown as the area within the green box "
                "in the image viewer."
             ]])
        print(table.draw())
        print("\nPlease navigate to http://<Target_IP>:3000 in your web browser to see "
              "the image viewer. Place the april tag marker in the line of sight of camera "
              "within the target area specified by the green rectangle.")
        print("Adjust the position of the april tag marker "
              "in the viewer such that on the image viewer the marker is fully within the "
              "target area (within the green box). A green semi-transparent rectangle is "
              "overlaid on top of detected april tags and defines the detected tag's borders.\n")

        # Allow the user to confirm the scene is correctly setup
        setup_done = ""
        while setup_done not in ['c']:
            setup_done = input("Press 'c' to confirm completed scene setup and continue:")

    def get_websight_config(self):
        return {
            "webroot": "packages/sight/webroot",
            "port": 3000,
            "ui_config": {
                "windows": {
                    "Color View: Place April Tags marker within boxed target region": {
                        "renderer":
                        "2d",
                        "dims": {
                            "width": 640,
                            "height": 360
                        },
                        "channels": [{
                            "name": "sensor_cert/viewer/ImageViewer/image"
                        }, {
                            "name": "sensor_cert/evaluator/AprilTagsEvaluator/Region"
                        }, {
                            "name": "sensor_cert/fiducials_viewer/FiducialsViewer/fiducials"
                        }],
                        "markers": [{
                            "name": "AprilTagTarget"
                        }]
                    }
                }
            }
        }

    def extra_app_setup(self, app):
        component_name = self.module_spec["codelet"].split("::")[-1]

        # Add fiducial viewer and image viewer to app
        app.add("viewer", ["isaac::viewers::ImageViewer"])
        app.add("fiducials_viewer", ["isaac::viewers::FiducialsViewer"])

        # Add april tags detection codelet to app
        app.load_module("packages/fiducials:april_tags")
        app.add("april_tags_detection", ["isaac::fiducials::AprilTagsDetection"])

        # Connect dut to april tags detection codelet
        app.connect(app.nodes["dut"].components[component_name], self.sensor_spec["color_tx"],
                    app.nodes["april_tags_detection"].components["AprilTagsDetection"], "image")
        app.connect(app.nodes["dut"].components[component_name],
                    self.sensor_spec["color_intrinsics_tx"],
                    app.nodes["april_tags_detection"].components["AprilTagsDetection"], "intrinsics")

        # Connect april tags detection codelet to evaluator
        app.connect(app.nodes["april_tags_detection"].components["AprilTagsDetection"],
                              "april_tags",
                              app.nodes["evaluator"].components["AprilTagsEvaluator"],
                              "fiducials")

        # Connect april tags detection codelet to fiducial viewer
        app.connect(app.nodes["april_tags_detection"].components["AprilTagsDetection"],
                              "april_tags",
                              app.nodes["fiducials_viewer"].components["FiducialsViewer"],
                              "fiducials")

        # Connect dut to image viewer
        app.connect(app.nodes["dut"].components[component_name], self.sensor_spec["color_tx"],
                    app.nodes["viewer"].components["ImageViewer"], "image")
        app.connect(app.nodes["dut"].components[component_name],
                    self.sensor_spec["color_intrinsics_tx"],
                    app.nodes["viewer"].components["ImageViewer"], "intrinsics")

    def get_evaluator(self):
        return "isaac::evaluators::AprilTagsEvaluator"

    def get_links(self):
        return [(self.sensor_spec["color_tx"], "color")]

    def get_evaluator_config(self):
        return {
            "tick_period": "5ms",
            "test_timeout": int(self.test_duration * 4.0),
            "num_frames": int(self.test_duration * self.sensor_spec["fps"])
        }

    @staticmethod
    def get_description():
        return ("An April tag detection test evaluates the fidelity of the image "
                "being provided by a camera by passing its frames through "
                "an AprilTag detector and assessing whether detections are made correctly.\n\n"
                "The test captures 1s worth of frames and evaluates whether the "
                "aprilTag detections for each frame are found and within a target region.")

    def get_tolerances(self):
        return []

    def get_tolerance_map(self):
        return {}

    def execute(self):
        while True:
            passed = super().execute()
            if passed:
                return passed
            print(self.detailed_report)
            print("Test failed")
            answer = ""
            while answer not in ["r", "c"]:
                answer = input("Press r to retry or c to continue: ")
            if answer == 'c':
                return passed
