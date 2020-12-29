'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from isaac import Application

# Main part that sets up the app's logic and starts it afterwards.
if __name__ == '__main__':
    app = Application(app_filename='packages/path_planner/apps/pose2_graph_builder.app.json')
    app.load("packages/multi_robot_fof/pose2_graph_builder.config.json")

    # Start the application.
    app.run()
