'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

from isaac import *


def create_app_and_pose_initializers(name_0, name_1, lhs, rhs):
    app = Application()
    pose_0 = app.add(name_0).add(app.registry.isaac.alice.PoseInitializer)
    pose_0.config.lhs_frame = lhs
    pose_0.config.rhs_frame = rhs
    pose_0.config.pose = [0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0]
    pose_0.config.report_success = True
    pose_1 = app.add(name_1).add(app.registry.isaac.alice.PoseInitializer)
    pose_1.config.lhs_frame = lhs
    pose_1.config.rhs_frame = rhs
    pose_1.config.pose = [0.0, 1.0, 0.0, 0.0, 2.0, 2.0, 2.0]
    pose_1.config.report_success = True
    return app


def main():
    app_file = 'packages/pyalice/tests/pynode_test_status.app.json'
    node_name = 'status'
    invalid_node_name = 'dummy'

    app = Application(app_file)
    app.start_wait_stop(0.2)
    # test invalid node
    invalid_node = app.nodes[invalid_node_name]
    assert invalid_node is None
    # test node valid and running
    node = app.nodes[node_name]
    assert node is not None and node.status == bindings.Status.Running

    # test node failure and application's more_jsons input with a single extra json
    more_jsons = 'packages/pyalice/tests/pynode_failure.json'
    app = Application(app_file, more_jsons)
    app.start_wait_stop(0.2)
    node = app.nodes[node_name]
    assert node is not None and node.status == bindings.Status.Failure

    # test "disable_automatic_start". This test is as same as the one above, except the status is
    # not failure, because the node is not started.
    more_jsons = 'packages/pyalice/tests/pynode_failure.json'
    app = Application(app_file, more_jsons)
    node = app.nodes[node_name]
    node.config["disable_automatic_start"] = True
    app.start_wait_stop(0.2)
    assert node is not None and node.status == bindings.Status.Running

    # test node success and application's more_jsons input with a single extra json
    more_jsons = 'packages/pyalice/tests/pynode_success.json'
    app = Application(app_file, more_jsons)
    app.start_wait_stop(0.2)
    node = app.nodes[node_name]
    assert node is not None and node.status == bindings.Status.Success

    # test node failure and application's more_jsons input with multiple comma-separated extra jsons
    more_jsons = 'packages/pyalice/tests/pynode_success.json,packages/pyalice/tests/pynode_failure.json'
    app = Application(app_file, more_jsons)
    app.start_wait_stop(0.2)
    node = app.nodes[node_name]
    assert node is not None and node.status == bindings.Status.Failure

    # test node success and application's more_jsons input with multiple comma-separated extra jsons
    more_jsons = 'packages/pyalice/tests/pynode_failure.json,packages/pyalice/tests/pynode_success.json'
    app = Application(app_file, more_jsons)
    app.start_wait_stop(0.2)
    node = app.nodes[node_name]
    assert node is not None and node.status == bindings.Status.Success

    # test start order
    LHS = "world"
    RHS = "target"
    NAME_0 = "pose_0"
    NAME_1 = "pose_1"

    app = create_app_and_pose_initializers(NAME_0, NAME_1, LHS, RHS)
    app.nodes[NAME_0].config["start_order"] = -100
    app.start_wait_stop(0.5)
    actual_pose = app.atlas.pose(LHS, RHS, 1.0)
    print(actual_pose)
    assert actual_pose[1][1] == 2.0

    app = create_app_and_pose_initializers(NAME_0, NAME_1, LHS, RHS)
    app.nodes[NAME_1].config["start_order"] = -100
    app.start_wait_stop(0.5)
    actual_pose = app.atlas.pose(LHS, RHS, 1.0)
    print(actual_pose)
    assert actual_pose[1][1] == 1.0

    # test component iterator
    app = Application(name='foo')
    bar_node = app.add('bar')
    bar_node.add(app.registry.isaac.alice.Random, 'bar1')
    bar_node.add(app.registry.isaac.alice.Random, 'bar2')

    bar_comp1 = None
    bar_comp2 = None
    for c in bar_node.components:
        if c.name == 'bar1':
            bar_comp1 = c
        if c.name == 'bar2':
            bar_comp2 = c
    assert bar_comp1 is not None
    assert bar_comp2 is not None


if __name__ == '__main__':
    main()
