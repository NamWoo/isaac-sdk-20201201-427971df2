'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import time
import math
import queue

from isaac import *


def create_capnp_SO2_from_angle(angle, rotation_msg):
    angle = math.radians(angle)
    rotation_msg.q.x = math.cos(angle)
    rotation_msg.q.y = math.sin(angle)


def get_angle_from_capnp_SO2(rotation_msg):
    return math.degrees(math.atan2(rotation_msg.q.y, rotation_msg.q.x))


class MyPyCodeletProducer(Codelet):
    def start(self):
        self.tx = self.isaac_proto_tx("Pose2dProto", "pose")
        self.tick_periodically(0.1)
        self.cnt = 0
        self.config.x = 3.0
        self.config.y = -2.1

    def tick(self):
        proto_msg = self.tx.init()
        create_capnp_SO2_from_angle(60.0, proto_msg.proto.rotation)
        proto_msg.proto.translation.x = self.config.x
        proto_msg.proto.translation.y = self.config.y

        self.tx.publish()

        # test logger and tick utils
        tick_dt = self.tick_dt
        is_first_tick = self.is_first_tick

        desired_tick_dt = 0.1 if self.cnt != 0 else 0.0
        desired_is_first_tick = True if self.cnt == 0 else False
        desired_get_tick_count = self.cnt + 1

        self.log_info("tick time: {} (dt = {}), first tick: {}, ticks #{}".format(
            self.tick_time, tick_dt, is_first_tick, self.tick_count))
        epsilon = 0.02
        assert abs(tick_dt - desired_tick_dt) < epsilon or self.cnt == 0,\
            "Tick DT differ too much at tick #{} {} {}".format(
                self.cnt, tick_dt, desired_tick_dt)
        assert is_first_tick == desired_is_first_tick, "is_first_tick fails to produce correct result"
        assert self.tick_count == desired_get_tick_count, "tick_count fails"

        self.cnt += 1

    def stop(self):
        assert self.cnt > 5, \
              "messaging system is unexpectedly slow (target: {}, actual: {})".format(5, self.cnt)


class MyPyCodeletConsumer(Codelet):
    def start(self):
        self.rx = self.isaac_proto_rx("Pose2dProto", "pose")
        self.tick_on_message(self.rx)
        self.cnt = 0

    def tick(self):
        self.cnt += 1
        message = self.rx.message
        angle = get_angle_from_capnp_SO2(message.proto.rotation)
        self.log_info("pubtime: {}, acqtime: {}, uuid: {}".format(message.pubtime, message.acqtime,
                                                                  message.uuid))
        assert abs(angle - 60.0) < 1e-14, \
          "angle not matched; expected: {:.15f}, actual: {:.6f}".format(60, angle)
        assert abs(message.proto.translation.x - 3.0) < 1e-14, \
          "translation(x) not matched; expected: {:.6f}, actual: {:.6f}".format(3.0,
            message.proto.translation.x)
        assert abs(message.proto.translation.y + 2.1) < 1e-14, \
          "translation(x) not matched; expected: {:.6f}, actual: {:.6f}".format(-2.1,
            message.proto.translation.y)

    def stop(self):
        assert self.cnt > 5, \
              "messaging system is unexpectedly slow (target: {}, actual: {})".format(5, self.cnt)


class MyPyCodeletProducerDouble(Codelet):
    def start(self):
        self.tx = self.isaac_proto_tx("Pose2dProto", "pose")
        self.tx2 = self.isaac_proto_tx("Pose2dProto", "pose2")
        self.tick_periodically(0.1)

    def tick(self):
        app_timestamp = int(self.tx.app.clock.time * 1e9)

        message1 = self.tx.init()
        create_capnp_SO2_from_angle(60.0, message1.proto.rotation)
        message1.proto.translation.x = 3.0
        message1.proto.translation.y = -2.1
        message1.acqtime = app_timestamp
        self.tx.publish()

        message2 = self.tx2.init()
        create_capnp_SO2_from_angle(60.0, message2.proto.rotation)
        message2.proto.translation.x = 3.0
        message2.proto.translation.y = -2.1
        message2.acqtime = app_timestamp
        self.tx2.publish()


class MyPyCodeletConsumerDouble(Codelet):
    def start(self):
        self.rx = self.isaac_proto_rx("UuidProto", "pose")
        self.rx2 = self.isaac_proto_rx("UuidProto", "pose2")
        self.tick_on_message(self.rx)
        self.tick_on_message(self.rx2)
        self.synchronize(self.rx, self.rx2)
        self.cnt = 0

    def tick(self):
        self.cnt += 1
        assert self.rx.message.acqtime == self.rx2.message.acqtime
        self.log_info("rx - pubtime: {}, acqtime: {}, uuid: {}".format(
            self.rx.message.pubtime, self.rx.message.acqtime, self.rx.message.uuid))
        self.log_info("rx2 - pubtime: {}, acqtime: {}, uuid: {}".format(
            self.rx2.message.pubtime, self.rx2.message.acqtime, self.rx2.message.uuid))

    def stop(self):
        assert self.cnt > 5, \
              "messaging system is unexpectedly slow (target: {}, actual: {})".format(5, self.cnt)


class PyCodeletImageReader(Codelet):
    def start(self):
        self.rx = self.isaac_proto_rx("ImageProto", "rgb_image")
        self.tick_on_message(self.rx)
        self.cnt = 0

    def tick(self):
        msg = self.rx.message

        assert msg.proto.rows == 4096, "Incorrect image rows {}".format(msg.proto.rows)
        assert msg.proto.cols == 4096, "Incorrect image cols {}".format(msg.proto.cols)
        buffer_data = msg.buffers[msg.proto.dataBufferIndex]
        assert len(buffer_data) == 4096 * 4096 * 3, "Incorrect image bytes length {}".format(
            len(buffer_data))

        self.log_critical('Ticking {}'.format(self.cnt))
        self.cnt += 1

    def stop(self):
        assert self.cnt > 0, "ticking count {}".format(self.cnt)


class PyCodeletTickInteger(Codelet):
    def start(self):
        # Makes sure tick_periodically takes int argument
        self.tick_periodically(2)


def main():
    # PyCodlet => PyCodelet
    app = Application("packages/pyalice/tests/pycodelet_test_py2py.app.json")
    app.nodes['py_producer'].add(MyPyCodeletProducer)
    app.nodes['py_consumer'].add(MyPyCodeletConsumer)
    app.start_wait_stop(1.0)

    # C++ Codelet => PyCodelet
    app = Application("packages/pyalice/tests/pycodelet_test_cpp2py.app.json")
    app.nodes['py_consumer'].add(MyPyCodeletConsumer)
    app.start_wait_stop(1.0)

    # PyCodelet => C++ Codelet
    app = Application("packages/pyalice/tests/pycodelet_test_py2cpp.app.json")
    app.nodes['py_producer'].add(MyPyCodeletProducer)
    app.start_wait_stop(1.0)

    # Synchronized Tick
    app = Application("packages/pyalice/tests/pycodelet_test_sync.app.json")
    app.nodes['py_producer'].add(MyPyCodeletProducerDouble)
    app.nodes['py_consumer'].add(MyPyCodeletConsumerDouble)
    app.start_wait_stop(1.0)

    # Tests passing image buffer with message
    app = Application("packages/pyalice/tests/pycodelet_test_img.app.json")
    # Adds PyCodelet instance as needed and connect via Python API
    app.nodes['py_reader'].add(PyCodeletImageReader)
    app.connect('cpp_image/isaac.message_generators.ImageLoader', 'color', 'py_reader/PyCodelet',
                'rgb_image')
    node_int = app.add('inttick')
    node_int.add(PyCodeletTickInteger)
    app.start_wait_stop(5.0)
    reader_instances = list(app._pycodelet_frontends.items())
    assert len(reader_instances) > 0
    reader = reader_instances[0][1]

    assert isinstance(reader, PyCodeletImageReader)
    assert reader.cnt > 0

    # Quits in case not started
    app = Application()
    foo_node = app.add('foo')
    foo_node.config['disable_automatic_start'] = True
    foo_node.add(PyCodeletTickInteger)
    app.run(1.0)


if __name__ == '__main__':
    main()
