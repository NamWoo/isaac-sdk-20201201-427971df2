'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import PIL.Image
import time
import math
import numpy

from isaac import *


class PyImageProducer(Codelet):
    def start(self):
        self.tx = self.isaac_proto_tx("ImageProto", "color")
        self.tick_periodically(0.1)
        self.log_info("pyimage start tick over")
        self.cnt = 0

        image_filename = self.config.color_filename
        self.log_info("color_filename {}".format(image_filename))
        self.image = numpy.array(PIL.Image.open(image_filename).convert(mode="RGB"))

    def tick(self):
        # get isaac parameters. Both ways are equivalent
        message = self.tx.init()

        message.buffers = [numpy.array(self.image)]
        message.proto.dataBufferIndex = 0

        shape = self.image.shape
        message.proto.rows = shape[0]
        message.proto.cols = shape[1]
        message.proto.channels = shape[2]

        message.proto.elementType = 'uint8'
        self.tx.publish()

        self.cnt += 1

    def stop(self):
        assert self.cnt > 0, "ticking count {}".format(self.cnt)


class PyImageReader(Codelet):
    def start(self):
        self.rx1 = self.isaac_proto_rx("ImageProto", "rgb_image_1")
        self.rx2 = self.isaac_proto_rx("ImageProto", "rgb_image_2")
        self.tick_on_message(self.rx1)
        self.cnt = 0

    def tick(self):
        img_msg1 = self.rx1.message
        img_msg2 = self.rx2.message
        if img_msg1 is None:
            return

        assert img_msg1.proto.rows == img_msg2.proto.rows, "Incorrect image rows {} {}".format(
            img_msg1.proto.rows, img_msg1.proto.rows)
        assert img_msg1.proto.cols == img_msg2.proto.cols, "Incorrect image cols {} {}".format(
            img_msg1.proto.cols, img_msg2.proto.cols)
        assert img_msg1.proto.channels == img_msg2.proto.channels, "Incorrect image cols {} {}".format(
            img_msg1.proto.channels, img_msg2.proto.channels)
        assert img_msg1.proto.elementType == img_msg2.proto.elementType, "Incorrect element type {} {}".format(
            img_msg1.proto.elementType, img_msg2.proto.elementType)

        buffer_data_1 = img_msg1.buffers[img_msg1.proto.dataBufferIndex]
        buffer_data_2 = img_msg1.buffers[img_msg1.proto.dataBufferIndex]

        expected_bytes_length = img_msg1.proto.rows * img_msg1.proto.rows * img_msg1.proto.channels
        assert len(buffer_data_1) == expected_bytes_length, "Incorrect buffer size {}".format(
            len(buffer_data_1))
        assert buffer_data_1 == buffer_data_2, "Inconsistent buffer data"
        self.cnt += 1

    def stop(self):
        assert self.cnt > 0, "ticking count {}".format(self.cnt)


def main():
    app = Application("packages/pyalice/tests/buffer_test.app.json")
    app.nodes['py_image'].add(PyImageProducer)
    app.nodes['py_reader'].add(PyImageReader)
    app.start_wait_stop(2.0)

    reader_instances = list(app._pycodelet_frontends.items())
    assert len(reader_instances) == 2

    read_cnt = 0
    reader = reader_instances[0][1]
    if isinstance(reader, PyImageReader):
        read_cnt = reader.cnt
    reader = reader_instances[1][1]
    if isinstance(reader, PyImageReader):
        read_cnt = reader.cnt
    assert read_cnt > 0


if __name__ == '__main__':
    main()
