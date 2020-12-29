'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import copy
import os
from typing import List
from capnp.lib.capnp import _DynamicStructBuilder    # pylint: disable=no-name-in-module
from numpy import ndarray
from packages.pyalice import Message
from packages.pyalice.bindings import PybindMessage    # pylint: disable=no-name-in-module
from packages.pyalice.Component import Component

try:
    from packages.cask.pycask import PybindCask
except ImportError as error:
    pass

try:
    from packages_x86_64.cask.pycask import PybindCask
except ImportError as error:
    pass

try:
    from packages_jetpack44.cask.pycask import PybindCask
except ImportError as error:
    pass


class Cask(object):
    ''' Provides access to an ISAAC SDK log file '''
    def __init__(self, root, writable: bool = False):
        self.load(root, writable)

    def load(self, root, writable: bool):
        ''' Creates a new cask and loads an ISAAC SDK log file from the given location. '''
        if not os.path.isdir(root) and not writable:
            raise ValueError('Cask path does not exist: {}'.format(root))
        self._root = root
        self._cask = PybindCask(root, writable)
        self._writable = writable

        self._channels = dict()
        if not writable:
            # FIXME Maintains channel indices in write mode
            channels = self._cask.get_channels()
            for name, uuid in channels:
                if name in self._channels:
                    raise RuntimeError('Duplicate channel tag not supported yet')
                self._channels[name] = _CaskChannel(self._cask, name, uuid)

    def write_message(self, message):
        '''
        Writes proto as message with specified uuid and buffer into Cask

        Args:
            uuid (str): uuid in string of the message
            msg_proto (capnp._DynamicStructBuilder): capnp proto builder
            buffers (buffers: List[numpy.ndarray]): buffers of the message
        '''
        if message is None or not isinstance(message,
                                             (Message.MessageReader, Message.MessageBuilder)):
            # Checks for valid parameter of message
            return False
        if not self._writable:
            raise RuntimeError('Writing is not supported in read-only mode')
        if isinstance(message, Message.MessageReader):
            return self._cask.write_message(message._message)
        if isinstance(message, Message.MessageBuilder):
            return self._cask.write_message(message.uuid, message.proto.schema.node.id,
                                            message.proto.to_bytes(), message.buffers,
                                            message.acqtime, message.pubtime)
        return False

    def read_message(self, uuid: str):
        '''
        Reads a proto message with specified uuid from Cask

        Args:
            uuid (str): uuid in string of the message
        '''
        assert uuid is not None
        if self._writable:
            # FIXME Handles reading in write model
            raise NotImplementedError('Reading is to be implemented in writing mode')
        return Message.create_message_reader(self._cask.read_message(uuid))

    def open_channel(self, component: Component, tag: str):
        '''
        Grabs a helper instance for writing message to channel in cask

        Args:
            component (Component): Component from which the message would be published
            tag (str): Channel name on which the message would be published
        '''
        if not self.writable or component is None or tag is None or \
            not isinstance(component, Component) or not isinstance(tag, str):
            return None
        return _CaskChannelWriter(self, component, tag)

    @property
    def writable(self):
        ''' Returns true if the cask instance is for writing '''
        return self._writable

    @property
    def root(self):
        ''' The location of the opened log file '''
        return self._root

    @property
    def channels(self):
        ''' Lists all channels in the log '''
        return list(self._channels.values())

    def __getitem__(self, channel_name):
        ''' Gives the channel with the given name '''
        return self._channels[channel_name]


class _CaskChannel(object):
    ''' A channel in a cask contains a time series of messages '''
    def __init__(self, cask, name, uuid):
        ''' Loads the channel with given UUID from a cask '''
        self._cask = cask
        self._name = name
        self._messages = self._cask.get_channel_messages(uuid)
        self._index = 0

    @property
    def name(self):
        ''' The name of the channel '''
        return self._name

    def __str__(self):
        ''' Gives basic information about the channel '''
        return "{{name: {}, #: {}, tensor: {}}}".format(self.name, len(self._messages),
                                                        "yes" if self.tensors is not None else "no")

    def __len__(self):
        ''' The number of elements in the channel '''
        return len(self._messages)

    def __getitem__(self, index):
        '''
        Gets a message in the channel by its index.
        This function loads the message from disk into memory.
        '''
        _, uuid = self._messages[index]
        message = Message.create_message_reader(self._cask.read_message(uuid))
        assert message.uuid == uuid
        return message

    def __iter__(self):
        ''' Starts iteration through messages '''
        self._index = 0
        return self

    def __next__(self):
        ''' Gets the next message in the iteration '''
        if self._index >= self.__len__():
            raise StopIteration
        self._index += 1
        return self.__getitem__(self._index - 1)

    @property
    def tensors(self):
        ''' Creates an iterable sequence which gives message tensors as numpy arrays  '''
        if len(self) == 0 or self[0].tensor.size == 0:
            return None
        return _CaskChannelTensorIterator(self)

    @property
    def metadata(self) -> List:
        '''
        Returns list of [acqtime (int timestamp), uuid (str)] for all messages in the channel
        '''
        return copy.deepcopy(self._messages)


class _CaskChannelTensorIterator(object):
    ''' An iterable object which returns messages on a channel as numpy arrays '''
    def __init__(self, channel):
        ''' Creates an iterator for the given Channel '''
        self._channel = channel

    def __len__(self):
        ''' The number of numpy arrays in the channel '''
        return len(self._channel)

    def __getitem__(self, index):
        ''' Gets the numpy array for the message at the given index in the channel '''
        return self._channel[index].tensor

    def __iter__(self):
        ''' Starts iteration through tensors '''
        self._index = 0
        return self

    def __next__(self):
        ''' Gets the next tensor in the iteration '''
        if self._index >= self.__len__():
            raise StopIteration
        self._index += 1
        return self.__getitem__(self._index - 1)


class _CaskChannelWriter(object):
    ''' A Wrapper object to help writing messages to a channel in cask '''
    def __init__(self, cask: Cask, component: Component, tag: str):
        self._cask = cask
        self._component = component
        self._tag = tag

    def write_message(self, message) -> bool:
        ''' Writes data from MessageBuilder or MessageReader into cask '''
        if message is None or not (isinstance(message, Message.MessageBuilder)
                                   or isinstance(message, Message.MessageReader)):
            return False
        if isinstance(message, Message.MessageReader):
            return self._cask._cask.write_channel_message(self._component._component, self._tag,
                                                          message._message)
        if isinstance(message, Message.MessageBuilder):
            return self._cask._cask.write_channel_message(self._component._component, self._tag,
                                                          message.uuid,
                                                          message.proto.schema.node.id,
                                                          message.proto.to_bytes(), message.buffers,
                                                          message.acqtime, message.pubtime)
