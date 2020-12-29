'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import json
import uuid
from numpy import array, ndarray
from .CapnpMessages import get_capnp_proto_schemata, capnp_schema_type_id_dict
from capnp.lib.capnp import _DynamicStructBuilder    # pylint: disable=no-name-in-module
from .bindings import PybindMessage    # pylint: disable=no-name-in-module
from typing import List

CAPNP_DICT = get_capnp_proto_schemata()    # proto name to proto schema
CAPNP_TYPE_ID_DICT = capnp_schema_type_id_dict()    # type id to proto schema


class MessageReader(object):
    '''
    For accessing message data received via Isaac Python API
    '''
    def __init__(self):
        self._proto = None
        self._message = None

    @property
    def uuid(self) -> str:
        ''' Uuid of the message '''
        return self._message.get_uuid_string()

    @property
    def pubtime(self) -> int:
        ''' Publishing time stamp in nanoseconds '''
        return self._message.get_publish_time()

    @property
    def acqtime(self) -> int:
        ''' Acquisition time stamp in nanoseconds '''
        return self._message.get_acquisition_time()

    @property
    def type_id(self) -> int:
        ''' Capnp proto type id as unsigned long integer '''
        return self._message.get_type_id()

    @property
    def proto(self) -> bytes:
        ''' Capnp proto reader for message data access '''
        if self._proto is not None:
            return self._proto

        data = self._message.get_capn_proto_bytes()
        if data:
            self._proto = CAPNP_TYPE_ID_DICT[self.type_id].from_bytes(data)

        return self._proto

    @property
    def json(self):
        ''' The message data in json format '''
        data = self._message.get_json_string()
        if data:
            return json.loads(data)
        else:
            return None

    @property
    def buffers(self) -> List[memoryview]:
        ''' List of memory buffers from the message as memoryview(s) '''
        return [memoryview(x) for x in self._message.get_buffers()]

    @property
    def tensor(self) -> ndarray:
        '''
        Tensor associated with this message as a numpy array.
        If there are multiple or no tensors associated with this message, an empty array.
        '''
        return array(self._message.get_tensor())


class MessageBuilder(object):
    '''
    For composing message to be sent via Isaac Python API
    '''
    def __init__(self):
        super().__init__()
        self._proto = None
        self._acqtime = int(0)
        self._pubtime = int(0)
        self._buffers = []
        self._uuid = str(uuid.uuid4())

    @property
    def proto(self) -> _DynamicStructBuilder:
        ''' Message data as Capnp Builder '''
        return self._proto

    @proto.setter
    def proto(self, proto: _DynamicStructBuilder):
        assert isinstance(proto, _DynamicStructBuilder), \
            'Only capnp proto Builder is allowed. Try create one using Message.CAPNP_DICT'
        self._proto = proto

    @property
    def acqtime(self) -> int:
        ''' Acquisition time stamp in nanoseconds '''
        return self._acqtime

    @acqtime.setter
    def acqtime(self, acqtime: int):
        assert isinstance(acqtime, int), 'acqtime must be integer timestamp'
        if acqtime > 0:
            self._acqtime = acqtime

    @property
    def pubtime(self) -> int:
        ''' Publishing time stamp in nanoseconds '''
        return self._pubtime

    @pubtime.setter
    def pubtime(self, pubtime: int):
        assert isinstance(pubtime, int), 'pubtime must be integer timestamp'
        if pubtime > 0:
            self._pubtime = pubtime

    @property
    def buffers(self) -> List[ndarray]:
        ''' List of memory buffers for the message as numpy ndarray '''
        return self._buffers

    @buffers.setter
    def buffers(self, buffers: List[ndarray]):
        assert isinstance(buffers, list), 'buffers has to be list of ndarray'
        self._buffers = buffers

    @property
    def uuid(self) -> str:
        ''' Uuid of the message '''
        return self._uuid

    @uuid.setter
    def uuid(self, uuid: str):
        assert isinstance(uuid, str), 'uuid has to be string-form UUID'
        self._uuid = uuid


def create_message_reader(pybind_message: PybindMessage) -> MessageReader:
    '''
    Creates a wrapper for received message (Read-Only)
    '''
    if pybind_message is None:
        return None
    message = MessageReader()
    message._message = pybind_message
    return message


def create_message_builder(proto_name: str) -> MessageBuilder:
    '''
    Creates ProtoMessage for populating and publishing from specified proto name
    '''
    if proto_name is None:
        return None
    assert proto_name in CAPNP_DICT, 'Could not load specified message type of %r'\
        '. Is it mis-spelling or missing capnp file?' % proto_name
    msg = MessageBuilder()
    msg.proto = CAPNP_DICT[proto_name].new_message()
    return msg
