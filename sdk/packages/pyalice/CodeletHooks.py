'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

# import all the cap'n'proto messages so that the pycodelet can decode them in backend
from .bindings import PybindPyCodelet    # pylint: disable=no-name-in-module
from .Message import *
from .CapnpMessages import get_capnp_proto_schemata
CAPNP_DICT = get_capnp_proto_schemata()


class RxHook(object):
    """
    Python message rx hook that mirrors the isaac message rx hook

    Args:
        proto_type (str): the name of the proto type. The script will find the proto in the pool
        tag (str): the channel/tag of the rx hook (same definition as isaac message rx hook)
        backend_receive (function): python function that calls pybind codelet to receive message
        backend_available (function): python function that calls pybind codelet to detect message

    Attributes:
        proto_schema (capnp struct object): the capnp object that is able to encode/decode message
        the rest of them are the same as above
    """
    def __init__(self, proto_type, tag, bridge):
        assert proto_type in CAPNP_DICT, \
          "proto message type \"{}\" not registered".format(proto_type)
        self.proto_schema = CAPNP_DICT[proto_type]
        self.tag = tag
        assert isinstance(bridge, PybindPyCodelet)
        self._bridge = bridge    # PybindPyCodelet
        self._node_name = self._bridge.get_node_name()
        assert self._node_name is not None and len(self._node_name) > 0
        self._component_name = self._bridge.get_component_name()
        assert self._component_name is not None and len(self._component_name) > 0
        self._msg = None    # Cache for message
        self.app = None

    @property
    def node_name(self):
        return self._node_name

    @property
    def component_name(self):
        return self._component_name

    @property
    def message(self):
        ''' Receives message. Returns message object or None '''
        if self.app is None:
            #return None
            raise RuntimeError('No Application connected')
        msg = self._bridge.get_message(self.tag)
        if msg is None:
            return None
        return create_message_reader(msg)


class TxHook(object):
    """
    Python message tx hook that mirrors the isaac message tx hook

    Args:
        proto_type (str): the name of the proto type. The script will find the proto in the pool
        tag (str): the channel/tag of the tx hook (same definition as isaac message tx hook)
        backend_publish (function): python function that calls pybind codelet to publish message

    Attributes:
        proto_schema (capnp struct object): the capnp object that is able to encode/decode message
        the rest of them are the same as above
    """
    def __init__(self, proto_type, tag, bridge):
        assert proto_type in CAPNP_DICT, \
          "proto message type \"{}\" not registered".format(proto_type)
        self.proto_schema = CAPNP_DICT[proto_type]
        self.tag = tag
        self._bridge = bridge    # PybindPyCodelet
        self._node_name = self._bridge.get_node_name()
        assert self._node_name is not None and len(self._node_name) > 0
        self._component_name = self._bridge.get_component_name()
        assert self._component_name is not None and len(self._component_name) > 0

        self.acqtime_cache = None
        self.pubtime_cache = None
        self.uuid_cache = None
        self.message_cache = None
        self.app = None
        self._proto_type = proto_type
        self._msg = None

    @property
    def msg(self):
        return self._msg

    def init(self):
        '''
        Creates a message to be published
        '''
        self._msg = create_message_builder(self._proto_type)
        return self._msg

    def publish(self):
        '''
        Publishes cached message to the channel
        '''
        if self.app is None or self._msg is None:
            print('None app or msg')
            return False
        self.app.publish(self._node_name, self._component_name, self.tag, self._msg)
        self._msg = None
