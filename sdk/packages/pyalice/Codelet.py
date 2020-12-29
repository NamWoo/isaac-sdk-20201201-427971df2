'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import json
from .CodeletHooks import TxHook, RxHook
from .bindings import Status    # pylint: disable=no-name-in-module


class Codelet(object):
    """Python Codelet frontend object where the users will directly use the child class of the object
  and modifies start/tick/stop functions to suit their need

    Attributes:
        name (str): the name of the codelet. This will be modified to contain node that it belongs
                     to.
        backend (CodeletBackend): the backend object that should not be accessed by the user
        logger (python logger object): the logger enables us to monitor python codelets
  """
    def __init__(self):
        self.name = None
        self.backend = None    # CodeletBackend instance
        self.logger = None
        self.app = None    # Application Instance
        self._component = None    # Stub Component Instance

    @property
    def component(self):
        '''
        The Component.py instance holding stub C++ codelet instance
        (isaac::alice::PyCodelet).
        '''
        return self._component

    # Convenient functions for logging
    def log_info(self, msg):
        self.logger.info(msg, extra={"codeletname": self.name})

    def log_warning(self, msg):
        self.logger.warning(msg, extra={"codeletname": self.name})

    def log_debug(self, msg):
        self.logger.debug(msg, extra={"codeletname": self.name})

    def log_error(self, msg):
        self.logger.error(msg, extra={"codeletname": self.name})

    def log_critical(self, msg):
        self.logger.critical(msg, extra={"codeletname": self.name})

    def log_exception(self, msg):
        self.logger.exception(msg, extra={"codeletname": self.name})

    # Data accessors
    def isaac_proto_rx(self, proto_type, tag):
        ''' Adds proto rx message hook '''
        assert self.backend is not None, \
            "Fatal: backend has not been initialized ({})".format(self.name)
        hook = RxHook(proto_type, tag, self.backend.bridge)
        hook.app = self.app
        self.backend.bridge.add_rx_hook(hook.tag)
        return hook

    def isaac_proto_tx(self, proto_type, tag):
        ''' Adds proto tx message hook '''
        assert self.backend is not None, \
            "Fatal: backend has not been initialized ({})".format(self.name)
        hook = TxHook(proto_type, tag, self.backend.bridge)
        hook.app = self.app
        return hook

    # wrapper functions for ticking behaviours configuration in isaac codelet
    def tick_on_message(self, rx):
        self.backend.bridge.tick_on_message(rx.tag)

    def tick_blocking(self):
        self.backend.bridge.tick_blocking()

    def tick_periodically(self, interval: float):
        if not isinstance(interval, (int, float)):
            raise ValueError('Ticking interval has to be float or int for seconds')
        interval = float(interval)
        self.backend.bridge.tick_periodically(interval)

    def synchronize(self, *args):
        for rx1, rx2 in zip(args[:-1], args[1:]):
            assert isinstance(rx1, RxHook) and isinstance(rx2, RxHook), \
                "can not synchronize transmitting hook"
            self.backend.bridge.synchronize(rx1.tag, rx2.tag)

    # wrapper functions for utility functions in isaac codelet
    @property
    def tick_time(self) -> float:
        ''' Tick time (Isaac app clock) in seconds '''
        return self.backend.bridge.get_tick_time()

    @property
    def tick_dt(self) -> float:
        ''' Time duration between the start of the current and the previous tick in seconds '''
        return self.backend.bridge.get_tick_dt()

    @property
    def is_first_tick(self) -> bool:
        ''' True if this is the first tick of the codelet '''
        return self.backend.bridge.is_first_tick()

    @property
    def tick_count(self) -> int:
        ''' Tick Count starting from 1 '''
        return self.backend.bridge.get_tick_count()

    @property
    def config(self):
        ''' Config Accessor which gets data from bound C++ PyCodelet instance '''
        return self.component.config

    # allow publishing json dicts or messages to sight
    def _show(self, json_dict):
        if not isinstance(json_dict, dict):
            raise ValueError('Invalid datatype received : Expected a dictionary')
        if 'name' not in json_dict or 'type' not in json_dict:
            raise ValueError('Invalid Sight Json : Missing keys')
        json_str = json.dumps(json_dict)
        self.backend.bridge.show(json_str)

    # publish variables (names and values) to sight
    def show(self, name, value, time=None):
        json_dict = {}
        json_dict["name"] = name
        json_dict["v"] = value
        json_dict["type"] = "plot"
        if time is not None:
            json_dict["t"] = time
        else:
            json_dict["t"] = self.tick_time
        self._show(json_dict)

    def report_success(self, msg: str = ''):
        ''' Update the status for the component to success. Intended to work with Behavior '''
        if not isinstance(msg, str):
            return False
        return self.backend.bridge.update_status(Status.Success, msg)

    def report_failure(self, msg: str = ''):
        ''' Update the status for the component to failure. Intended to work with Behavior '''
        if not isinstance(msg, str):
            return False
        return self.backend.bridge.update_status(Status.Failure, msg)

    """ Functions below are to be overrided by the user
    """

    def start(self):
        pass

    def tick(self):
        pass

    def stop(self):
        pass
