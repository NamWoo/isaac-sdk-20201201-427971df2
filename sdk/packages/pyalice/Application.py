'''
Copyright (c) 2018-2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from typing import List
import argparse
import copy
import glob
import json
import logging
import os
import queue
import time
import traceback
import types
import uuid
import sys

import numpy
import quaternion

from . import Message, Component
from .bindings import PybindApplication    # pylint: disable=no-name-in-module
from .bindings import PybindClock    # pylint: disable=no-name-in-module
from .bindings import PybindAtlas    # pylint: disable=no-name-in-module
from .bindings import PybindPyCodelet    # pylint: disable=no-name-in-module
from .bindings import PybindMessage    # pylint: disable=no-name-in-module
from .Codelet import Codelet
from .bindings import Status
from .CodeletBackend import CodeletBackend
from .CodeletFlowControl import CodeletFlowControl
from .module_explorer import ModuleExplorer
from .Node import Node


class _NodesAccess(object):
    '''
    This class provides easy access to nodes of managed application
    '''
    def __init__(self, app):
        """
        Creates a nodes accessor

        Args:
            node (Node): The node to access
        """
        if app is None:
            raise ValueError('app must not be None')
        self._app = app
        self._names = app._app.get_all_node_names()

    def __getitem__(self, key: str):
        '''
        Provides "nodes['key']" read access of nodes in managed app
        '''
        if isinstance(key, str):
            app = self._app
            handle = app._app.find_node_by_name(key)
            if handle is None or handle.is_null():
                return None
            else:
                return Node(app, handle)
        return None

    def __len__(self):
        return len(self._names)

    def __iter__(self):
        self._index = 0
        return self

    def __next__(self):
        names = self._names
        if self._index >= len(names):
            raise StopIteration
        name = names[self._index]
        self._index += 1
        return self[name]


class _ClockAccess(object):
    '''
    This class provides easy access to isaac app clock
    '''
    def __init__(self, app):
        if app is None:
            raise ValueError('app must not be None')
        self._clock = PybindClock(app._app)

    @property
    def time(self) -> float:
        '''
        Gets the isaac application time in seconds. Returns None in case of error.
        '''
        result = self._clock.get_time()
        if result < 0.0:
            return None
        return result


class _AtlasAccessor(object):
    '''
    This class provides easy access to isaac app pose tree
    '''
    def __init__(self, app):
        if app is None:
            raise ValueError('app must not be None')
        self._pose_tree = PybindAtlas(app._app)

    def pose(self, lhs: str, rhs: str, app_time: float):
        '''
        Gets (interpolated) pose of lhs_T_rhs at specified app time (in seconds)

        Args:
            lhs (str): left-hand-side frame name
            rhs (str): right-hand-side frame name
            app_time (float): Isaac app time in seconds
        '''
        result = self._pose_tree.get_pose(lhs, rhs, app_time)
        if result is None:
            return None
        return [
            numpy.quaternion(result[0], result[1], result[2], result[3]),
            numpy.array([result[4], result[5], result[6]])
        ]

    def set_pose(self, lhs: str, rhs: str, app_time: float, pose: List[object]) -> bool:
        '''
        Provides pose of lhs_T_rhs at specified app time (in seconds) to Isaac app pose tree.

        Args:
            lhs (str): left-hand-side frame name
            rhs (str): right-hand-side frame name
            app_time (float): Isaac app time in seconds
            pose (List): tuple of (numpy.quaternion, numpy.array(3)) for rotation and translation
                accordingly.
        '''
        if len(pose) != 2 or \
            not isinstance(pose[0], numpy.quaternion) or\
            not isinstance(pose[1], numpy.ndarray):
            raise ValueError('Invalid parameter for setting pose')
        q = pose[0]
        t = pose[1]
        pose_list = [q.w, q.x, q.y, q.z, t[0], t[1], t[2]]
        return self._pose_tree.set_pose(lhs, rhs, app_time, pose_list)


class Application(object):
    """
    Central application similar to the C++ alice::Application
    """
    def __init__(self,
                 app_filename: str = None,
                 more_jsons: str = "",
                 name=None,
                 modules=None,
                 argv=sys.argv):
        """
        Creates an Isaac application

        Either a full app JSON file can be given using `app_filename`, or `name` and `modules` can
        be used to start an empty application.

        Args:
            app_filename (str): the main application json filename
            more_jsons (str): a comma-separated string of additional jsons to load for the app
            name (str): the name of the application
            modules (List(str)): a list of modules to be loaded
            argv: Command line arguments from sys.argv
        """
        # Use the current working directory.
        self._asset_path = os.getcwd()

        # Parse command line args
        parser = argparse.ArgumentParser(description=' simulator for navigation')
        parser.add_argument('--max_duration',
                            dest='max_duration',
                            type=str,
                            help='Maximum running time in seconds/minutes/hours [s|m|h]')
        parser.add_argument('--performance_report_out',
                            dest='performance_report_out',
                            type=str,
                            help='The path to write performance report to.')
        self._args, _ = parser.parse_known_args(argv)
        if not hasattr(self._args, 'max_duration'):
            self._args.max_duration = None
        if self._args.max_duration is not None:
            if not isinstance(self._args.max_duration, str) or not len(self._args.max_duration) > 1:
                raise ValueError('max_duration must be string of format <number>[s|m|h]')
            duration_unit = self._args.max_duration[-1]
            self._args.max_duration = float(self._args.max_duration[:-1])
            if duration_unit == 'm':
                self._args.max_duration = self._args.max_duration * 60
            elif duration_unit == 'h':
                self._args.max_duration = self._args.max_duration * 60 * 60
            else:
                assert duration_unit == 's', "max_duration has to come with unit [s|m|h]"

        if app_filename is not None:
            assert name is None
            assert modules is None
            app_json_str = ''
            with open(app_filename, 'r') as f:
                app_json_str = f.read()
            self._app = PybindApplication(app_json_str, more_jsons, self._asset_path)
        else:
            if name is None:
                name = str(uuid.uuid4())
            if modules is None:
                modules = []
            json_loader = {"name": name, "modules": modules}
            if hasattr(self._args, 'performance_report_out')\
                and self._args.performance_report_out is not None:
                json_loader['performance_report_out'] = self._args.performance_report_out
            self._app = PybindApplication(json.dumps(json_loader), more_jsons, self._asset_path)

        # Accessors
        self._clock = _ClockAccess(self)
        self._pose_tree = _AtlasAccessor(self)

        # Error Queue
        self._queue = queue.Queue()

        # Isaac only supports start once and stop once or not start at all. Track it.
        # None for newly created, 1 for started, 0 for stopped.
        self._start_status = None

        # TODO - use ISAAC SDK logger
        FORMAT = '%(asctime)-15s %(levelname)s %(message)s'
        logging.basicConfig(format=FORMAT)
        self.logger = logging.getLogger('pycodelet')
        self.logger.setLevel(logging.DEBUG)
        self.default_logger_config = {"codeletname": "main"}

        # compile all pycodelet backends
        self._pycodelet_backends = {}
        self._pycodelet_frontends = {}

    @property
    def nodes(self):
        ''' Returns the node accessor '''
        return _NodesAccess(self)

    @property
    def registry(self):
        ''' Returns a component registry object which can be used to discover components '''
        return ModuleExplorer(self._app.get_loaded_components())

    def _bind_pycodelet(self, codelet_class, component: Component):
        ''' Binds a python codelet class to cpp stub component (isaac.alice.PyCodelet) '''
        if component is None or codelet_class is None:
            raise ValueError('Invalid Arguments')

        full_name = '{}/{}'.format(component.node.name, component.name)
        self.logger.debug("Binding PyCodelet {}".format(full_name))

        # Creates instance of Python Codelet
        assert issubclass(codelet_class, Codelet)
        frontend = codelet_class()
        frontend.name = full_name
        frontend._component = component
        frontend.logger = self.logger
        frontend.app = self
        self._pycodelet_frontends[component] = frontend

    def _start_pycodelets(self):
        ''' Creates and starts Python threads (CodeletBackend) supporting Python codelets '''
        for component, frontend in self._pycodelet_frontends.items():
            # Creates Python Threads
            backend = CodeletBackend()
            # Creates bridge instance connecting C++ and Python code
            bridge = PybindPyCodelet(component.handle)
            # Wires references
            frontend.backend = backend
            backend.frontend = frontend
            backend.bridge = bridge
            # Creates instance handling execution sequence
            backend.flow_controller = CodeletFlowControl(backend, bridge)
            # Wires Queue for gathering exception information
            backend.flow_controller.queue = self._queue

            self._pycodelet_backends[component] = backend
            backend.start()

    def start(self):
        ''' Starts the application '''
        if self._start_status is not None:
            raise RuntimeError('Isaac App can not be started twice')
        self._start_status = int(1)

        self.logger.debug("Launching isaac core", extra=self.default_logger_config)
        self._app.start()
        self.logger.debug("Launching pycodelet threads", extra=self.default_logger_config)

        self._start_pycodelets()

    def stop(self, request_stop=True):
        ''' Stops the application

        Args:
            request_stop (bool): If True, this function will request the application to stop using
            the _app.stop() function, otherwise it's assumed the stop function already got called
            and this function will only clear itself.
        '''
        if self._start_status is None:
            # Not-started app does not need to be stopped
            return
        if self._start_status == int(0):
            raise RuntimeError('Isaac App shall not be stopped more than once')
        self._start_status = int(0)

        if request_stop:
            self._app.stop()

        for _, backend in self._pycodelet_backends.items():
            backend.py_exit_flag = True
            backend.join()
        self.logger.debug("Python Codelets All stopped...", extra=self.default_logger_config)

        if self._queue.empty():
            return
        # Prints out all errors and raises
        while not self._queue.empty():
            self.logger.critical(str(self._queue.get()))
        raise RuntimeError('Exceptions happened on PyCodelet Thread(s)')

    def add(self, name: str, components: List[str] = None, create_message_ledger=True):
        '''
        Adds a new node to the app

        Args:
            name (str): The name of the node. The name must be unique within the application.
            components (List(str)): A list of component types which will be added to the node.
            create_message_ledger (bool): If disabled a message ledger will not be added
                automatically
        '''
        node = Node(self, self._app.create_node(name))
        if components is None:
            components = []
        if create_message_ledger:
            components.insert(0, "isaac::alice::MessageLedger")
        for component_type in components:
            node.add(component_type)
        return node

    def connect(self, tx, tx_tag: str, rx, rx_tag: str):
        '''
        Connects message channels

        Args:
            tx (Component): The component which transmits messages
            tx_tag (str): The channel on the component `tx` which transmits messages
            rx (Component): The component which receives messages
            rx_tag (str): The channel on the component `rx` which receives messages
        '''
        if isinstance(tx, str) and isinstance(rx, str):
            self._app.connect('{}/{}'.format(tx, tx_tag), '{}/{}'.format(rx, rx_tag))
            return
        if isinstance(tx, Component.Component) and isinstance(rx, Component.Component):
            self._app.connect(tx.handle, tx_tag, rx.handle, rx_tag)
            return
        raise ValueError('tx and rx have to be both instances of str or both Component')

    def start_wait_stop(self, duration: float = None):
        '''
        Starts the application waits for the given duration and stops the application. If duration
        is not given the application will run forever or until Ctrl+C is pressed in the console.
        '''
        self.start()
        try:
            if duration is None:
                while True:
                    time.sleep(1.0)
            else:
                time.sleep(duration)
        except:
            traceback.print_exc()
        self.stop(True)

    def wait_for_node(self, node: str, duration: float = None, poll_interval=0.1):
        '''
        Waits for the node to stop running or for the duration to lapse.
        If duration is not given the application will run forever or until
        Ctrl+C is pressed in the console.]

        Args:
            node (str): the name of the node to wait to stop running
            duration (float): timeout in seconds, can run forever, if duration is not specified
            poll_interval (float): node status polling interval in seconds

        Returns: node status reached.
        '''

        running_node = self.find_node_by_name(node)
        end_time = self.clock.time + duration if duration is not None else None

        while True:
            status = running_node.get_status()
            if status != Status.Running:
                break

            time_remaining = end_time - self.clock.time if duration is not None else poll_interval
            if time_remaining < 0.0:
                self.logger.debug("Python Codelets, Timeout.", extra=self.default_logger_config)
                break
            else:
                time.sleep(min(time_remaining, poll_interval))

        return status

    @property
    def uuid(self):
        ''' Returns the UUID of the application as a stringl '''
        return self._app.uuid()

    def find_node_by_name(self, name: str):
        ''' Finds the node with the given name in the application '''
        return self._app.find_node_by_name(name)

    def receive(self, node: str, component: str, channel: str) -> Message.MessageReader:
        '''
        Receive the latest message from the supplied node/component/channel. If no new message is
        available, None will be returned.

        Args:
            node (str): name of node
            component (str): name of component
            channel (str): name of channel
        '''
        pybind_message = self._app.receive(node, component, channel)

        if pybind_message.is_null():
            return None

        return Message.create_message_reader(pybind_message)

    def publish(self, node: str, component: str, channel: str, message: Message.MessageBuilder):
        '''
        Publishes a proto message to specified node/component/channel with provided proto data.
        '''
        assert node is not None and component is not None and channel is not None \
            and message is not None and isinstance(message, Message.MessageBuilder), 'Invalid argument'
        ok = self._app.publish(node, component, channel, message.uuid, message.proto.schema.node.id,
                               message.proto.to_bytes(), message.buffers, message.acqtime)
        if not ok:
            raise Exception('Failed to publish message')

    def expand_assets_path(self, asset_path: str) -> (str, str):
        '''
        Expands path like @workspace//foo/bar to workspace name and external/workspace/foo/bar
        '''
        return self._app.expand_asset_path(asset_path)

    @property
    def home_workspace_name(self):
        return self._app.get_home_workspace_name()

    @home_workspace_name.setter
    def home_workspace_name(self, home_workspace_name):
        self._app.set_home_workspace_name(home_workspace_name)

    def load(self, filename: str, prefix: str = '') -> bool:
        '''
        Loads a graph definition to app graph from specified json file. If prefix is specified, \
        the definition would be loaded as subgraph. Otherwise the loaded graph would be merged \
        with existing graph including nodes, edges and config.

        Args:
            filename (str): path to the subgraph json file
            prefix (str): name of subgraph name in case of loading subgraph
        '''
        workspace, path = self.expand_assets_path(filename)

        if len(workspace) == 0 and not os.path.isfile(path):
            # Could not find file, search in asset path
            if os.path.isfile(os.path.join(self._asset_path, path)):
                # Found file in asset path, use this location
                path = os.path.join(self._asset_path, path)
            else:
                # Still could not find file, raise exception
                raise ValueError('"{}" is not a valid file for subgraph.'.format(filename))

        if not self._app.load(path, prefix, workspace):
            raise ValueError('Failed to load subraph file {}.'.format(filename))

    def load_module(self, module: str) -> bool:
        '''
        Loads a specified module (shared library)

        Args:
            module (str): name of the module, e.g., 'flatsim'
        '''
        if not isinstance(module, str):
            return False
        return self._app.load_module(module)

    @property
    def clock(self) -> _ClockAccess:
        ''' Accessor of Isaac app clock '''
        return self._clock

    @property
    def atlas(self) -> _AtlasAccessor:
        ''' Accessor of poses in Isaac app '''
        return self._pose_tree

    def run(self, event=None):
        '''
        Starts the application and waits for event to stop.
        Complies with command line argument max_duration if it is set.

        Args:
            event (str or float or int or None):
                name of node to wait for or number of seconds to wait.
                In case of None, would wait for Ctrl-C/SIGINT.
        '''
        if event is not None and not isinstance(event, (int, float, str)):
            raise ValueError('Invalid event to wait for')
        node_name = event if isinstance(event, str) else None
        max_duration = event if isinstance(event, float) else None
        if max_duration is None:
            max_duration = self._args.max_duration
        elif isinstance(self._args.max_duration, (float, int)):
            max_duration = min(float(max_duration), float(self._args.max_duration))

        if self._start_status is not None:
            raise RuntimeError('Isaac App cannot be started twice')
        self._start_status = int(1)
        self._start_pycodelets()

        try:
            self._app.run(max_duration, node_name)
        except KeyboardInterrupt:
            traceback.print_exc()
        finally:
            self.stop(False)
