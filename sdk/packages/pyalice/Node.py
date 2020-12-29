'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from .bindings import Status    # pylint: disable=no-name-in-module
from .Codelet import Codelet
from .Component import Component
from .module_explorer import ModuleExplorer


class _ComponentsAccess(object):
    '''
    This class provides easy access to components of managed node
    '''
    _NODE_ATTR_NAME = '_node'

    def __init__(self, node):
        """
        Creates a components accessor

        Args:
            node (Node): The node to access
        """
        if node is None:
            raise ValueError('node must not be None')
        self._node = node
        self._names = node._node.get_all_component_names()

    def __getitem__(self, key: str):
        '''
        Provides 'components['key']' read access of managed node
        '''
        if isinstance(key, str):
            handle = self._node._node.get_component(key)
            if handle is None:
                return None
            return Component(self._node, handle)
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


class _NodeConfig(object):
    '''
    This class provides easy access to config of node
    '''
    def __init__(self, node):
        """
        Creates a config accessor

        Args:
            node (Node): The node to access
        """
        if node is None:
            raise ValueError('node must not be None')
        self._node = node._node

    def __setitem__(self, key: str, value):
        '''
        Provides 'config['key']' to set the config parameter value with specified param key for
        managed component

        Args:
            key (str): parameter name
            value: new parameter value
        '''
        if value is None:
            raise ValueError('Could not set None')
        if key == "disable_automatic_start":
            if not isinstance(value, bool):
                raise ValueError('Cannot set Boolean disable_automatic_start to {}'.format(value))
            self._node.disable_automatic_start(value)
            return
        if key == "start_order":
            if not isinstance(value, int):
                raise ValueError('Cannot set Int start order to {}'.format(value))
            self._node.set_start_order(value)
            return
        raise ValueError('"{}" is not a valid configuration for nodes'.format(key))


class Node(object):
    """
    Nodes are the building blocks of an ISAAC SDK application.

    This class is a thin wrapper around PybindNode which provides more convenience functions.
    """
    def __init__(self, app, node):
        """
        Creates a new node based on a pybind node

        Args:
            app (Application): The app which owns this node
            node (PybindNode): The pyhind node handle
        """
        self._app = app
        self._node = node
        self._config = _NodeConfig(self)

    @property
    def handle(self):
        """Returns a handle to the underlying pybind node"""
        return self._node

    @property
    def app(self):
        """The app which owns this node"""
        return self._app

    @property
    def name(self):
        """The name of the node"""
        return self._node.name()

    @property
    def components(self):
        ''' Returns components accessor '''
        return _ComponentsAccess(self)

    @property
    def config(self):
        ''' Returns config accessor '''
        return self._config

    def add(self, ctype, name=None):
        """
        Adds a new component to the node. Could be cpp codelet or Python codelet.
        Returns the created component instance or corresponding C++ component instance for Python
        codelet.

        The component is created based on the desired type name. The type name can either be given
        as a string or by using a ModuleExplorer. If a ModuleExplorer is used the selection must
        have been narrowed down to a single component otherwise an exception is raised.

        Every component must have a unique name. If the name is not given it is chosen based on the
        type name.

        Args:
            ctype (str or ModuleExplorer or Codelet class): The type of the component to be added.
            name (str): The (optional) name of the component
        """
        if isinstance(ctype, ModuleExplorer):
            ctype = ctype.name
        if isinstance(ctype, str):
            if ctype is None:
                raise Exception("Component type must not be None")
            if name is None:
                name = ctype.split("::")[-1]
            return Component(self, self._node.add_component(ctype, name))
        # For PyCodelet, a corresponding C++ isaac.alice.PyCodelet component would be created
        # before binding is done via Application.
        if issubclass(ctype, Codelet):
            if name is None:
                name = 'PyCodelet'
            stub_type = self._app.registry.isaac.alice.PyCodelet.name
            # Creates the PyCodelet component if needed
            component = self.components[name]
            if component is None:
                component = Component(self, self._node.add_component(stub_type, name))
            # Creates Python Codelt instance now
            self._app._bind_pycodelet(ctype, component)
            return component

    def __getitem__(self, name):
        """
        Returns the component in the node with the given name. If there is no such component None
        is returned.

        Args:
            name (str): The name of the component to return.
        """
        handle = self._node.get_component(name)
        if handle is None:
            return None
        else:
            return Component(self, handle)

    @property
    def status(self) -> Status:
        '''
        The current status of the node as defined in engine/alice/status.hpp.
        Possible values are:
            .bindings.Status.Success
            .bindings.Status.Failure
            .bindings.Status.Running
            .bindings.Status.Invalid
        '''
        return self._node.get_status()

    def start(self):
        '''
        Starts the node if it is stopped (Success, Failure status or Stopped by stop() below).
        Shall only be used for node that is stopped.
        Use with care.
        '''
        self._node.start()

    def stop(self):
        '''
        Stops the node if it is running (Running status). Shall only be used for node that is
        currently running.
        Use with care.
        '''
        self._node.stop()
