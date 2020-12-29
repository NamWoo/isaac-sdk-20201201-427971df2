'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from .module_explorer import ModuleExplorer
from .config import Config
from .bindings import PybindComponent    # pylint: disable=no-name-in-module


class Component(object):
    """
    Components are the building blocks of nodes in an ISAAC SDK application

    This class is a thin wrapper around PybindNode which provides more convenience functions.
    """
    def __init__(self, node, component):
        """
        Creates a new component based on a pybind component

        Args:
            node (Node): The node which owns this component
            component (PybindComponent): The pyhind component handle
        """
        self._node = node    # Instance of Node
        assert isinstance(component, PybindComponent)
        self._component = component
        # Config Accessor
        self._config = Config(self)

    @property
    def handle(self):
        """Returns a handle to the underlying pybind component"""
        return self._component

    @property
    def node(self):
        """The node which owns this component"""
        return self._node

    @property
    def name(self):
        """The name of the component"""
        return self._component.name()

    @property
    def config(self):
        ''' Accessor for component config '''
        return self._config
