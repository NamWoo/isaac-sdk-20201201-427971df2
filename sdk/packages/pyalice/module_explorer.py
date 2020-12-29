'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from typing import List


class ModuleExplorer(object):
    """
    This class can be used to find available components
    """
    def __init__(self, components: List[str], prefix=""):
        """
        Creates a new module explorer

        Args:
            components (List(str)): A list of components which are available
            prefix (str): The common prefix for all available components
        """

        # Filter all components which start with the prefix
        if prefix is None:
            self._components = components
        else:
            self._components = []
            for component in components:
                if component.startswith(prefix):
                    self._components.append(component)

        # Compute possible next selection criteria
        self._next = set()
        for component in self._components:
            if prefix == "":
                index = 0
            else:
                index = len(prefix) + 2
            remainder = component[index:]
            tokens = remainder.split("::")
            self._next.add(tokens[0])

        self._prefix = prefix

    def __getattr__(self, name):
        """
        Returns a module explorer which contains all components which have a combined prefix of
        the current prefix and the given `name`.

        Args:
            name (str): The additional string which is added to the prefix. Must not contain `::`.
        """
        if name in self._next:
            if self._prefix == "":
                next_prefix = name
            else:
                next_prefix = self._prefix + "::" + name
            return ModuleExplorer(self._components, prefix=next_prefix)
        raise RuntimeError('No component name fits {}::{}'.format(self._prefix, name))

    def __str__(self):
        return str(self._components)

    @property
    def name(self):
        """
        If this the selection of components was narrowed to a single component the type of this
        component is returned. Otherwise None is returned.
        """
        if len(self._components) == 1:
            return self._components[0]
        else:
            # Check if there is an exact match for the current prefix. This is necessary to resolve
            # for example [foo.bar.Test, foo.bar.TestMore]
            for component in self._components:
                if component == self._prefix:
                    return component
            return None
