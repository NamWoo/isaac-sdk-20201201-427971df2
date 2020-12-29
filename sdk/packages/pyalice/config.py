'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import json


class Config(object):
    '''
    This class provides easy access to config of application
    '''
    _COMPONENT_ATTR_NAME = '_component'

    def __init__(self, component):
        """
        Creates a component config accessor

        Args:
            app (Component): The application to access
        """
        if component is None:
            raise ValueError('Component must not be None')
        super().__setattr__(self._COMPONENT_ATTR_NAME, component)

    def __getattr__(self, name):
        '''
        Reads the config parameter value of managed component

        Args:
            name (str): parameter name
        '''
        component = super().__getattribute__(self._COMPONENT_ATTR_NAME)
        assert component is not None
        result = json.loads(
            component.node.app._app.get_config(component.node.name, component.name, name))
        if isinstance(result, dict) and len(result) == 0:
            # In case of key not found, gives None
            return None
        return result

    def __setattr__(self, name, value):
        '''
        Sets the config parameter value with specified param name for managed component

        Args:
            name (str): parameter name
            value: new parameter value
        '''
        if value is None:
            raise ValueError('Could not set None')
        component = super().__getattribute__(self._COMPONENT_ATTR_NAME)
        component.node.app._app.set_config(component.node.name, component.name, name,
                                           json.dumps(value))

    def __delattr__(self, name):
        '''
        Provides 'del config.key' support of component config

        Args:
            name (str): parameter name
        '''
        component = super().__getattribute__(self._COMPONENT_ATTR_NAME)
        assert component is not None
        component.node.app._app.erase_config(component.node.name, component.name, name)

    def __getitem__(self, key: str):
        '''
        Provides 'config['key']' read access of component config
        '''
        return self.__getattr__(key)

    def __setitem__(self, key: str, value):
        '''
        Provides 'config['key']' write access of component config
        '''
        self.__setattr__(key, value)

    def __delitem__(self, key: str):
        '''
        Provides 'del config['key']' support of component config
        '''
        self.__delattr__(key)