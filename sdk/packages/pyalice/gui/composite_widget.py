'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import ipywidgets as widgets
import numpy as np
from typing import List

from .. import Message, Composite


class CompositeWidget:
    """
    Create a ipywidget panel for displaying and controlling scalar variables in CompositeProto.
    caller in jupyter notebook should use IPython.display(this.panel) to show the widget
    """

    def __init__(self, entities: List, measure: str, limits: List):
        """
        Create a widget panel with sliders and labels for all scalar entities of a certain
        measure in a CompositeProto message

        Args:
            entities (List[str]): a list of entity names to parse state and send command
            measure (str): Composite measure to read and write message, for example "position"
            limits (List[List[int]]): a list of [min, max] values for each entry in the slider.
                                      size must match entities
        """
        self._has_state = False
        self._measure = measure
        self._entities = entities
        num_entities = len(self._entities)
        self._quantities = [[x, self._measure, 1] for x in self._entities]
        # Validate limits data
        if len(limits) != num_entities:
            raise ValueError("Size of limits does not match entity")
        for limit in limits:
            if len(limit) != 2:
                raise ValueError("Size of limit must be two")
        # Add a slider to change command and a label to display state to widget for each entity
        self._sliders = []
        self._labels = []
        names = []
        for i in range(num_entities):
            slider = widgets.FloatSlider(
                value=0.5 * (limits[i][0] + limits[i][1]),
                min=limits[i][0],
                max=limits[i][1],
                step=0.001,
                disabled=False,
                continuous_update=False,
                orientation='horizontal',
                readout=True,
                readout_format='.3f')
            self._sliders.append(slider)
            label = widgets.Label(value='N/A')
            self._labels.append(label)
            names.append(widgets.Label(value=self._entities[i], layout={'width': '150px'}))
        self._panel = widgets.VBox(
            [widgets.HBox(x) for x in zip(names, self._sliders, self._labels)])

    @property
    def panel(self):
        """
        Returns the widget panel object. Use IPython.display(panel) to show the widget panel
        inline in notebook
        """
        return self._panel

    @property
    def composite(self):
        """
        Reads commands from sliders in widget panel and generates the CompositeProto message.
        Returns None if the size of entities to parse is zero.
        """
        # only return a command if the initial state is known. prevents sending a very large delta
        if not self._has_state:
            return None

        num_entities = len(self._entities)
        if num_entities == 0:
            return None

        values = np.array([x.value for x in self._sliders], dtype=np.float64)
        return Composite.create_composite_message(self._quantities, values)

    @composite.setter
    def composite(self, composite: Message.MessageReader):
        """
        Show the current state from message to the labels in widget panel

        Args:
            composite: CompositeProto message received
        """
        values = Composite.parse_composite_message(composite, self._quantities)
        if len(self._labels) != len(values):
            raise ValueError("Size of state doesn't match number of labels")

        for i in range(len(values)):
            self._labels[i].value = '{:.3f}'.format(values[i])

        # set slider value (for command) to initial state
        if not self._has_state:
            for i in range(len(values)):
                self._sliders[i].value = values[i]
            self._has_state = True
