'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from abc import abstractmethod
import random


class TaskPlannerInterface(object):
    '''Abstract base class for all TaskPlanner subclasses.'''

    @abstractmethod
    def get_next_object_to_pick(self):
        '''Return the name of the next object to pick up, or None if no tasks remain.'''
        pass

    @abstractmethod
    def get_next_putdown_pose(self):
        '''Return the putdown pose for the currently held object, or None if no tasks remain.'''
        pass

    @abstractmethod
    def all_tasks_done(self):
        '''Return True if no tasks remain, False otherwise.'''
        pass

    @abstractmethod
    def mark_current_task_as_done(self):
        '''Mark the current pick and place task as done and make the next one active.'''
        pass

    @abstractmethod
    def clear_all_tasks(self):
        '''Remove all tasks. Can be used for clean up after failed missions'''
        pass


class SimpleTaskPlanner(TaskPlannerInterface):
    '''Planner that maintains an ordered list of pick and place tasks.'''

    def __init__(self):
        '''Reset the internal object and destination pose lists.'''
        self.objects_to_pick = []
        self.putdown_poses = []

    def pick_and_place_object(self, object_name, putdown_pose):
        '''Add an object named object_name to pick up and place at pose putdown_pose.

        Keyword arguments:
        object_name (str): The name of the object's pose tree reference frame.
        putdown_pose (list(float)): The 3D destination pose where to place the object as
                             [q.w, q.x, q.y, q.z, t.x, t.y, t.z].
        '''
        self.objects_to_pick.append(object_name)
        self.putdown_poses.append(putdown_pose)

    def get_next_object_to_pick(self):
        if len(self.objects_to_pick) > 0:
            return self.objects_to_pick[0]

    def get_next_putdown_pose(self):
        if len(self.putdown_poses) > 0:
            return self.putdown_poses[0]

    def all_tasks_done(self):
        return len(self.objects_to_pick) == 0

    def mark_current_task_as_done(self):
        self.objects_to_pick = self.objects_to_pick[1:]
        self.putdown_poses = self.putdown_poses[1:]

    def clear_all_tasks(self):
        self.objects_to_pick = []
        self.putdown_poses = []


class ContinuousStackingTaskPlanner(TaskPlannerInterface):
    '''Planner that continuously stacks and unstacks a given set of objects.'''

    def __init__(self, object_height, plane_offset):
        '''Initialize the internal planner state.

        Keyword arguments:
        object_height (float): The common height of the objects to be stacked.
        plane_offset (float): The z offset for the stack from the plane.
        '''
        self.object_height = object_height
        self.plane_offset = plane_offset
        self.object_identifiers = []
        self.object_idle_poses = []
        self.currently_stacking = True
        self.objects_on_stack = []
        self.currently_held_object = None
        self.is_picking = True

    def add_object(self, object_identifier):
        '''Add an object to consider for stacking.

        Keyword arguments:
        object_identifier (str): The pose tree reference frame name of the object.
        '''
        self.object_identifiers.append(object_identifier)

    def add_object_idle_pose(self, idle_pose):
        '''Add a pose where to place objects during unstacking.

        There should be as many idle poses as objects considered for stacking.

        Keyword arguments:
        idle_pose (list(float)): The 3D pose where to place an object during unstacking as
                                 [q.w, q.x, q.y, q.z, t.x, t.y, t.z].
        '''
        self.object_idle_poses.append({'pose': idle_pose, 'occupied-by': None})

    def set_base_stack_pose(self, base_stack_pose):
        '''Set the 6 DoF base pose of the stack with respect to the "world" pose tree frane.

        Keyword arguments:
        base_stack_pose (list(float)): The 3D base pose of where to place the stack as
                                       [q.w, q.x, q.y, q.z, t.x, t.y, t.z].
        '''
        self.base_stack_pose = base_stack_pose

    def get_next_object_to_pick(self):
        self.currently_held_object = None

        if self.currently_stacking:
            # Stack. Pick next random non-stacked object.
            while self.currently_held_object is None:
                potential_object = random.choice(self.object_identifiers)

                if not potential_object in self.objects_on_stack:
                    self.currently_held_object = potential_object

            for idle_pose in self.object_idle_poses:
                if idle_pose['occupied-by'] == self.currently_held_object:
                    idle_pose['occupied-by'] = None
                    break
        else:
            # Un-stack. Pick the upper-most object.
            self.currently_held_object = self.objects_on_stack[-1]
            self.objects_on_stack = self.objects_on_stack[:-1]

        return self.currently_held_object

    def get_next_putdown_pose(self):
        putdown_pose = None

        if self.currently_stacking:
            putdown_pose = self.base_stack_pose.copy()
            # Add an offset equal to the object height times objects on stack
            putdown_pose[6] += len(self.objects_on_stack) * self.object_height
        else:
            # During un-stacking, place object at a random, free idle pose.
            while putdown_pose is None:
                idle_pose = random.choice(self.object_idle_poses)
                if idle_pose['occupied-by'] is None:
                    idle_pose['occupied-by'] = self.currently_held_object
                    putdown_pose = idle_pose['pose'].copy()

        # Add the plane offset to the putdown pose
        putdown_pose[6] += self.plane_offset

        return putdown_pose

    def all_tasks_done(self):
        '''Return False as the planner continuously stacks and unstacks.'''
        return False

    def mark_current_task_as_done(self):
        '''Mark the current pick and place task as done and make the next one active.

        When in stacking mode, check if all objects were placed on top of the stack and switch to
        unstacking mode in that case. When in unstacking mode, check if all objects were taken from
        the stack and switch into stacking mode.
        '''
        if self.currently_stacking:
            self.objects_on_stack.append(self.currently_held_object)
            self.currently_held_object = None

            if len(self.objects_on_stack) == len(self.object_identifiers):
                self.currently_stacking = False

        if not self.currently_stacking and len(self.objects_on_stack) == 0:
            self.currently_stacking = True
