'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import time
from threading import Thread
from .CodeletFlowControl import CodeletFlowControl


class CodeletBackend(Thread):
    '''
    Python CodeletBackend object that helps Python Codelet to communicate to the C++ PyCodelet

    Args:
        *args, **kwargs: threading.Thread arguments

    Attributes:
        frontend (Codelet): Python Codelet instance to run
        flow_controller (CodeletFlowControl): the execution primitive that helps the backend to
          synchronize with C++ PyCodelet. The flow_controller will execute backend's callable
          attributes based on requests from C++ side.
    '''
    def __init__(self, *args, **kwargs):
        ''' Creates the backend thread that serves one Python codelet instance '''
        super(CodeletBackend, self).__init__(*args, **kwargs)
        self.bridge = None
        self.frontend = None
        self.flow_controller = None
        # True if the thread shall exit, otherwise keep checking if the node is re-started
        self.py_exit_flag = False

    def __str__(self):
        return self.frontend.name

    def __repr__(self):
        return self.__str__()

    def py_start(self):
        ''' Wrapper function for Codelet::start() '''
        self.frontend.start()

    def py_tick(self):
        ''' Wrapper function for Codelet::tick() '''
        self.frontend.tick()

    def py_stop(self):
        ''' Wrapper function for Codelet::stop() '''
        self.frontend.stop()

    def run(self):
        ''' Keeps running until CodeletFlowControl signals stop '''
        assert isinstance(self.flow_controller, CodeletFlowControl)
        self.frontend.log_debug("Launching {}".format(self.frontend.name))
        while True:
            keep_running = self.flow_controller.run()
            if not keep_running:
                if self.py_exit_flag:
                    # Python side is shutting down, exits
                    break
                else:
                    # Check if the node is started again until python side stops
                    time.sleep(0.05)
        self.frontend.log_debug("Stopped {}".format(self.frontend.name))
