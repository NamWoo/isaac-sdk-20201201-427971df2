'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import sys


class CodeletFlowControl(object):
    """
    Python flow control primitive that helps Python codelet backend to execute the job that C++
    PyCodeletFlowControl sends

    Args:
        backend (CodeletBackend): the object that actually is able to execute the job
        bridge (PybindPyCodelet): the pybind C++ pycodelet that sends the job to python side

    Attributes:
        same as above
    """
    def __init__(self, backend, bridge):
        self.backend = backend
        self.bridge = bridge
        self.queue = None

    def run(self):
        """
        Execution primitive for performing a job that the C++ PyCodelet delegates. The run function
        will first request for a job and perform the job. Upon completing the job the function
        notifies the C++ PyCodelet.

        Returns:
            False if only when the codelet needs to be stopped
        """
        job = self.bridge.python_wait_for_job()    # Requests for a job
        # Null job means exiting
        if not job:
            return False
        keep_running = True
        try:
            assert hasattr(self.backend, job), \
                "python flow controller {} cannot execute job {} (attribute not found)".format( \
                self.backend, job)
            job_callable = getattr(self.backend, job)
            assert callable(job_callable), \
                "python flow controller {} cannot execute job {} (not callable)".format( \
                self.backend, job)
            job_callable()
        except Exception:
            self.queue.put(sys.exc_info())
            self.backend.frontend.report_failure(str(sys.exc_info()))
        finally:
            # notifies the bridge that the job is completed
            self.bridge.python_job_finished()
        return keep_running
