'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

'''

"""
Helpers that allow managing of processing stages and experiments, allow to:
 * load a set of log paths to process;
 * set a stage for a new set of logs (to be produced by a processing stage);
 * process set of logs in parallel;

For example:
>>>
>>> from packages.cask.apps import multi_cask_processing as processing
>>>
>>> roots = processing.load_roots()
>>>
>>> roots
>>>
["data/raw/set1/07a24b64-3f03-11ea-b17c-cb1441dbfa9c",
 "data/raw/set2/0c2d809a-38cd-11ea-8bb7-79860d087101"]

>>> apriltags_roots = processing.processing_stage_roots(roots, "apriltags")
>>>
>>> apriltags_roots
>>>
["data/apriltags/set1/07a24b64-3f03-11ea-b17c-cb1441dbfa9c.apriltags",
 "data/apriltags/set2/0c2d809a-38cd-11ea-8bb7-79860d087101.apriltags"]

>>>
>>> # Setup tasks
>>> isaac_tasks = []
>>> for root, apriltags_root in zip(roots, apriltags_roots):
>>>      isaac_tasks.append({
>>>         'app_filename' : "packages/detect_net/apps/apriltags_process.app.json",
>>>         'more_json' : {
>>>              "config": {
>>>                  "replay.interface": {"output": {"cask_directory": root}},
>>>                  "record.interface": {"input": {"base_directory": apriltags_root}}
>>>              }
>>>         },
>>>         'success_node': "replay.interface",
>>>         'duration': 2040.0, 'poll_interval': 1.0, 'exit_interval': 1.0
>>>        })

>>> # Run tasks in parallel, this will produce Isaac logs in the apriltags_roots
>>> result = processing.run_parallel_tasks(isaac_tasks, max_workers = 4)

"""

import glob, os.path, bisect, json, logging, tempfile, concurrent.futures, time, sys
from isaac import Application, Cask, bindings

# Making the tqdm dependency optional pass-through, as it is not available on Jetson platforms.
try:
    from tqdm import tqdm
except ImportError:
    tqdm = lambda iterator, *args, **kwargs: iterator


def load_roots(workspace = "", roots_directory = "data/raw"):
    '''
    This function searches file system for './data/raw/**/kv' files and outputs`
    the list of Cask roots sub-directories containing found 'kv' files.

    It is used to find root paths of Isaac Cask logs. Examples, a call without arguments
    in a directory containing following subdirectories:

        ├── data
            ├── raw
            │   ├── 1b4e53a4-3424-11ea-8651-039447a80f01 / kv
            │   ├── set1
            │   │   ├── 07a24b64-3f03-11ea-b17c-cb1441dbfa9c / kv

    would return a list:
        [   "data/raw/1b4e53a4-3424-11ea-8651-039447a80f01",
            "data/raw/set1/07a24b64-3f03-11ea-b17c-cb1441dbfa9c"    ]

    The function allows to specify a workspace and log roots directory in that workspace.
    In that case the search pattern is modified to: '<workspace>/<roots_directory>/**/kv'

    It is recommended to use the workspace parameter as system/deploynment dependent location.
    And to use the <roots_directory> parameter to be invariant to the deploynment location.

    It is used to find root paths of Isaac Cask logs. Examples, a call without arguments
    in a directory containing following subdirectories:

    Arguments:
        workspace (str): a path to the top folder containing log files. For example,
                         this could be a path to the deployed Isaac SDK pacckage or
                         a scratch temporary space.

        roots_directory (str): a relative (to workspace) path to "root" log paths.

    Returns:
        The list of Cask roots sub-directories containing found 'kv' files
    '''

    if roots_directory.startswith("/"):
        logging.warning("Unexpected roots_directory = %s absolute path. Note: roots_directory "
                        "is expected to be a relative path, invariant to the deployment location. "
                        "Please use the workspace parameter instead.", roots_directory)

    log_paths = glob.glob(os.path.join(workspace, roots_directory, '**', 'kv'), recursive=True)
    roots = sorted([log[:-3] for log in log_paths])
    logging.info("Found: %d logs in the %s directory.",
            (len(roots), os.path.join(workspace, roots_directory)))
    return roots


def processing_stage_roots(roots,
                           processing_stage,
                           experiment = None,
                           output_workspace = None,
                           roots_directory = "data/raw",
                           report_existing = True):
    '''
    Converts roots to processing stage roots.

    This function takes a list root paths for Isaac Logs and:
       1. substitutes basename(roots_directory) in this path into 'stage/experiment' path
       2. adds '.stage.experiment' extention to the end of the path

    This is used to create unique path for results of a stage of a
    processing pipeline and/or results of the experiment.

    The "os.path.basename()" of these paths are:
      * guaranteed to start from the UUID of the root log.

    Example:

    >>> roots = ["data/raw/set1/07a24b64-3f03-11ea-b17c-cb1441dbfa9c",
    >>>          "data/raw/set1/0c2d809a-38cd-11ea-8bb7-79860d087101"]
    >>>
    >>> apriltags_roots = processing_stage_roots(roots, "apriltags")
    >>>
    >>> apriltags_roots
    >>>
    ["data/apriltags/set1/07a24b64-3f03-11ea-b17c-cb1441dbfa9c.apriltags",
     "data/apriltags/set1/0c2d809a-38cd-11ea-8bb7-79860d087101.apriltags"]

    Arguments:
        roots (list): a list of paths to the existing log files.
        processing_stage (str): the name of the processing stage of the experiment
        experiment (str): the name of the experiment
        output_workspace (str): an optional path to the top folder for output log files.
                                For example, this could be a path to the deployed
                                Isaac SDK pacckage or a scratch temporary space.
                                This path doesn't have to be the same as the original
                                logs workspace path. By default, it will be the original
                                workspace path.
    Returns:
        (list) : a list of new paths to new processing stage roots.
    '''

    normalized_roots_directory = os.path.normpath(roots_directory)
    data_directory = os.path.dirname(normalized_roots_directory)
    normalized_roots_directory = os.path.join(normalized_roots_directory, '')
    stage = os.path.join(processing_stage, experiment) if experiment else processing_stage
    extension =  ('.' + processing_stage)  + ('.' + experiment if experiment else '')

    target_roots = []
    for root in roots:
        try:
            workspace,log = root.split(normalized_roots_directory)
        except:
            logging.error("Couldn't locate roots_directory %s in the %s root. Please check that ",
                          "you've specified the roots_directory and that root path contains it.",
                           root, normalized_roots_directory)
            continue

        if output_workspace is not None:
            workspace = output_workspace

        target_roots.append(os.path.join(workspace, data_directory, stage, log) + extension)

    if report_existing:
        existing_roots = [path for path in target_roots if os.path.exists(path)]
        logging.info("Found %d already existing paths for %s (out of %d)",
                        (len(existing_roots), processing_stage, len(roots)))
    return target_roots


def load_experiment(workspace = "",
                    processing_stage = "predictions",
                    experiment = None,
                    model = None,
                    output_workspace = None,
                    roots_directory = "data/raw",
                    ground_truth_directory = "ground_truth",
                    models_directory = "models",
                    ):
    '''
    This function searches file system for workspace/data/raw/**/kv files and outputs:
      * the list of sub-directories containing found 'kv' files.
      * the path to the experimental model file
      * the list of target paths, for the ground truth (labels)
      * the list of target paths, for the processing stage of an experiment

    The os.path.basename() of these paths are:
      * guaranteed to be unique;
      * guaranteed to start from UUID of the log;

    For example:
        roots, label_roots, detection_roots, model =
            load_experiment("packages/detect_net", "detections", "run0", "model.onnx")

    Could produce following set in the "packages/detect_net" folder:
            ├── raw
            │   ├── set1
            │       ├── 07a24b64-3f03-11ea-b17c-cb1441dbfa9c
            │       ├── 1b4e53a4-3424-11ea-8651-039447a80f01
            │       ├── ...│
            │
            ├── ground_truth
            │   ├── set1
            │       ├── 07a24b64-3f03-11ea-b17c-cb1441dbfa9c.labels
            │       ├── 1b4e53a4-3424-11ea-8651-039447a80f01.labels
            │       ├── ...│
            │
            ├── detections
                ├── run0
                        ├── set1
                            ├── 07a24b64-3f03-11ea-b17c-cb1441dbfa9c.detections.run0
                            ├── 1b4e53a4-3424-11ea-8651-039447a80f01.detections.run0
                            ├── ...│

    Arguments:
        workspace (str): a path to the top folder containing log files. For example,
                         this could be a path to the deployed Isaac SDK pacckage or
                         a scratch temporary space.

        roots_directory (str): a relative (to workspace) path to "root" log paths.
        processing_stage (str): the name of the processing stage of the experiment
        experiment (str): the name of the experiment
        output_workspace (str): an optional path to the top folder for output log files.
                                For example, this could be a path to the deployed
                                Isaac SDK pacckage or a scratch temporary space.
                                This path doesn't have to be the same as the original
                                logs workspace path. By default, it will be the original
                                workspace path.
    Returns:
        A tuple containing:
            roots (list): a list of paths to the existing log files.
            ground_truth_roots (list) : a list of ground truth processing stage root paths.
            stage_roots (list) : a list of processing stage root paths.
            model (str) (optional): a path to the model, if the model argument was specified

    '''
    roots =  load_roots(workspace, roots_directory)
    label_roots = processing_stage_roots(roots, ground_truth_directory,
                                         output_workspace = output_workspace,
                                         roots_directory = roots_directory)
    stage_roots = processing_stage_roots(roots, processing_stage, experiment,
                                         output_workspace = output_workspace,
                                         roots_directory = roots_directory)
    if model:
        model = os.path.join(workspace, models_directory, model)
        logging.info("Model %s was%s found" % (model, "" if os.path.exists(model) else "n't"))
        return roots, label_roots, stage_roots, model
    else:
        return roots, label_roots, stage_roots


def load_log(root):
    """
    Loads a cask for a given root.

    Note: a root may contain a Cask log file or a single arbitraty
          named UUID subfolder containig a single Cask log file
    """
    log_paths = glob.glob(os.path.join(root, '**', 'kv'), recursive=True)
    if not log_paths:
        logging.error("Log path %s doesn't contain any Isaac logs." % root)
        return None

    if len(log_paths) > 1:
        logging.error("Log path %s should only contain a single Isaac log."
                      "Please remove these logs and re-run the processing stage." % root)
        return None

    return Cask(log_paths[0][:-3])


def read_message(cask, channel, acqtime):
    """
        Retrieves a message by acqtime. Raises ValueError, if the message was not found.

        Arguments:
            cask (isaac.Cask): opened log file
            channel (str): Cask channel name
            acqtime (int): Target message acquisition time
    """

    class BisectHelper(object):
        def __init__(self, series):
            self.series = series
        def __len__(self):
            return len(self.series)
        def __getitem__(self, message_index):
            return self.series[message_index][0]

    cask_channel = cask[channel]
    message_index = bisect.bisect_left(BisectHelper(cask_channel.metadata), acqtime)
    if message_index == len(cask_channel) or cask_channel[message_index].acqtime != acqtime:
        raise ValueError
    return cask_channel[message_index]


def run(task):
    '''
        Given a task description, starts the application, waits for the node to stop
        running or for the duration to lapse. Stops the application after the execution.

        Can be used standalone or with the run_parallel helper.

        Example:
        >>> task = {
        >>>         'app_filename' : 'packages/detect_net/apps/apriltags_process.app.json',
        >>>         'more_json' : {
        >>>              "config": {
        >>>                  "replay.interface": {"output": {"cask_directory": root}},
        >>>                  "record.interface": {"input": {"base_directory": apriltags_root}}
        >>>              }
        >>>         },
        >>>         'success_node': "replay.interface",
        >>>         'duration': 2040.0, 'poll_interval': 1.0, 'exit_interval': 1.0
        >>>        }
        >>>
        >>> run(task)

        Args:
            task['app_filename'] (str): the main application json filename
            task['more_json'] (dict): a dictionary containing additional configuration,
                                   to serialize into json and load as more_jsons to the app
            task['more_jsons'] (str): a comma-separated string of additional jsons to load
            task['name'] (str): the name of the application
            task['modules'] (List(str)): a list of modules to be loaded
            task['argv']: Command line arguments from sys.argv
            task['success_node'] (string): the name of the node to wait to stop running
            task['duration'] (float): timeout in seconds, can run forever, if duration is None
            task['exit_interval'] (float): exit interval in seconds

        Returns: True, if the success status had been reached.  False otherwise.
    '''
    with tempfile.NamedTemporaryFile(mode='w+t', suffix='.json', delete=True) as more_json:
        json.dump(task['more_json'], more_json)
        more_json.flush()
        more_jsons = ((task['more_jsons'] + ',') if 'more_jsons' in task else '') + more_json.name
        app = Application(app_filename = task['app_filename'],
                          more_jsons = more_jsons,
                          name = task.get('name', None),
                          modules = task.get('modules', None),
                          argv = task.get('argv', sys.argv))

    app.start()
    status = app.wait_for_node(task['success_node'], task['duration'])
    time.sleep(task['exit_interval'])
    app.stop()
    return status == bindings.Status.Success


def run_parallel(tasks, max_workers=2, run_first_task_sequentially = True):
    '''
        Given a list of task descriptions, runs the tasks in parallel in separate processes.

        Example: please refer to the example at the module level.

        Args a list of tasks, each containing:
            task['app_filename'] (str): the main application json filename
            task['more_json'] (dict): a dictionary containing additional configuration,
                                   to serialize into json and load as more_jsons to the app
            task['success_node'] (string): the name of the node to wait to stop running
            task['duration'] (float): timeout in seconds, can run forever, if duration is None
            task['exit_interval'] (float): exit interval in seconds

        Returns: True, if the success status had been reached.  False otherwise.
    '''
    executor = concurrent.futures.ProcessPoolExecutor(max_workers)
    logging.disable(level = logging.DEBUG)
    if run_first_task_sequentially:
        results = list(tqdm(executor.map(run, tasks[:1]), total = 1, desc = "First task")) + \
                  list(tqdm(executor.map(run, tasks[1:]), total = len(tasks) - 1, desc = "Tasks"))
    else:
        results = list(tqdm(executor.map(run_task, tasks), total = len(tasks), desc = "Tasks"))
    logging.disable(level = logging.NOTSET)

    # Report failed tasks
    unsuccesful_tasks = [task for task, result in zip(tasks, results) if not result]
    if unsuccesful_tasks:
        logging.error("Status unsuccesful on tasks %s" % unsuccesful_tasks)
        return False
    return True

if __name__ == "__main__":
    pass