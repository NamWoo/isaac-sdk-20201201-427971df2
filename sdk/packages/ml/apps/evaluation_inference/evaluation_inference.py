'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import argparse
import json
import os
import time
from pathlib import Path

from packages.pyalice import Application
from packages.cask.apps import multi_cask_processing
"""
This application runs parallel inference processes on a set of multiple Isaac logs. An Isaac
application for inference that includes replay and record nodes must be specified, in addition
to a separate configuration file containing inference parameters. The output is a set of Isaac
logs containing the inference predictions with optional JSON metadata (specified as per RACI
Data Workflow) written per output log.
"""


def main(args):
    # Set up input cask and output cask directories
    input_cask_list = multi_cask_processing.load_roots(workspace=args.input_cask_workspace)
    input_cask_uuid_list = [os.path.split(i)[1] for i in input_cask_list]
    predictions_cask_list = multi_cask_processing.processing_stage_roots(
        input_cask_list,
        args.output_directory_name,
        experiment=args.experiment,
        output_workspace=args.output_cask_workspace)

    # Set up tasks to run in parallel
    isaac_tasks = []
    for image, prediction in zip(input_cask_list, predictions_cask_list):
        isaac_tasks.append({
            'app_filename': args.inference_app,
            'more_jsons': args.inference_config,
            'more_json': {
                "config": {
                    "replay.interface": {
                        "output": {
                            "cask_directory": image
                        }
                    },
                    "record.interface": {
                        "input": {
                            "base_directory": prediction
                        }
                    }
                }
            },
            'success_node': "replay.interface",
            'duration': args.max_runtime,
            'poll_interval': 1.0,
            'exit_interval': 1.0
        })

    # Run tasks
    if (args.parallel):
        result = multi_cask_processing.run_parallel(isaac_tasks, max_workers=args.max_workers)
    else:
        for task in isaac_tasks:
            result = multi_cask_processing.run(task)

    # Write metadata to JSON data per output cask
    if args.raci_metadata:
        # Populate general metadata
        metadata_json = {}
        metadata_json["Robot_Name"] = ""
        metadata_json["Upload_Time"] = ""
        training_metadata = {}
        training_metadata["Dataset_Size"] = args.dataset_size
        training_metadata["Num_Epochs"] = args.num_epochs
        metadata_json["Training"] = training_metadata

        # Load inference config JSON and save to metadata
        inference_config = {}
        inference_config_path = os.fspath(Path(args.inference_config).resolve())
        with open(inference_config_path) as f:
            inference_config = json.load(f)
        metadata_json["Inference"] = inference_config

        # Write all metadata files
        for cask_idx in range(len(predictions_cask_list)):
            predictions_cask = predictions_cask_list[cask_idx]
            cask_metadata = metadata_json
            cask_metadata["Image_Cask_File"] = input_cask_uuid_list[cask_idx]
            cask_metadata_path = predictions_cask + "_md.json"
            with open(cask_metadata_path, 'w') as f:
                json.dump(cask_metadata, f, indent=2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Perform inference for evaluation')
    parser.add_argument(
        '--inference_app',
        dest='inference_app',
        default='packages/detect_net/evaluation/detect_net_eval_inference.app.json',
        help='Application file that replays a log, performs inference, and records results')
    parser.add_argument(
        '--inference_config',
        dest='inference_config',
        default='packages/detect_net/apps/detect_net_industrial_dolly.config.json',
        help='Config file to load for inference parameters.')
    parser.add_argument('--parallel', dest='parallel', action='store_true')
    parser.add_argument('--no_parallel', dest='parallel', action='store_false')
    parser.set_defaults(parallel=True)
    parser.add_argument('--raci_metadata', dest='raci_metadata', action='store_true')
    parser.add_argument('--no_raci_metadata', dest='raci_metadata', action='store_false')
    parser.set_defaults(raci_metadata=True)
    parser.add_argument('--input_cask_workspace',
                        dest='input_cask_workspace',
                        required=True,
                        help='The workspace containing the input cask files.'
                        'Input logs must be directory data/raw inside this workspace.')
    parser.add_argument(
        '--output_cask_workspace',
        dest='output_cask_workspace',
        type=str,
        default=None,
        help='The workspace to write predictions cask output.'
        'The output cask files are written in data/<output_directory_name> inside this workspace.'
        'If not set, it is assumed to be the input_cask_workspace')
    parser.add_argument(
        '--output_directory_name',
        dest='output_directory_name',
        type=str,
        default='predictions',
        help='Base directory name to write predictions cask output.'
        'Cask files are created in <output_cask_workspace>/data/<output_directory_name>.')
    parser.add_argument(
        '--experiment',
        dest='experiment',
        type=str,
        default=None,
        help='Name of the evaluation experiment. It helps in maintaining unique id for each run.')
    parser.add_argument('--max_runtime',
                        dest='max_runtime',
                        type=float,
                        default='2040.0',
                        help='Max number of seconds to run the inference app')
    parser.add_argument('--max_workers',
                        dest='max_workers',
                        type=int,
                        default=1,
                        help='Max number of workers to use during parallel inference')
    parser.add_argument('--dataset_size',
                        dest='dataset_size',
                        type=int,
                        default=0,
                        help='Size of training dataset to write to metadata')
    parser.add_argument('--num_epochs',
                        dest='num_epochs',
                        type=int,
                        default=0,
                        help='Number of training epochs to write to metadata')

    args, _ = parser.parse_known_args()
    main(args)
