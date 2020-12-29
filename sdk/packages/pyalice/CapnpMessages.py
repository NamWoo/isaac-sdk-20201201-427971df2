'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import os
import capnp
import glob

PATH = "/messages/*.capnp"       # the path to find all ".capnp" files
CLASS_NAME = "_StructModule"    # the name that pycapnp uses for its pycapnp schema object


def get_capnp_proto_schemata():
    '''Load all the capnp'n'proto schemata in the project. The function will glob through all the
  files with "*.capnp" extension name. It will return a dictionary that maps from the name of each
  capnp proto schema to the actual pycapnp schema object that can read/create capnp'n'proto message.

  Returns:
      A dictionary that maps from the name of capnp proto schema to the pycapnp proto schema object
  '''
    capnp_files = glob.glob(os.getcwd() + PATH)
    if not capnp_files:
        capnp_files = glob.glob(os.path.dirname(os.path.abspath(__file__)) + PATH)

    capnp_dict = {}
    for capnp_f in capnp_files:    # loop through the codebase to find all
        module = capnp.load(capnp_f)    # the .capnp files
        for name, obj in module.__dict__.items():    # register the name of the proto message
            if obj.__class__.__name__ == CLASS_NAME:    # type
                assert name not in capnp_dict
                capnp_dict[name] = obj    # store the capnp struct
    return capnp_dict


def capnp_schema_type_id_dict():
    '''
  Creates a dictionary which maps Capn'proto type ids to class schemata
  '''
    capnp_files = glob.glob(os.getcwd() + PATH)
    if not capnp_files:
        capnp_files = glob.glob(os.path.dirname(os.path.abspath(__file__)) + PATH)

    result = {}
    for capnp_f in capnp_files:
        module = capnp.load(capnp_f)
        for name, obj in module.__dict__.items():
            if obj.__class__.__name__ == CLASS_NAME:
                assert name not in result
                result[obj.schema.node.id] = obj
    return result
