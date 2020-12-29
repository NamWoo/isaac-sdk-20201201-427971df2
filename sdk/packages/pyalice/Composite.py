'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import numpy as np
from typing import List

from . import Message
'''
This file provides helper functions to create a CompositeProto message. See
`create_composite_message` for details.
'''

ELEMENT_TYPE_F64 = 3


def _write_list_proto(proto, name, items):
    '''Writes a list of items to a proto'''
    target = proto.init(name, len(items))
    for i in range(len(items)):
        target[i] = items[i]


def _write_composite_quantity(proto, entity, measure, dimensions):
    '''Writes a single quantity to a proto'''
    proto.entity = entity
    proto.elementType = ELEMENT_TYPE_F64
    proto.measure = measure

    if dimensions is None:
        dimensions = [1]
    if not isinstance(dimensions, list):
        dimensions = [dimensions]
    _write_list_proto(proto.dimensions, "coefficients", dimensions)


def _write_composite_quantities(proto, quantities):
    '''Writes a list of quantities to a composite proto'''
    quantities_proto = proto.init('quantities', len(quantities))
    for i in range(len(quantities)):
        _write_composite_quantity(quantities_proto[i], quantities[i][0], quantities[i][1],
                                  quantities[i][2])


def create_composite_message(quantities, values):
    '''
    Creates a message of type CompositeProto with given schema and values.

    The schema is defined by a list of quantities where each quanty is a list of three elements:
    name, measure and dimensions. Dimensions can be a single value to define a vector or scalar,
    or a list to define a multi-dimensionsal quantity. Values are stored as a rank 3 numpy array
    where the first dimension is the number of batches, the second dimensions is the number of time
    entries per batch and the third dimension is the number of elements. The number of elements has
    to be identical to the sum of all quantity elements as defined by quantity dimensions.

    Message UUID, acqtime, and pubtime will not be set.

    Args:
        quantities (List): list of quantities
        values (numpy.array): a rank 3 numpy array
    '''
    msg = Message.create_message_builder('CompositeProto')
    proto = msg.proto

    # write schema
    _write_composite_quantities(proto, quantities)
    proto.schemaHash = ""

    # write values
    if values.dtype != "float64":
        raise Exception("Currently only float64 values are supported")
    proto.values.elementType = ELEMENT_TYPE_F64
    _write_list_proto(proto.values, "sizes", values.shape)
    proto.values.dataBufferIndex = 0
    msg.buffers = [values]

    return msg


def _quantities_match(requested, received):
    '''
    Checks if two quantities are equal

    Args:
        requested: A quantity described in create_composite_message comment
        received: json representation of a quantity received in message.
    '''
    if requested[0] != received['entity']: return False
    if requested[1] != received['measure']: return False
    # dimension is optional in received or can be scalar in requested, so needs to check all cases
    x_dim = requested[2] if isinstance(requested[2], list) else [requested[2]]
    y_dim = received["dimensions"]["coefficients"] if "dimensions" in received else [1]
    return x_dim == y_dim


def parse_composite_message(message: Message.MessageReader, quantities: List):
    """
    Return values as numpy array from CompositeProto. Do not support timestamp and batching yet.

    Args:
        message (Message.MessageReader): CompositeProto message reader
        quantities (List): list of quantities
    Returns:
        numpy.array, rank 1 or 2 (with timeseries) or 3 (with batch)
    """
    if message.type_id != Message.CAPNP_DICT["CompositeProto"].schema.node.id:
        raise ValueError("Not a CompositeProto")

    count = len(quantities)
    indices = [None] * count

    tensor = message.tensor
    if tensor is None:
        raise ValueError("Message buffer is empty")
    tensor_shape = tensor.shape
    rank = len(tensor_shape)
    if rank == 0 or rank > 3:
        raise ValueError("Message buffer rank {} not supported".format(rank))

    offset = 0
    for q in message.json["quantities"]:
        # dimension is optional in CompositeProto
        d = np.prod(np.array(q["dimensions"]["coefficients"])) if "dimensions" in q else 1
        for i in range(count):
            p = quantities[i]
            if _quantities_match(p, q):
                indices[i] = [offset, d]
        offset += d
    # tensor size incorrect
    if offset != tensor_shape[-1]:
        raise ValueError("CompositeProto tensor size does not match schema")

    # quantity missing in message
    for i in range(count):
        if indices[i] is None:
            raise ValueError("Missing {} in composite message".format(quantities[i]))

    # create an index array
    row_indices = [x[0] + y for x in indices for y in range(x[1])]
    if rank == 1:
        return tensor[row_indices]
    if rank == 2:
        return tensor[:, row_indices]
    if rank == 3:
        return tensor[:, :, row_indices]
