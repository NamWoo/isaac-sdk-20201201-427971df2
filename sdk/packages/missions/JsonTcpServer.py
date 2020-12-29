'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
from collections import deque
import json
import select
import socket
import threading
import time
from typing import Callable, List
import uuid

from isaac import Message as im

# The maximum number of messages to store for a given channel before dropping the oldest message
MESSAGE_QUEUE_SIZE = 25
# How many seconds to wait between calls to the update callback
UPDATE_CALLBACK_PERIOD = 1


def _signed_to_unsigned(value: int, bits: int = 64) -> int:
    ''' Reinterprets the bits of a signed integer as an unsigned integer '''
    if value < 0:
        return value + 2**bits
    return value


def _unsigned_to_signed(value: int, bits: int = 64) -> int:
    ''' Reinterprets the bits of an unsigned integer as an signed integer '''
    if value >= (1 << (bits - 1)):
        return value - 2**bits
    return value


def _transform_field_of_type(schema, struct, transform, match_type: str = "uint64"):
    '''
    Go through a dictionary encoded capnp-proto and perform the transformation function on all
    fields that match the given type

    Args:
        schema (capnp schema): The capnp schema the dictionary follows
        struct (dict): The dictionary to transform
        transform (function): The transformation function to apply on all matching fields
        match_type (str): The capnp type of fields to be transformed

    '''
    for field in schema.node.to_dict()["struct"]["fields"]:
        # Get the name and type of the field
        name = field["name"]
        field_type = list(field['slot']['type'].keys())[0]
        field_data = field['slot']['type'][field_type]
        if field_type == match_type:
            struct[name] = transform(struct[name])
        elif field_type == "struct":
            _transform_field_of_type(im.CAPNP_TYPE_ID_DICT[field_data["typeId"]].schema,
                                     struct[name], transform, match_type)


def _proto_to_dict(proto):
    '''
    Converts a capnp proto to a dictionary and convert all 64 bit integers to strings

    This is due to https://github.com/capnproto/capnproto/issues/617. When capnp deserializes
    json to a proto, it expects all int64/uint64 fields to be encoded as strings. Pycapnp,
    however, serializes them as integers, so this conversion step is necessary.
    '''
    proto_dict = proto.to_dict()
    _transform_field_of_type(proto.schema, proto_dict, lambda x: str(x))
    return proto_dict


def _dict_to_proto(proto_dict, proto_id):
    '''
    Converts a dictionary where all 64 bit integers are strings to a capnp proto

    This is due to https://github.com/capnproto/capnproto/issues/617. When capnp serializes a proto
    to json, all int64/uint64 fields are encoded as strings. Pycapnp, however expects them to be
    integers, so this conversion step is necessary.
    '''
    _transform_field_of_type(im.CAPNP_TYPE_ID_DICT[proto_id].schema, proto_dict, lambda x: int(x))
    return im.CAPNP_TYPE_ID_DICT[proto_id].from_dict(proto_dict)


class JsonTcpServerConnection:
    '''
    A class representing the connection between a JsonTcpServer and a TCP Client.
    '''

    def __init__(self, connection):
        self._socket = connection
        self._outgoing_text = ""
        self._incoming_text = ""
        self._messages = {}
        self._socket.setblocking(False)
        self._connected = True
        self._message_callback = None

    def fileno(self):
        '''
        Gets fileno of underlying socket, used to allow JsonTcpServerConnection as input to the
        "select" function
        '''
        return self._socket.fileno()

    @property
    def has_data_to_write(self):
        '''Returns whether or not there is data queued to be sent to the client'''
        return len(self._outgoing_text) > 0

    @property
    def address(self):
        '''Gets the address of the connected TCP Client'''
        return self._socket.getpeername()

    def send_message(self, message, channel: str):
        '''
        Sends a message to the TCP client

        Args:
            message (MessageBuilder): A Capnp message to send to the client
            channel (str): The channel to send the message on

        Raises:
            IOError: The client has disconnected
        '''
        if not self.connected:
            raise IOError("Client has disconnected")
        message_uuid = uuid.uuid4()
        serialized_message = {
            "header": {
                "channel": channel,
                "pubtime": str(message.pubtime),
                "acqtime": str(message.acqtime),
                "proto": str(_unsigned_to_signed(message.proto.schema.node.id)),
                "uuid": {
                    "lower": (message_uuid.int >> 0) & ((1 << 64) - 1),
                    "upper": (message_uuid.int >> 64) & ((1 << 64) - 1),
                }
            },
            "payload": _proto_to_dict(message.proto)
        }

        self._outgoing_text += json.dumps(serialized_message) + "\n"

    def set_message_callback(self, callback: Callable[[im.MessageBuilder, str], None]):
        '''Sets a callback function to be called whenever a new message is received'''
        self._message_callback = callback

    def get_channels(self) -> List:
        '''Returns a list of all channels that have unread messages'''
        return self._messages.keys()

    def get_next_message(self, channel: str):
        '''
        Gets the next unread message on the given channel and pops it from the channel's queue.

        Args:
            channel (str): The channel to fetch the next message from

        Returns:
            A MessageBuilder containing the oldest unread message or 'None' if all messages have
            been read

        Raises:
            IOError: The client has disconnected
        '''
        if not self.connected:
            raise IOError("Client has disconnected")

        # Attempt to get the next message from the deque, otherwise return None
        try:
            return self._messages.get(channel, deque()).popleft()
        except IndexError as e:
            return None

    def _process_input_buffer(self):
        '''
        Parses the incoming text buffer, and if any complete messages have been received, converts
        them to capnp messages
        '''
        split_text = self._incoming_text.split("\n")
        new_messages = split_text[:-1]
        self._incoming_text = split_text[-1]

        # For each message, build a capnp message
        for message_text in new_messages:
            # Get the header/payload from the json text
            json_message = json.loads(message_text)
            header = json_message['header']
            payload = json_message['payload']

            # Convert fields that may be stored as string to int
            header["proto"] = int(header["proto"])
            header["acqtime"] = int(header["acqtime"])
            header["pubtime"] = int(header["pubtime"])

            # The proto id is transmitted as int64_t, if its negative it needs to be
            # converted to a uint64_t
            header["proto"] = _signed_to_unsigned(header["proto"])

            # Create a capnp message from the payload
            payload_message = _dict_to_proto(payload, header["proto"])

            # Create an ISAAC style message
            builder = im.MessageBuilder()
            builder.proto = payload_message
            builder.acqtime = header["acqtime"]
            builder.pubtime = header["pubtime"]

            # Put the message into the appropriate message queue based on channel
            channel = header["channel"]
            if channel not in self._messages.keys():
                self._messages[channel] = deque([], MESSAGE_QUEUE_SIZE)
            self._messages[channel].append(builder)

            if self._message_callback is not None:
                self._message_callback(builder, channel)

    def _send(self):
        '''
        Sends bytes until there are no more to send or the socket cannot send anymore without
        blocking
        '''
        try:
            while len(self._outgoing_text) > 0:
                bytes_sent = self._socket.send(self._outgoing_text.encode())
                if bytes_sent == 0:
                    raise IOError("Socket connection down")
                self._outgoing_text = self._outgoing_text[bytes_sent:]
        except BlockingIOError as e:
            pass

    def _recv(self):
        '''
        Receives bytes until the socket cannot receive anymore without blocking
        '''
        try:
            while True:
                data = self._socket.recv(4096)
                if len(data) == 0:
                    raise IOError("Socket connection down")
                self._incoming_text += data.decode()
        except BlockingIOError as e:
            pass

        self._process_input_buffer()

    def update(self):
        '''
        Attempts to read/write any pending bytes on the nonblocking socket
        '''
        try:
            self._send()
            self._recv()
        except IOError as e:
            self.close()

    @property
    def connected(self) -> bool:
        return self._connected

    def close(self):
        self._socket.close()
        self._connected = False


class JsonTcpServer:
    '''
    This class listens on the specified port for connections from TCP Clients and creates a
    JsonTcpServerConnection class to manage new incoming connections. It spawns a background thread
    to asynchronously check for new connections and update the connections themselves
    '''

    def __init__(self,
                 port: int = 9998,
                 new_connection_handler: Callable[[JsonTcpServerConnection], None] = None,
                 update_callback: Callable[[], None] = None) -> None:
        '''
        Creates the JsonTcpServer

        Args:
            port (int): The TCP port to listen for incoming connections on
            new_connection_handler (function): A callback function to be called when there is a new
                                               connection
            update_callback (function): A callback function to be called after the JsonTcpServer
                                        updates all connection objects
        '''

        # Create a TCP server for robots to connect to and recieve missions
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("", port))
        self._socket.setblocking(False)
        self._socket.listen()
        self._new_connection_handler = new_connection_handler
        self._update_callback = update_callback
        self._connections = []

        # Start a thread to listen for connections and pull data from those connections
        self._running = True

        def thread_function():
            self._thread_function()

        self._thread = threading.Thread(target=thread_function)
        self._thread.daemon = True
        self._thread.start()

    @property
    def connections(self) -> List[JsonTcpServerConnection]:
        return self._connections.copy()

    def cleanup(self):
        self._running = False
        self._thread.join()

    def _thread_function(self):
        # The time when the next update should occur
        next_update_callback_time = time.monotonic()

        while self._running:
            # First, get any pending connections
            try:
                while True:
                    connection, address = self._socket.accept()
                    new_connection = JsonTcpServerConnection(connection)
                    self._connections.append(new_connection)
                    if self._new_connection_handler is not None:
                        self._new_connection_handler(new_connection)
            except BlockingIOError:
                pass

            # Wait until either:
            #   - One of the connections is ready to read/written to
            #   - It is time to call the update callback
            timeout = max(0, next_update_callback_time - time.monotonic())
            readable, writable, exception = select.select(self._connections,
                                                          filter(lambda x: x.has_data_to_write,
                                                                 self._connections),
                                                          self._connections, timeout)

            # Update all connections
            for connection in readable + writable:
                connection.update()

            # Remove all connections that are closed
            self._connections = list(filter(lambda c: c.connected, self._connections))

            # Call the update callback function if enough time has elapsed
            current_time = time.monotonic()
            if current_time >= next_update_callback_time:
                next_update_callback_time += UPDATE_CALLBACK_PERIOD
                if self._update_callback is not None:
                    self._update_callback()
