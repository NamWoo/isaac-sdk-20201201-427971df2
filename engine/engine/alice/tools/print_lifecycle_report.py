'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import redis
import argparse
from tabulate import tabulate


# Maps a status id to a descriptive name
STATUS_ID_TO_STR = {
    '0': 'N/A',
    '1': 'Constructed',
    '10': 'Pre Start',
    '11': 'Started',
    '30': 'Pre Stopped',
    '31': 'Stopped',
    '40': 'Deinitialized'
}


def print_lifecycle_report(db, app_uuid):
    '''Reads the lifecycle reports for an application from redis and prints it on the console'''
    print('Printing lifecycle report for application {}'.format(app_uuid))

    # Get the number of entries in the lifecycle report
    num = int(db.get('isaac:{}:lifecycle:count'.format(app_uuid)))

    # Read the whole lifecycle report keeping only the latest entry
    data = {}
    for i in range(num):
        result = db.hgetall('isaac:{}:lifecycle:{:05d}'.format(app_uuid, i))

        # Use empty string for component name if not present
        node_name = result['node']
        if 'component' in result:
            component_name = result['component']
            key = node_name + '/' + component_name
        else:
            component_name = ''
            key = node_name

        data[key] = [node_name, component_name, STATUS_ID_TO_STR[result['new']],
                     float(result['ts']) / 1000000000.0]

    # Create list of key and values and sort by key
    data = [[k, v] for k, v in data.items()]
    data.sort(key = lambda x: x[0])

    # Create list of only values
    data = [v[1] for v in data]

    print(tabulate(data, headers=['node', 'component', 'status', 'time']))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Print application lifecycle report')
    parser.add_argument('--host', type=str, default='localhost', help='IP of redis server')
    parser.add_argument('--port', type=int, default=6379, help='Port of redis server')
    parser.add_argument('--uuid', type=str, help='UUID of application for which to print report.'
                        'If not specified the latest application will be chosen.')
    args = parser.parse_args()

    # Connect to the redis server
    db = redis.StrictRedis(host=args.host, port=args.port, decode_responses=True)

    if args.uuid != None:
        app_uuid = args.uuid
    else:
        # Find the UUID of the latest run
        app_uuid = db.lrange('isaac:runs', -1, -1)[0]

    print_lifecycle_report(db, app_uuid)
