'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.

'''

from isaac import Cask
from packages.pyalice import Message, Application

import xml.etree.ElementTree as et
import json
import os
import uuid
import time
import argparse
"""
Converts a CVAT XML with human labeled bounding boxes to a cask containing Detections2 messages.
Assumes labels are saved in CVAT format and provided by NV labeling team.
"""


def slice_detections(all_ground_truth_bboxes, slice_mode):
    """ Find indices of detections of interest and modify the ground truth detections based on
        slice mode.
        Params:
            all_ground_truth_boxes (List of xml.etree.ElementTree.Element): List of all ground
                truth bounding boxes found in XML
            slice_mode (str): Slice mode that determines which indices to slice and return
        Returns:
            sliced_ground_truth_boxes (List of xml.etree.ElementTree.Element): List of sliced and
                modified ground truth bounding boxes
    """
    sliced_ground_truth_boxes = []
    if slice_mode == "dolly":
        # This mode returns all non-reflection full dolly detections and renames them "dolly"
        # from "Dolly-Full"
        for i in range(len(all_ground_truth_bboxes)):
            box = all_ground_truth_bboxes[i]
            row = {a.attrib['name']: a.text for a in box.findall("./attribute")}
            row.update(box.attrib)
            if row['label'] == 'Dolly-Full' and row['Reflection'] == 'false':
                box.attrib['label'] = "dolly"
                sliced_ground_truth_boxes.append(box)
    else:
        sliced_ground_truth_boxes = all_ground_truth_bboxes

    return sliced_ground_truth_boxes


def main(args):
    # Read CVAT XML file
    cvat_xml_path = args.cvat_xml
    if os.path.exists(cvat_xml_path):
        tree = et.parse(cvat_xml_path)
    else:
        print("Please provide a valid XML file from CVAT.")
        return

    # Get image cask UUID that these labels are associated with
    image_cask_uuid = cvat_xml_path.split('/')[-1].split('.')[0]

    # Start application to record
    app = Application()
    # Add a dummy node to publish the constructed Detections2 messages from
    app.add("node")
    message_ledger = app.nodes["node"].components["MessageLedger"]
    # Load record subgraph and configure
    app.load("packages/cask/apps/record.subgraph.json", prefix="record")
    record_interface = app.nodes["record.interface"].components["input"]
    record_interface.config.base_directory = args.base_directory_gt
    # Connect output of dummy node to recorder
    app.connect(message_ledger, 'in', record_interface, 'bounding_boxes')
    app.start()

    # Loop through each image element in the XML tree
    count = 0
    for image in tree.findall("./image"):
        # "Name" attribute corresponds to the image filepath that was input to the CVAT labeling
        # tool. Convention is: <image_cask_uuid>/<channel>/<acqtime>.png
        image_uuid, channel, png = image.attrib['name'].split('/')

        # Check that the image_uuid corresponds to the one specified by the XML filename
        if (image_uuid != image_cask_uuid): continue

        # Extract the acquisition time
        acqtime = int(png.lstrip('0').split('.')[0])

        # S the detections of interest for this image
        all_ground_truth_bboxes = image.findall("./box")
        sliced_ground_truth_boxes = slice_detections(all_ground_truth_bboxes, args.slice_mode)
        num_sliced_ground_truth_boxes = len(sliced_ground_truth_boxes)

        # Build Detections2Proto message
        detections2 = Message.create_message_builder('Detections2Proto')
        detections2.acqtime = acqtime
        detections2.uuid = str(uuid.uuid1())
        predictions = detections2.proto.init('predictions', num_sliced_ground_truth_boxes)
        bounding_boxes = detections2.proto.init('boundingBoxes', num_sliced_ground_truth_boxes)

        # Populate the Detections2Proto and PredictionProto messages per sliced boudning box
        for i in range(num_sliced_ground_truth_boxes):
            box = sliced_ground_truth_boxes[i]
            row = {a.attrib['name']: a.text for a in box.findall("./attribute")}
            row.update(box.attrib)

            prediction = predictions[i]
            prediction.label = row['label']
            prediction.confidence = 1.0

            bbox = bounding_boxes[i]
            bbox.min.y = float(row['xtl'])
            bbox.min.x = float(row['ytl'])
            bbox.max.y = float(row['xbr'])
            bbox.max.x = float(row['ybr'])

        # Publish the message to the node being recorded
        app.publish('node', 'MessageLedger', 'in', detections2)
        recv_msg = app.receive('node', 'MessageLedger', 'in')
        count += 1
        time.sleep(0.1)    #sleep to make sure we don't lose any messages

    app.stop()
    print("Wrote " + str(count) + " messages")

    # Write metadata to JSON data per output cask. The metadata servers to associate
    # corresponding image and ground truth casks. As per RACI evaluation workflow
    # and data management, image casks and ground truth casks are stored in separate
    # directories.
    if args.raci_metadata:
        # Populate ground truth cask metadata
        ground_truth_metadata_json = {}
        ground_truth_metadata_json["Image_Cask_File"] = image_cask_uuid
        ground_truth_metadata_json["Data_Source"] = "ground_truth"

        # Write ground truth cask metadata
        ground_truth_metadata_path = os.path.join(args.base_directory_gt, app.uuid + "_md.json")
        with open(ground_truth_metadata_path, 'w') as f:
            json.dump(ground_truth_metadata_json, f, indent=2)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert a CVAT XML file to a cask')
    parser.add_argument('--cvat_xml',
                        dest='cvat_xml',
                        required=True,
                        help='Path to the input CVAT XML file')
    parser.add_argument('--slice_mode',
                        dest='slice_mode',
                        choices=['all', 'dolly'],
                        default='all',
                        help='Slicing mode to determine which detections to extract')
    parser.add_argument('--base_directory_gt',
                        dest='base_directory_gt',
                        default='/tmp/data/ground_truth',
                        help='Location to save gt cask')
    parser.add_argument('--raci_metadata', dest='raci_metadata', action='store_true')
    parser.add_argument('--no_raci_metadata', dest='raci_metadata', action='store_false')
    parser.set_defaults(raci_metadata=True)
    args, _ = parser.parse_known_args()

    main(args)