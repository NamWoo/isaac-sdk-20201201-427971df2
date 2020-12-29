'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import argparse
import json
import re
import sys
from os import listdir
from os.path import isfile, join, isdir


# Function that open a files line per line and return the list of the lines
def extract_lines(file):
    lines = []
    with open(file, encoding='UTF-8') as fp:
        line = fp.readline()
        while line:
            lines.append(line)
            line = fp.readline()
    return lines


# Extract the indices of the lines that match the regex
def extract_matching_lines(lines, regex):
    idx = []
    for index in range(0, len(lines)):
        line = lines[index]
        if regex.match(line):
            idx.append(index)
    return idx


# Extracts the parameters of an ISAAC_XXXX(param1, param2, ...) macro
def extract_parameters(lines, line_idx, pos):
    params = []
    line = lines[line_idx]
    brackets = 0
    cur = ''
    while (True):
        if pos == len(lines[line_idx]):
            pos = 0
            line_idx = line_idx + 1
            if line_idx == len(lines):
                raise
            line = lines[line_idx]

        if line[pos] == ',' and brackets == 1:
            params.append(re.sub("\s+", " ", cur.replace('\n', '').replace('\\', '')).strip())
            cur = ''
        elif line[pos] == ')':
            brackets = brackets - 1
            if brackets == 0:
                break
            cur = cur + line[pos]
        elif line[pos] == '(':
            brackets = brackets + 1
            if brackets > 1:
                cur = cur + line[pos]
        else:
            cur = cur + line[pos]
        pos = pos + 1

    params.append(re.sub("\s+", " ", cur.replace('\n', '').replace('\\', '')).strip())
    return params


# Extract the comment that end at the line end
# It assumes each comment starts with // and no empty lines.
def extract_comment_ending(file, lines, end):
    start = end
    ret = ""
    COMMENT_REGEX = "^ *\/\/.*$"
    while start > 0 and re.match(COMMENT_REGEX, lines[start - 1]):
        start = start - 1
        ret = lines[start][lines[start].find("//") + 2:] + ret
    # Check if some comments are broken with empty line:
    while start > 0 and lines[start - 1] == "\n":
        start = start - 1

    if (start > 0 and re.match(COMMENT_REGEX, lines[start - 1])):
        print("Comment potentially missing in file %s: %s" % (file, lines[end]), file=sys.stderr)

    if ret == "":
        print("Cannot extract comment file %s: %s" % (file, lines[end]), file=sys.stderr)
    return ret


# Find all the lines that contains a specific macro
def extract_isaac_macro(file, lines, macro):
    ret = []
    list_idx = extract_matching_lines(lines, re.compile(".*" + macro + "\(.*"))
    for idx in list_idx:
        params = extract_parameters(lines, idx, lines[idx].find(
            '(',
            re.search(macro, lines[idx]).start()))
        ret.append({'comment': extract_comment_ending(file, lines, idx), 'params': params})
    return ret


# Find where a class end
def get_end_line(lines, line_id):
    counter = 0
    while counter == 0:
        counter += lines[line_id].count('{')
        line_id = line_id + 1

    while counter > 0:
        counter += lines[line_id].count('{')
        counter -= lines[line_id].count('}')
        line_id = line_id + 1
    return line_id


# Looks for a given component in a file and extract the information about it
def process_component(file, json_components, macro, comp_name):
    lines = extract_lines(file)
    # Check if the file contain a codelet

    component_idx = extract_matching_lines(lines, re.compile("^" + macro + "\(.*\).*$"))
    for idx in component_idx:
        line = lines[idx]
        # Extract the name
        pattern = macro + "("
        name = line[line.find(pattern) + len(pattern):line.find(")")]

        class_line = extract_matching_lines(
            lines, re.compile("^ *class " + name.split("::")[-1] + "[\ :].*"))
        if class_line is None or len(class_line) == 0:
            continue
        end_line = get_end_line(lines, class_line[0])

        comments = extract_comment_ending(file, lines, class_line[0])
        # If this is an experimental component, we ignore it
        if "@experimental" in comments:
            continue

        json_component = {
            'base_type_name': comp_name,
            'file': file,
            'type_name': name,
            'comment': comments
        }

        list_pose2 = extract_isaac_macro(file, lines[class_line[0]:end_line], "ISAAC_POSE2")
        if len(list_pose2) > 0:
            obj = []
            for pose2 in list_pose2:
                obj.append({
                    'type': 'pose',
                    'lhs': pose2['params'][0],
                    'rhs': pose2['params'][1],
                    'comment': pose2['comment']
                })
            json_component['pose2'] = obj

        list_pose3 = extract_isaac_macro(file, lines[class_line[0]:end_line], "ISAAC_POSE3")
        if len(list_pose3) > 0:
            obj = []
            for pose3 in list_pose3:
                obj.append({
                    'type': 'pose',
                    'lhs': pose3['params'][0],
                    'rhs': pose3['params'][1],
                    'comment': pose3['comment']
                })
            json_component['pose3'] = obj

        list_params = extract_isaac_macro(file, lines[class_line[0]:end_line], "ISAAC_PARAM")
        if len(list_params) > 0:
            obj = []
            for params in list_params:
                obj.append({
                    'type': 'config',
                    'key': params['params'][1],
                    'type_name': params['params'][0],
                    'default': params['params'][2] if len(params['params']) > 2 else '',
                    'comment': params['comment']
                })
            json_component['params'] = obj

        list_rx = extract_isaac_macro(file, lines[class_line[0]:end_line], "ISAAC_.*_RX")
        if len(list_rx) > 0:
            obj = []
            for rx in list_rx:
                obj.append({
                    'type': 'message',
                    'direction': 'rx',
                    'type_name': rx['params'][0],
                    'tag': rx['params'][1],
                    'comment': rx['comment']
                })
            json_component['rx'] = obj

        list_tx = extract_isaac_macro(file, lines[class_line[0]:end_line], "ISAAC_.*_TX")
        if len(list_tx) > 0:
            obj = []
            for tx in list_tx:
                obj.append({
                    'type': 'message',
                    'direction': 'tx',
                    'type_name': tx['params'][0],
                    'tag': tx['params'][1],
                    'comment': tx['comment']
                })
            json_component['tx'] = obj

        json_components[name] = json_component
    return json_components


# Returns the list of c++ file in a folder and its subfolder
def get_cpp_files(path):
    files = []
    for f in listdir(path):
        full_path = join(path, f)
        if isfile(full_path):
            if f.endswith('.hpp') or f.endswith('.h'):
                files.append(full_path)
        elif isdir(full_path):
            files = files + get_cpp_files(full_path)
    return files


# Looks for Codelet/component in a file and extract the information about it
def process_file(file, json_out):
    json_out = process_component(file, json_out, "ISAAC_ALICE_REGISTER_CODELET",
                                 "isaac::alice::Codelet")
    json_out = process_component(file, json_out, "ISAAC_ALICE_REGISTER_COMPONENT",
                                 "isaac::alice::Component")
    return json_out


# Setup command line parsing
parser = argparse.ArgumentParser(description='Isaac SDK Component API JSON extractor')
parser.add_argument(
    'inputs',
    metavar='input',
    type=str,
    nargs='+',
    help='List of input directories with components')
parser.add_argument('--out', dest='output', help='File where component API as JSON is written')
parser.add_argument('--dummy', dest='dummy')


def main():
    ''' Example usage: python3 -m doc/extract_component_info.py packages --out api.json '''
    args = parser.parse_args()

    json_out = {}

    for target in args.inputs:
        if isfile(target):
            json_out = process_file(target, json_out)
        else:
            files = get_cpp_files(target)
            for file in files:
                json_out = process_file(file, json_out)

    if args.output == None:
        print(json.dumps(json_out, indent=2))
    else:
        open(args.output, 'w').write(json.dumps(json_out, indent=2))


if __name__ == '__main__':
    main()
