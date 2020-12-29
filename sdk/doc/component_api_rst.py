# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import sys
import argparse
import os
import json


def rst_header_underline(level):
    ''' Gets a underline character for an RST headline of given level (0: highest level) '''
    if level == 0:
        return '='
    if level == 1:
        return '-'
    if level == 2:
        return '~'
    fail("Level too deep")


def rst_header(name, level):
    ''' Creates an RST header with underline '''
    return name + '\n' + (rst_header_underline(level) * len(name)) + '\n'


def is_experimental(c):
    txt = get_comment(c)
    return "@experimental" in txt or "@internal" in txt


def get_name(x):
    return x["type_name"].replace("::", ".")


def get_classname(x):
    return x["type_name"].split("::")[-1]


def get_namespace(x):
    return ".".join(x["type_name"].split("::")[0:-1])


def get_type(c):
    btn = c["base_type_name"]
    if btn == "isaac::alice::Codelet":
        return "Codelet - This component ticks either periodically or when it receives messages."
    elif btn == "isaac::alice::Component":
        return "Component - This component does not tick and only provides certain helper functions."
    else:
        return "Other"


def get_comment(x):
    segment_strs = str(x["comment"]).split('\n\n')
    result_str = ''
    for seg in segment_strs:
        if (seg.lstrip()[:3] == '.. '):
            # Verbatim for special RST controlled style
            result_str += seg
        else:
            trivial = True
            for line in seg.split('\n'):
                if len(line) > 1 and (line[0] != ' ' or line[1] == ' '):
                    trivial = False
                    break
            if trivial:
                # Remove all line break for trivial (all 1 space indent) segment
                result_str += seg.replace('\n', ' ')
            else:
                # Verbatim otherwise
                result_str += seg
        result_str += '\n\n'
    return result_str


def get_parameter_typename(x):
    v = x["type_name"]
    if v == "std::string": return "string"
    if v == "nlohmann::json": return "json"
    else: return v


def get_default_as_string(x):
    if "default" in x:
        return "default={}".format(x["default"])
    else:
        return "default=N/A"


def component_to_rst(component):
    printable_name = get_name(component)

    # Replace :: with .

    header = ".. _{}:\n\n".format(printable_name) + rst_header(printable_name, 1)

    description = r'''**Description**

{}

**Type:** {}
'''.format(get_comment(component), get_type(component))

    rx = "**Incoming messages**"
    if "rx" in component:
        rx += "\n\n"
        for x in component["rx"]:
            rx += "* {} [:ref:`{}`]: {}\n".format(x["tag"], x["type_name"], get_comment(x))
    else:
        rx += "\n  (none)\n"

    tx = "**Outgoing messages**"
    if "tx" in component:
        tx += "\n\n"
        for x in component["tx"]:
            tx += "* {} [:ref:`{}`]: {}\n".format(x["tag"], x["type_name"], get_comment(x))
    else:
        tx += "\n  (none)\n"

    px = "**Parameters**"
    if "params" in component:
        px += "\n\n"
        for x in component["params"]:
            px += "* {} [*{}*] [{}]: {}\n".format(x["key"], get_parameter_typename(x),
                                                  get_default_as_string(x), get_comment(x))
    else:
        px += "\n  (none)\n"

    return header + "\n" + description + "\n" + rx + "\n" + tx + "\n" + px + "\n"


def take_or_empty(x, i):
    if i >= len(x): return ""
    else: return x[i]


def component_index_table_rows(x):
    rx = [y["tag"] for y in x["rx"]] if "rx" in x else []
    tx = [y["tag"] for y in x["tx"]] if "tx" in x else []
    px = [y["key"] for y in x["params"]] if "params" in x else []

    return [[
        get_namespace(x), "`{} <{}_>`__".format(get_classname(x), get_name(x)),
        str(len(rx)),
        str(len(tx)),
        str(len(px))
    ]]

    # # transpose it
    # q = [get_name(x)]
    # rx = [y["tag"] for y in x["rx"]] if "rx" in x else []
    # tx = [y["tag"] for y in x["tx"]] if "tx" in x else []
    # count = max(len(q), len(rx), len(tx))
    # result = []
    # for i in range(count):
    #     result += [[take_or_empty(q, i), take_or_empty(rx, i), take_or_empty(tx, i)]]
    #     if i + 1 < count:
    #         result += [["", "", ""]]
    # print(result)
    # return result

    # return [
    #     get_name(x),
    #     ", ".join([y["tag"] for y in x["rx"]]) if "rx" in x else "",
    #     ", ".join([y["tag"] for y in x["tx"]]) if "tx" in x else ""
    # ]


def components_to_rst(filenames):
    # Get all component JSONs and sort them
    components = []
    for filename in filenames:
        for k, v in json.loads(open(filename, 'r').read()).items():
            components = components + [v]
    components.sort(key=lambda x: x["type_name"])

    # Intro
    intro = r'''
This following section contains a list of all components which are available in Isaac SDK.
For each component, the incoming and outgoing message channels and the corresponding message types
are listed. Additionally, all parameters with their names and types and corresponding default
values are explained.

The following table gives an overview over all components. The columns '# Incoming', '# Outgoing'
and '# Parameters' indicate how many incoming message channels, outgoing message channels and
parameters the corresponding module has.

'''

    # Create a table with all components
    table = [
        '-----',
        ['**Namespace**', '**Name**', "**# Incoming**", "**# Outgoing**", "**# Parameters**"],
        '====='
    ]
    for x in components:
        if is_experimental(x): continue
        table += component_index_table_rows(x)
        table += ['-----']
    index = createRstTable(table)

    # Create RST
    body = ""
    for v in components:
        if is_experimental(v): continue
        body += component_to_rst(v) + "\n\n"

    return ".. _component_api_documentation:" + '\n' + '\n' + \
            rst_header("Component API Overview", 0) + '\n' + \
            intro + '\n' + \
            index + "\n" + \
            rst_header("Components", 0) + "\n" + \
            body + '\n'


def createRstTable(table):
    '''
    `data` is expected to be a list of rows. A row can either be a list of strings or a string.
    A list of strings is interpreted ad the column entries for this row. A string is interpreted as
    a delimter where each character specifies the delimter to use the corresponding row.
    Example: The table
                 ['---', ['a', 'b'], '---']
             will result in the following RST:
                 +---+---+
                 + a | b |
                 +---+---+
    '''

    # Return empty string for empty table
    if len(table) == 0:
        return ''

    # Verify consistend number of columns
    num_cols = len(table[0])
    for row in table:
        if num_cols != len(row):
            raise Exception("Not all rows have the same length: {} vs {}".format(
                num_cols, len(row)))

    # Compute width for each column
    col_width = [0] * num_cols
    for row in table:
        if isinstance(row, str):
            continue
        for i in range(num_cols):
            col_width[i] = max(col_width[i], len(row[i]))

    # Create the table row by row
    text = ""
    for row in table:
        if isinstance(row, str):
            # Create a separator line, for example: '+----+-------+----+'
            text += '+'
            for i in range(num_cols):
                text += (col_width[i] + 2) * row[i] + '+'
        else:
            # Create a data row, for example: '| aa | hello | bb |'
            text += '|'
            for i in range(num_cols):
                text += ' ' + padWithSpace(row[i], col_width[i]) + ' |'
        text += '\n'

    return text


def padWithSpace(text, desired_length):
    ''' Pads text with spaces at the end such that it has the desired length '''
    return text + (desired_length - len(text)) * ' '


# Setup command line parsing
parser = argparse.ArgumentParser(description='Isaac SDK Component API doc generator')
parser.add_argument(
    'inputs',
    metavar='input',
    type=str,
    nargs='+',
    help=
    'List of input *.json files with component API information for which to create documentation')
parser.add_argument('--out', dest='output', help='A file where RST output is written')


def main(argv):
    ''' Example usage: python3 -m create_rst ../*.capnp --out index.rst '''
    args = parser.parse_args()

    print('Parsing', len(args.inputs), 'files:')
    parsed = components_to_rst(args.inputs)

    print('Writing RST to', args.output)
    open(args.output, 'w').write(parsed)


if __name__ == '__main__':
    main(sys.argv)
