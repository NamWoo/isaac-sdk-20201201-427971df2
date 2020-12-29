# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import sys
import argparse
import pyparsing as pp
import os
import json
import graphviz


def create_performance_graph_drawio(app_graph, report):
    ''' Creates the performance report in the draw.io CSV format '''
    header1 = r'''#
##
## Example CSV import. Use ## for comments and # for configuration. Paste CSV below.
## The following names are reserved and should not be used (or ignored):
## id, tooltip, placeholder(s), link and label (see below)
##
#
## Node label with placeholders and HTML.
## Default is '%name_of_first_column%'.
#
## label: %name%<br><i style="color:gray;">%position%</i><br><a href="mailto:%email%">Email</a>
# label: %nname%<br><i style="color:gray;">%cname%</i><br>Avg: %dt% ms, Total: %total% s, Count: %count%
#
## Node style (placeholders are replaced once).
## Default is the current style for nodes.
#
## style: label;image=%image%;whiteSpace=wrap;html=1;rounded=1;fillColor=%fill%;strokeColor=%stroke%;
# style: whiteSpace=wrap;html=1;rounded=1;fillColor=%fill%;strokeWidth=%strokewidth%;
#
## Parent style for nodes with child nodes (placeholders are replaced once).
#
# parentstyle: swimlane;whiteSpace=wrap;html=1;childLayout=stackLayout;horizontal=1;horizontalStack=0;resizeParent=1;resizeLast=0;collapsible=1;
#
## Uses the given column name as the identity for cells (updates existing cells).
## Default is no identity (empty value or -).
#
# identity: -
#
## Uses the given column name as the parent reference for cells. Default is no parent (empty or -).
## The identity above is used for resolving the reference so it must be specified.
#
# parent: -
#
## Adds a prefix to the identity of cells to make sure they do not collide with existing cells (whose
## IDs are numbers from 0..n, sometimes with a GUID prefix in the context of realtime collaboration).
## Default is csvimport-.
#
# namespace: csvimport-
#
## Connections between rows ("from": source colum, "to": target column).
## Label, style and invert are optional. Defaults are '', current style and false.
## In addition to label, an optional fromlabel and tolabel can be used to name the column
## that contains the text for the label in the edges source or target (invert ignored).
## The label is concatenated in the form fromlabel + label + tolabel if all are defined.
## The target column may contain a comma-separated list of values.
## Multiple connect entries are allowed.
#
## connect: {"from": "manager", "to": "name", "invert": true, "label": "manages", \
##          "style": "curved=1;endArrow=blockThin;endFill=1;fontSize=11;"}
## connect: {"from": "refs", "to": "id", "style": "curved=1;fontSize=11;"}
'''

    header2 = r'''#
#
## Node x-coordinate. Possible value is a column name. Default is empty. Layouts will
## override this value.
#
# left:
#
## Node y-coordinate. Possible value is a column name. Default is empty. Layouts will
## override this value.
#
# top:
#
## Node width. Possible value is a number (in px), auto or an @ sign followed by a column
## name that contains the value for the width. Default is auto.
#
# width: auto
#
## Node height. Possible value is a number (in px), auto or an @ sign followed by a column
## name that contains the value for the height. Default is auto.
#
# height: auto
#
## Padding for autosize. Default is 0.
#
# padding: 0
#
## Comma-separated list of ignored columns for metadata. (These can be
## used for connections and styles but will not be added as metadata.)
#
# ignore: id,image,fill,stroke
#
## Column to be renamed to link attribute (used as link).
#
# link: url
#
## Spacing between nodes. Default is 40.
#
# nodespacing: 40
#
## Spacing between levels of hierarchical layouts. Default is 100.
#
# levelspacing: 100
#
## Spacing between parallel edges. Default is 40.
#
# edgespacing: 40
#
## Name of layout. Possible values are auto, none, verticaltree, horizontaltree,
## verticalflow, horizontalflow, organic, circle. Default is auto.
#
# layout: organic
#
## ---- CSV below this line. First line are column names. ----
##name,position,id,location,manager,email,fill,stroke,refs,url,image
##Evan Miller,CFO,emi,Office 1,,me@example.com,#dae8fc,#6c8ebf,,https://www.draw.io,https://cdn3.iconfinder.com/data/icons/user-avatars-1/512/users-9-2-128.png
##Edward Morrison,Brand Manager,emo,Office 2,Evan Miller,me@example.com,#d5e8d4,#82b366,,https://www.draw.io,https://cdn3.iconfinder.com/data/icons/user-avatars-1/512/users-10-3-128.png
##Ron Donovan,System Admin,rdo,Office 3,Evan Miller,me@example.com,#d5e8d4,#82b366,"emo,tva",https://www.draw.io,https://cdn3.iconfinder.com/data/icons/user-avatars-1/512/users-2-128.png
##Tessa Valet,HR Director,tva,Office 4,Evan Miller,me@example.com,#d5e8d4,#82b366,,https://www.draw.io,https://cdn3.iconfinder.com/data/icons/user-avatars-1/512/users-3-128.png
'''
    connections = '# connect: {"from": "refs", "to": "id", "style": "curved=1;fontSize=11;" }\n'

    connect, _ = get_app_graph_connections(app_graph)
    combined_total = get_total_exe_time(app_graph, report)

    # Create the performance graph in form of a CSV table
    csv = "id,nname,cname,count,dt,total,refs,fill,strokewidth\n"
    for v in app_graph["nodes"]:
        nname = v["name"]
        for c in v["components"]:
            cname = c["name"]
            name = nname + "/" + cname

            # Get the performance statistics for the component
            mode, dt, count, total = get_perf_of_compoent(report, name)

            # Make sure that we also have components which are not connected
            if name not in connect: connect[name] = []

            # Don't display components which don't do anything
            if len(connect[name]) == 0 and count == 0: continue

            # Get the desired style for the component
            fill, strokewidth = get_performance_style(dt, total, combined_total, mode)

            # Write one row per component to the CSV file
            csv += '{},{},{},{},{},{},"{}",{},{}\n'.format(
                name, nname, cname, count, round(dt, 1), round(total, 1),
                ",".join(connect[name]), fill, strokewidth)

    return header1 + connections + header2 + csv


def create_performance_graph_graphviz(app_graph, report, outfile, image):
    ''' Creates the performance report in the DOT graph language format, and generates visualization. You can also just xdot (https://pypi.org/project/xdot/) for interactive visualization '''
    connect_to, connect_from = get_app_graph_connections(app_graph)
    combined_total = get_total_exe_time(app_graph, report)

    dot = graphviz.Digraph(comment='Perf Graph', format=image, node_attr={'style': 'filled', 'ratio': 'compress'})
    # force minimal gap between ranks
    dot.attr(ranksep="1")
    # rank nodes left to right instead of top-down
    dot.attr(rankdir='LR')

    for v in app_graph["nodes"]:
        nname = v["name"]

        # add each compute node as a subgraph encloses all its components
        with dot.subgraph(name='cluster_'+nname) as node_subgraph:
            node_subgraph.attr(label=nname)
            node_subgraph.attr(style='filled')
            node_subgraph.attr(color='lightgrey')
            node_subgraph.attr(shape="Mrecord")

            for c in v["components"]:
                cname = c["name"]
                name = nname + "/" + cname

                # Get the performance statistics for the component
                mode, dt, count, total = get_perf_of_compoent(report, name)

                # Make sure that we also have components which are not connected
                if name not in connect_to: connect_to[name] = []
                if name not in connect_from: connect_from[name] = []

                # Don't display components which don't do anything
                if len(connect_to[name]) == 0 and len(connect_from[name]) == 0 and count == 0: continue

                # Get the desired style for the component
                fill, strokewidth = get_performance_style(dt, total, combined_total, mode)

                # add node to the graph.
                cname_pieces = cname.split('.')
                if len(cname_pieces) > 1:
                    # for component that belows to a package, display the package name in smaller font for better visualization
                    node_subgraph.node(dot_name_cleanup(name),
                                       label='<<FONT POINT-SIZE="10">{}</FONT><BR />{}<BR /><FONT POINT-SIZE="10">Avg time/tick: {} ms<BR />Ticks: {}</FONT>>'.format('.'.join(cname_pieces[:-1]), cname_pieces[-1], round(dt, 1), count),
                                       color="{}".format(fill))
                else:
                    node_subgraph.node(dot_name_cleanup(name),
                                       label='<{}<BR /><FONT POINT-SIZE="10">Avg time/tick: {} ms<BR />Ticks: {}</FONT>>'.format(cname_pieces[-1], round(dt, 1), count),
                                       color="{}".format(fill))

                # add edges from current node to the graph.
                for target in connect_to[name]:
                    if target.split('/')[0] is nname:
                        # component of the same node, add edge to subgraph
                        node_subgraph.edge(dot_name_cleanup(name), dot_name_cleanup(target))
                    else:
                        # component of a different node, add edge to graph
                        dot.edge(dot_name_cleanup(name), dot_name_cleanup(target))

    dot.render(outfile, view=True)


def get_perf_of_compoent(report, name):
    if name not in report:
        mode = -1
        dt = 0
        count = 0
        total = 0
    else:
        mode = report[name]["mode"]
        dt = report[name]["avg_time_ms"]
        count = report[name]["count"]
        total = dt * count / 1000.0
    return mode, dt, count, total


def get_name(channel):
    ''' Gets 'nname/cname' from a channel name in the form 'nname/cname/channel '''
    tokens = channel.split('/')
    return tokens[0] + "/" + tokens[1]


def dot_name_cleanup(name):
    ''' replace '/' and '.' in graphviz node name by '_' '''
    return name.replace('/', '_').replace('.', '_')


def is_connected(edges, name):
    ''' Returns true if the component with given name is connected to another component '''
    incoming = False
    outgoing = False
    if name in edges: incoming = True
    for k, v in edges.items():
        for x in v:
            if name in x: outgoing = True
    return incoming and outgoing


def get_app_graph_connections(app_graph):
    ''' Collect edges in form of a dictionary where each source is mapped to a list of targets '''
    connect_to = {}
    connect_from = {}
    for v in app_graph["edges"]:
        src = get_name(v["source"])
        dst = get_name(v["target"])
        if src not in connect_to:
            connect_to[src] = []
        connect_to[src] = connect_to[src] + [dst]
        if dst not in connect_from:
            connect_from[dst] = []
        connect_from[dst] = connect_from[dst] + [src]
    return connect_to, connect_from


def get_total_exe_time(app_graph, report):
    ''' Measure the total time spent for all measured components '''
    combined_total = 0
    for v in app_graph["nodes"]:
        nname = v["name"]
        for c in v["components"]:
            cname = c["name"]
            name = nname + "/" + cname
            if name not in report: continue
            dt = report[name]["avg_time_ms"]
            count = report[name]["count"]
            combined_total += dt * count / 1000.0
    return combined_total


def get_performance_style(dt, total, combined_total, mode):
    ''' Chooses color and stroke width based on performance statistics '''
    if total / combined_total < 0.01: return "#d7ffb7", 1
    if dt < 0.5: return "#d7ffb7", 1
    if mode == 0 or mode == 1: return "#ffffff", 1

    fps = 1000.0 / max(dt, 0.001)
    a = 10.0
    b = 40.0
    if fps < a: p = 0.0
    elif fps > b: p = 1.0
    else: p = (fps - a) / (b - a)

    color = interpolate_color("#004831", "#76B900", p)

    sw = 1
    if total / combined_total > 0.25: sw = 2

    return color, sw


def hex_to_rgb(hex_color):
    '''
    Converts a color in hex formnat to RGB list of integers.
    Example: "#ff0000" => [255, 0, 0]
    '''
    return [int(hex_color[i:i + 2], 16) for i in [1, 3, 5]]


def int8ub_to_hex2(x):
    ''' Converts an integer in the range [0, 255] to two-digit hex format '''
    if x < 0: return "00"
    elif x < 16: return "0{0:x}".format(x)
    elif x < 256: return "{0:x}".format(x)
    else: return "ff";


def rgb_to_hex(rgb_color):
    '''
    Converts an RGB color in form of a list of integer to hex formnat
    Example: [255, 0, 0] => "#ff0000"
    '''
    return "#" + "".join([int8ub_to_hex2(int(v)) for v in rgb_color])


def interpolate_color(color_1_hex, color_2_hex, p):
    ''' Interpolates between two hex colors and returns a hex color '''
    color_1_rgb = hex_to_rgb(color_1_hex)
    color_2_rgb = hex_to_rgb(color_2_hex)
    rgb = [int(float(color_1_rgb[0]) * (1.0 - p) + float(color_2_rgb[0]) * p),
           int(float(color_1_rgb[1]) * (1.0 - p) + float(color_2_rgb[1]) * p),
           int(float(color_1_rgb[2]) * (1.0 - p) + float(color_2_rgb[2]) * p)]
    return rgb_to_hex(rgb)


# Setup command line parsing
parser = argparse.ArgumentParser(description='Isaac SDK Perf Visualization Tool')
parser.add_argument('--report', dest='report',
                   help='Performance report written the by Isaac application')
parser.add_argument('--out', dest='output',
                   help='Filename for output performance graph')
parser.add_argument('--format', dest='format', default='graphviz_dot', choices = ['graphviz_dot', 'draw_io_csv'],
                   help='Format of output graph file')
parser.add_argument('--image', dest='image', default='pdf', choices = ['pdf', 'svg', 'png'],
                   help='Format of the output graph image for graphviz (ignore for draw_io)')


def main(argv):
    ''' Example usage:
    python3 engine/alice/tools/create_performance_graph.py --report scheduler_report.json --out performance_graph.dot [--format graphviz_dot]
    '''
    args = parser.parse_args()

    report_json = json.loads(open(args.report, "r").read())

    if args.format == "graphviz_dot":
            create_performance_graph_graphviz(report_json["app_graph"], report_json["perf"], args.output, args.image)
    elif args.format == "draw_io_csv":
        open(args.output, 'w').write(
            create_performance_graph_drawio(report_json["app_graph"], report_json["perf"]))

if __name__ == '__main__':
    main(sys.argv)