'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import sys
import argparse
import pyparsing as pp
import os

main_enums = []
main_structs = []
last_name = ''
last_comment = []


def capnpIsPrimTypes(txt):
    ''' Checks if a type is Cap'n'Proto primitive type '''
    return [ \
      'Void', 'Bool', 'Text', 'Data', \
      'Int8', 'Int16', 'Int32', 'Int64', \
      'UInt8', 'UInt16', 'UInt32', 'UInt64', \
      'Float32', 'Float64' \
    ].count(txt)


def rstHeaderUnderline(level):
    ''' Gets a underline character for an RST headline of given level (0: highest level) '''
    if level == 0:
        return '='
    if level == 1:
        return '-'
    if level == 2:
        return '~'
    fail("Level too deep")


def rstHeader(name, level):
    ''' Creates an RST header with underline '''
    return name + '\n' + (rstHeaderUnderline(level) * len(name)) + '\n'


def rstCapnpCodeBlock(text):
    ''' Creates an RST code block with capnp syntax highlighting '''
    return ".. code-block:: capnp\n\n" + addIndentation(text, 4) + "\n"


def addIndentation(text, level):
    ''' Adds spaces at the beginning of every line '''
    padding = level * ' '
    return ''.join(padding + line for line in text.splitlines(True))


def createToken(text, token, id=None):
    ''' Creates a parsing token based on given id '''
    global main_enums
    global main_structs
    global last_name
    global last_comment
    if id == "cp_struct_name":
        if last_name == '':
            last_name = text
    if id == "cp_comment_struct":
        if last_name == '':
            last_comment.append(text)
    if id == "cp_mainstruct":
        main_structs += [{
            "name": last_name,
            "comment": last_comment,
            "struct": True,
            "contents": rstCapnpCodeBlock(text)
        }]
        last_name = ''
        last_comment = []
        return rstCapnpCodeBlock(text)
    if id == "cp_enum":
        main_enums += [{
            "name": last_name,
            "struct": False,
            "contents": rstCapnpCodeBlock(text)
        }]
        last_name = ''
        return rstCapnpCodeBlock(text)
    if id == "cp_scope":
        return addIndentation(text, 2)
    return text


def createSpan(text, id):
    ''' Creates a span token '''
    return createToken(text, 'span', id)


def createDiv(text, id=None):
    ''' Creates a div token '''
    return createToken(text, 'div', id) + ('\n' if id != "cp_scope" else '')


def createLink(text, href, id):
    ''' Creates a link token with href '''
    # TODO: return '<a id="' + id + '" href="' + href + '">' + text + '</a>'
    return text


def createAnchor(text, name, cssclass):
    ''' Creates an anchor token for a link '''
    # TODO: return '<a class="' + cssclass + '" name="' + name + '">' + text + '</a>'
    return text


def ppToSpan(pptoken, cssclass, transform=None):
    ''' Sets up a pyparsing token to transform into a span token '''
    tf = lambda t: ''.join(t) if transform == None else transform(t)
    return pptoken.setParseAction(lambda t: createSpan(tf(t), cssclass))


def ppToDiv(pptoken, cssclass=None, transform=None):
    ''' Sets up a pyparsing token to transform into a div token '''
    tf = lambda t: ''.join(t) if transform == None else transform(t)
    return pptoken.setParseAction(lambda t: createDiv(tf(t), cssclass))


def ppDel(pptoken):
    ''' Sets up a pyparsing token to be deleted after parsing '''
    return pptoken.setParseAction(lambda t: [])


registeredAnchors = []


def ppToAnchor(t):
    ''' Sets up a pyparsing token to transform into a achor token and saves it's name for later '''
    registeredAnchors.append(t[0])
    return createAnchor(t[0], t[0], 'anchor')


def fieldTypeAction(t):
    ''' An pyparsing action to create a linked field type '''
    name = t[0]
    if capnpIsPrimTypes(name):
        return createSpan(name, 'cp_field_type_prim')
    else:
        return createLink(name, '#' + name, 'cp_field_type_user')


def fieldTypeListAction(t):
    ''' An pyparsing action to create a linked field for a List(...) type '''
    if capnpIsPrimTypes(t[1]):
        return createSpan(''.join(t), 'cp_field_type_prim')
    else:
        return createSpan(t[0], 'cp_field_type_prim') \
            + fieldTypeAction([t[1]]) \
            + createSpan(t[2], 'cp_field_type_prim')


def capnpCreateGrammar():
    ''' Creates the pyparsing grammar for capnproto and creates actions to convert to RST '''

    ws2space = pp.ZeroOrMore(pp.White()).setParseAction(lambda t: ' ')
    nonopt2space = pp.OneOrMore(pp.White()).setParseAction(lambda t: ' ')
    ws2del = ppDel(pp.ZeroOrMore(pp.White()))

    bracket0 = ppToSpan(pp.Empty() + '{\n', 'cp_op_curly_open')
    bracket1 = ppToSpan(pp.Empty() + '}', 'cp_op_curly_close')
    semi = ppToSpan(pp.Empty() + ';', 'cp_op_semi')

    structNameRaw = pp.Word(pp.alphanums)

    structName = ppToSpan(pp.Empty() + structNameRaw, 'cp_struct_name', ppToAnchor)
    fieldName = ppToSpan(pp.Word(pp.alphanums), 'cp_field_name')
    enumName = ppToSpan(pp.Word(pp.alphanums), 'cp_enum_name', ppToAnchor)

    structKeyword = ppToSpan(pp.Empty() + 'struct', 'cp_struct_keyword')
    enumKeyword = ppToSpan(pp.Empty() + 'enum', 'cp_enum_keyword')

    cpid = ppDel('@' + pp.Word(pp.alphanums) + ';')
    ordinal = ppToSpan('@' + ws2del + pp.Word(pp.nums), 'cp_ordinal')

    fieldType = pp.Or([ \
        (pp.Empty() + structNameRaw).setParseAction(fieldTypeAction), \
        ('List(' + structNameRaw + ')').setParseAction(fieldTypeListAction)])
    field = ppToDiv(
        fieldName + ws2space + ordinal + ws2del + ':' + ws2space + fieldType + ws2del + semi,
        'cp_field')
    comment = ppToDiv(ws2del + '#' + ws2space + pp.Word(pp.printables + ' ') + pp.LineEnd(),
                      'cp_comment', lambda t: ''.join(t[:-1]))
    comment_struct = ppToDiv(ws2del + '#' + ws2space + pp.Word(pp.printables + ' ') + pp.LineEnd(),
                             'cp_comment_struct', lambda t: ''.join(t[:-1]))

    enum_field = ppToDiv(ws2del + fieldName + ws2space + ordinal + ws2del + semi + ws2del,
                         'cp_enum_field')
    enum_entry = ws2del + pp.Or([comment, enum_field]) + ws2del
    enum_body = ppToDiv(pp.ZeroOrMore(enum_entry), 'cp_scope')
    enum = ppToDiv(
        pp.ZeroOrMore(comment) + enumKeyword + nonopt2space + enumName + ws2space + bracket0 +
        ws2del + enum_body + ws2del + bracket1, 'cp_enum')

    struct = pp.Forward()
    struct_entry = ws2del + pp.Or([comment, field, enum, struct]) + ws2del
    struct_body = ppToDiv(pp.ZeroOrMore(struct_entry), 'cp_scope')
    struct << ppToDiv(
        pp.ZeroOrMore(comment) + structKeyword + nonopt2space + structName + ws2space + bracket0 +
        ws2del + struct_body + ws2del + bracket1, 'cp_struct')

    mainstruct = pp.Forward()
    mainstruct << ppToDiv(
        pp.ZeroOrMore(comment_struct) + structKeyword + nonopt2space + structName + ws2space +
        bracket0 + ws2del + struct_body + ws2del + bracket1, 'cp_mainstruct')

    using = ppDel(pp.Empty() + 'using import "' + pp.Word(pp.alphanums + "./_") + '".' +
                  pp.Word(pp.alphanums) + ';')

    capnp = ws2del + pp.ZeroOrMore(comment) + cpid + ws2del + pp.ZeroOrMore(
        pp.Or([mainstruct + ws2del, using + ws2del, enum + ws2del]))
    return capnp.leaveWhitespace()


def capnpParse(text, verbose=False):
    ''' Parses a capnproto and converts into RST '''
    tokens = capnpCreateGrammar().parseString(text)
    result = '\n'.join(tokens.asList())
    if verbose:
        print('==========================================================')
        print(text)
        print('==========================================================')
        print(tokens)
        print('==========================================================')
        print(result)
        print('==========================================================')
        print(main_structs)
        print('==========================================================')
    return result


def padWithSpace(text, desired_length):
    ''' Pads text with spaces at the end such that it has the desired length '''
    return text + (desired_length - len(text)) * ' '


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
            fail("Not all rows have the same length")

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


def prepareDescriptionFromCommentLines(comment_lines, length=40):
    '''
    Format the given comment lines into a list of text lines with maximum length but shorter than
    the given maximum `max_length`. Comment lines are expected to start with '# ' and this prefix
    will be removed. Lines will be broken only at word boundaries and ' ' is used as word delimiter.
    If a word is longer than the given `max_length` this single will still form an entry in the
    result.
    '''

    # If not comment is given we return an empty string
    if len(comment_lines) == 0:
        return ['']

    # Concatenate comment lines
    text = ''
    for line in comment_lines:
        u = ''
        if line[0:2] == '# ':
            u = line[2:]
        elif line[1] == '#':
            u = line[1:]
        else:
            fail("Comment need to start with '#'")
        text += u + ' '

    # Split comment text at word boundary
    words = text.split(' ')
    if len(words) == 0:
        fail("This should not happen")

    # Fill up lines by taking words until the line is not longer than the desired line length.
    line = words[0]
    lines = []
    for word in words[1:]:
        if len(line) + len(word) + 1 > length:
            lines.append(line)
            line = word
        else:
            line = line + ' ' + word
    # Don't forget the last potentially short line
    if len(line) > 0:
        lines.append(line)

    return lines


def capnpToRst(filenames):
    ''' Parses multiple capnproto files and creates an RST documentation '''
    global registeredAnchors
    global main_structs
    global main_enums

    # Start the overview table
    toc = ['---', ["Section", "Message", "Description"], '===']
    body = ''

    # Parse all capnp files and collect information
    for filename in filenames:
        print('Parsing', filename)

        # get basename of filename as section title
        tag = os.path.splitext(os.path.basename(filename))[0]
        body += rstHeader(tag, 2) + '\n'

        # Load the code from the file and parse it to collect desired tokens
        text = open(filename, 'r').read()
        current_body = rstHeader(tag, 1) + '\n' + capnpParse(text) + '\n\n'

        # in the sidebar create a top-level list containing all structs/enums/... in the file
        current_messages = sorted(list(set(registeredAnchors)))
        registeredAnchors = []

        # Get the list of all protos sorted by name
        sorted_structs = sorted(main_structs + main_enums, key=lambda kvp: kvp["name"])
        main_structs = []
        main_enums = []

        # Insert rows for the proto section into the table
        first = True
        for data in sorted_structs:
            # Join first column per section
            if not first:
                toc.append(' --')

            if data["struct"]:
                # Reformat comment lines together
                comment = prepareDescriptionFromCommentLines(data["comment"])
                # Three columns per row: section, name, description
                first_comment_line = True
                for line in comment:
                    if first_comment_line:
                        toc.append(['`' + tag + '`_' if first else '', data["name"] + '_', line])
                    else:
                        toc.append(['', '', line])
                    first_comment_line = False
            else:
                toc.append(['`' + tag + '`_' if first else '', '', ''])
            # Append tag, title, and proto definition. For example,
            # .. _Goal2Proto:
            #
            # Goal2Proto
            # -------
            #
            # .. code-block:: capnp
            #
            #     # Describe a goal in a given referential frame
            #     struct Goal2Proto {
            #       # The goal expressed in Pose2
            #       goal @0: Pose2dProto;
            #       # the tolerance radius of the goal.
            #       tolerance @1: Float32;
            #       # name of the frame the goal is.
            #       goalFrame @2: Text;
            #       # Whether or not we should stop the robot. If set to true all the other parameters will be ignored
            #       stopRobot @3: Bool;
            #     }
            #
            body += '.. _{}:\n\n{}\n{}\n\n{}\n'.format(data["name"], data["name"],
                                                       '-' * len(data["name"]), data["contents"])
            first = False

        # Finish the section
        if len(sorted_structs) != 0:
            toc.append('---')

    return ".. _message_api_documentation:" + '\n' + '\n' + \
        rstHeader("Message API Overview", 0) + \
        '\n' + \
        '.. tabularcolumns:: |p{3cm}|p{5cm}|p{7cm}|\n' + \
        '\n' + \
        createRstTable(toc) + \
        '\n' + \
        rstHeader("Protobuffers", 0) + \
        '\n' + \
        body


# Setup command line parsing
parser = argparse.ArgumentParser(description='Isaac SDK Message API doc generator')
parser.add_argument('inputs',
                    metavar='input',
                    type=str,
                    nargs='+',
                    help='List of input *.capnp files for which do create documentation')
parser.add_argument('--out', dest='output', help='A file where RST output is written')


def main(argv):
    ''' Example usage: python3 -m create_rst ../*.capnp --out index.rst '''
    args = parser.parse_args()

    print('Parsing', len(args.inputs), 'files:')
    parsed = capnpToRst(args.inputs)

    print('Writing RST to', args.output)
    open(args.output, 'w').write(parsed)


if __name__ == '__main__':
    main(sys.argv)
