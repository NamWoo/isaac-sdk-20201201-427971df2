#!/usr/bin/env python
'''
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''
import os
import re

# Read the complete git log for the currently active branch and stores it in `git_log`.
git_log = os.popen('git log').read()

def process_commit(commit_hash, commit_content):
    '''Extracts commit message and metadata if message contains 'breaking change'.'''
    # The 'Author:' and 'Date:' lines have fixed positions. Extract their content here.
    author = commit_content[0][8:]
    date = commit_content[1][8:]
    message = []

    # Go through the content of the commit, with an offset by two lines as these are the 'Author:'
    # and 'Date:' lines.
    for line in commit_content[2:]:
        # Look for the change id; every commit should contain one. This is recorded for possible
        # later reference to find out which CL contained the breaking change.
        if line[4:14] == 'Change-Id:':
            change_id = line[14:].strip()
            # The change id is the last line considered for this commit.
            break
        else:
            # Everything that is not the change id is part of the message.
            stripped_line = line.strip()
            if len(stripped_line) > 0:
                message.append(stripped_line)

    # Look through all lines of the message. If one contains the string 'breaking change'
    # (case-insensitive) then it is a breaking change.
    is_breaking_change = False
    for line in message:
        breaking_change_match = re.search(r'breaking change', line, re.IGNORECASE)

        if breaking_change_match is not None:
            is_breaking_change = True
            break

    # If a 'breaking change' string was found, return all recorded data.
    if is_breaking_change:
        return {
            'sha': commit_hash,
            'author': author,
            'date': date,
            'message': message
        }

    return None

breaking_changes = []
commit_content = []
commit_hash = None
# Separate the `git_log` output by lines of the form `commit <sha>`. Each line will be processed as
# individual commit message.
for line in git_log.splitlines():
    commit_match = re.search(r'^commit ([a-f0-9]+)', line)

    if not commit_match == None:
        if commit_hash is not None and len(commit_content) > 0:
            breaking_change = process_commit(commit_hash, commit_content)

            if breaking_change is not None:
                breaking_changes.append(breaking_change)

        commit_hash = commit_match.group(1)
        commit_content = []
    else:
        commit_content.append(line)

# As output, the commit sha, the author, the date and the commit message of each breaking change
# message are concatenated, separated by newlines. This is printed to the console for further
# processing.
output_buffer = ""
for breaking_change in breaking_changes:
    message = "\n".join(breaking_change['message'])
    output_buffer += "---\ncommit {}\nAuthor: {}\nDate:   {}\n\n{}\n\n\n".format(
        breaking_change['sha'],
        breaking_change['author'],
        breaking_change['date'],
        message
    )

print(output_buffer)
