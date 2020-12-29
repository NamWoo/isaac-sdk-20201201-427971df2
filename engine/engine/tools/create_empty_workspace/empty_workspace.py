'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import os
import os.path
import sys
import shutil
from string import Template

kTemplateFilename = "engine/engine/tools/create_empty_workspace/empty_workspace.tpl"
kWorkspaceFilename = "WORKSPACE"
kBazelResourceFilename = ".bazelrc"


def main():
    """Creates new WORKSPACE file and bazel resource file in current folder which resemble
    an empty bazel workspace that depends on Isaac SDK.
    """

    # Get desired target workspace directory
    if len(sys.argv) != 2:
        print("Usage: python empty_workspace.py MY_WORKSPACE_DIRECTORY")
        sys.exit(2)
    target = sys.argv[1]

    # Get the isaac diretory
    self_folder_name, _ = os.path.split(os.path.abspath(__file__))
    tpl_folder_name = kTemplateFilename[:kTemplateFilename.rfind('/')]
    isaac_folder_name = self_folder_name[:len(self_folder_name) - len(tpl_folder_name)]
    isaac_path = os.path.abspath(isaac_folder_name)

    if isaac_path.find(os.path.abspath(target)) != -1:
        print("ERROR: Directory for new workspace can not be the current Isaac directory")
        return

    # Create target directory
    if os.path.exists(target):
        assert os.path.isdir(target)
    else:
        try:
            os.makedirs(target)
            os.makedirs(target + "/build")
        except OSError:
            print("ERROR: Creation of workspacce directory '%s' failed.\n"
                  "Make sure you have access rights and the directory does not already exist." %
                  target)
            sys.exit(2)

    # Create WORKSPACE file from template
    with open(isaac_path + "/" + kTemplateFilename) as file:
        template = Template(file.read())
    workspace = template.substitute(dict(isaac_path=isaac_path))
    with open(target + "/" + kWorkspaceFilename, 'w') as file:
        file.write(workspace)

    # Copy bazel.rc file to new workspace
    shutil.copy(isaac_path + "/" + kBazelResourceFilename, target + "/" + kBazelResourceFilename)

    # Copy isaac.bzl
    with open(isaac_path + "/engine/engine/tools/create_empty_workspace/isaac.bzl.tpl") as file:
        template = Template(file.read())
    workspace = template.substitute(dict())
    with open(target + "/build/isaac.bzl", 'w') as file:
        file.write(workspace)

    # Copy BUILD.bzl
    with open(isaac_path + "/engine/engine/tools/create_empty_workspace/BUILD.tpl") as file:
        template = Template(file.read())
    workspace = template.substitute(dict())
    with open(target + "/build/BUILD", 'w') as file:
        file.write(workspace)

    print("Successfully created new workspace in " + target)


if __name__ == '__main__':
    main()
