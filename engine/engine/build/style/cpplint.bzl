"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# We only allow certain source files in isaac
_CPP_ALLOWED_EXTENSIONS = ["c", "cpp", "h", "hpp", "tpp"]

# Do not lint auto-generated cap'n'proto or protocol buffer files
_CPP_IGNORED_EXTENSIONS = ["pb.h", "pb.cc", "capnp.h", "capnp.c++"]

# Do not lint auto-generated cap'n'proto or protocol buffer files
_FILTER_OPTIONS = [
    "-build/c++11",
    "-readability/todo",
    "-runtime/explicit",
    "-runtime/references",
    "-build/header_guard",
    "+build/include_alpha",
]

# Additional arguments to cpplint.py
_CPPLINT_EXTRA_ARGUMENTS = [
    "--extensions=" + ",".join(_CPP_ALLOWED_EXTENSIONS),
    "--linelength=100",
    "--headers=h,hpp",
    "--filter=" + ",".join(_FILTER_OPTIONS),
]

def _is_source_label(file):
    filename = file
    """ Checks if a label is a valid source """
    for extension in _CPP_IGNORED_EXTENSIONS:
        if filename.endswith("." + extension):
            return False
    for extension in _CPP_ALLOWED_EXTENSIONS:
        if filename.endswith("." + extension):
            return True

    # In the rare case that we need to pass a shared library as a dependeny we ignore it. This
    # is for example currently happening for the yolo package.
    if filename.endswith(".so"):
        return True

    fail("Unrecognized extension for source file '%s'" % filename)

def _generate_file_locations_impl(ctx):
    paths = []
    for label in ctx.attr.labels:
        file = label.files.to_list()[0]
        if _is_source_label(file.basename):
            paths.append(file.short_path)
    ctx.actions.write(ctx.outputs.file_paths, "\n".join(paths))
    return DefaultInfo(runfiles = ctx.runfiles(files = [ctx.outputs.file_paths]))

_generate_file_locations = rule(
    implementation = _generate_file_locations_impl,
    attrs = {"labels": attr.label_list(allow_files = True)},
    outputs = {"file_paths": "%{name}_files"},
)

def cpplint(name, srcs):
    file_locations_label = "_" + name + "_file_locations"
    _generate_file_locations(name = file_locations_label, labels = srcs)

    native.py_test(
        name = "_cpplint_" + name,
        srcs = ["@com_nvidia_isaac_engine//engine/build/style:cpplint"],
        data = srcs + [file_locations_label],
        main = "@com_nvidia_isaac_engine//engine/build/style:cpplint.py",
        args = _CPPLINT_EXTRA_ARGUMENTS + ["$(location %s)" % file_locations_label],
        size = "small",
        tags = ["cpplint", "lint"],
        python_version = "PY2",
    )
