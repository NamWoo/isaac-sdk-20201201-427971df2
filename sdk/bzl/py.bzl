"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//bzl:module.bzl", "isaac_pkg")

def _expand_module_dep(target):
    """
    Expand the target name into a valid link to a shared library following these rules:
      1) foo               => //packages/foo:libfoo_module.so
      2) //packages/foo     => //packages/foo:libfoo_module.so
      3) //packages/foo:bar => //packages/foo:libbar_module.so
    """

    # Split by : to get path and target name
    tokens = target.split(":")
    if len(tokens) == 1:
        path = target
        name = target
        has_colon = False
    elif len(tokens) == 2:
        path = tokens[0]
        name = tokens[1]
        has_colon = True
    else:
        print("Invalid token:", target)
        return ""

    # Split path by '/' to get full path
    tokens = path.split("/")
    if len(tokens) == 1:
        path = "//packages/" + path
    if not has_colon:
        name = tokens[-1]

    return path + ":lib" + name + "_module.so"

def isaac_py_app(name, modules = [], data = [], deps = [], srcs = [], tags = [], excludes = [], remap_paths = {}, **kwargs):
    '''
    Defines a default Isaac application for Python API. `modules` gives an easy way to specify
    modules which are found in the //packages folder. Specifying a module here has the effect
    that it will be built and put in the runfiles tree. Also defines a corresponding deployment
    package with prefix "-pkg".
    '''

    # Convert a list of modules into data dependencies using the following rules:
    module_sos = [_expand_module_dep(x) for x in modules]

    if "//packages/pyalice" not in deps and "@com_nvidia_isaac_sdk//packages/pyalice" not in deps:
        deps = deps + ["@com_nvidia_isaac_sdk//packages/pyalice"]
    native.py_binary(
        name = name,
        visibility = ["//visibility:public"],
        srcs = srcs if len(srcs) > 0 else srcs + [name + ".py"],
        data = module_sos + data,
        deps = deps,
        tags = tags,
        **kwargs
    )

    isaac_pkg(
        excludes = excludes,
        tags = tags + ["manual", "pkg"],
        name = name + "-pkg",
        srcs = [name],
        remap_paths = remap_paths,
    )

def _isaac_jupyter_runscript_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.out,
        substitutions = {
            "{NOTEBOOK_FILE}": ctx.file.notebook.path,
        },
    )

_isaac_jupyter_runscript = rule(
    implementation = _isaac_jupyter_runscript_impl,
    output_to_genfiles = True,
    attrs = {
        "notebook": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "template": attr.label(
            default = Label("//packages/pyalice:jupyter_launcher.sh.tpl"),
            allow_single_file = True,
        ),
    },
    outputs = {"out": "run_%{name}"},
)

def isaac_jupyter_app(name, notebook, modules = [], data = [], excludes = [], tags = [], remap_paths = [], **kwargs):
    """
    Defines a shell application that launches Jupyter notebook. `modules` gives an easy way to specify
    modules which are found in the //packages folder. Specifying a module here has the effect
    that it will be built and put in the runfiles tree. Also defines a corresponding deployment
    package with prefix "-pkg".

    """
    module_sos = [_expand_module_dep(x) for x in modules]
    script = str(name) + ".sh"
    _isaac_jupyter_runscript(
        name = script,
        notebook = notebook,
    )

    native.sh_binary(
        name = name,
        srcs = [script],
        data = [Label("//packages/pyalice"), notebook] + module_sos + data,
        tags = tags,
        **kwargs
    )

    isaac_pkg(
        excludes = excludes,
        tags = tags + ["manual", "pkg"],
        name = name + "-pkg",
        srcs = [name],
        remap_paths = remap_paths,
    )
