"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("@com_nvidia_isaac_engine//bzl:isaac_engine.bzl", "isaac_cc_library")

def isaac_cc_module(
        name,
        srcs = [],
        hdrs = [],
        visibility = None,
        deps = [],
        tags = [],
        **kwargs):
    """
    Creates an Isaac module. A module is a shared library which contains one or more components
    which can be used by Isaac applications. Due to conflicts with shared objects created for
    testing this currently names the shared object as libXXX_module.so where XXX is the desired
    name of the module.
    """

    default_deps = [
        Label("@com_nvidia_isaac_engine//engine/alice"),
        Label("@com_nvidia_isaac_engine//engine/core"),
        Label("//messages"),
    ]
    for x in default_deps:
        if x in deps:
            deps.remove(x)
    deps = deps + default_deps

    modlib_name = "_modlib_" + name

    isaac_cc_library(
        name = modlib_name,
        srcs = srcs,
        hdrs = hdrs,
        deps = deps,
        tags = tags,
        alwayslink = True,
        **kwargs
    )

    if visibility == None:
        visibility = ["//visibility:public"]

    # Create the shared library for the module. We depend on the module loader from Robot Engine.
    native.cc_binary(
        name = "lib" + name + "_module.so",
        visibility = visibility,
        deps = [Label("@com_nvidia_isaac_engine//engine/alice/tools:module")] + deps + [modlib_name],
        tags = tags,
        linkshared = True,
        **kwargs
    )

def _snakeCaseToUpperCamelCase(name):
    result = ""
    for x in name.split("_"):
        result += x.title()
    return result

def isaac_component(name, filename = None, deps = [], visibility = [], **kwargs):
    """
    A shortcut to create a library for a single component. Ideally every component should be in its
    own library to avoid entangled link dependency. `name` is the name of the component and this
    library in snake case. The source and header files are automatically referenced by using the
    same name but using UpperCamelCase. The libraries alice, core and messages are automatically
    added as link dependencies.
    For example: isaac_component(name = "dummy_image_viewer", deps = [...]) will use the two files
    DummyImageViewer.hpp and DummyImageViewer.cpp.
    """
    if filename == None:
        filename = _snakeCaseToUpperCamelCase(name)
    isaac_cc_library(
        name = name,
        srcs = [filename + ".cpp"],
        hdrs = [filename + ".hpp"],
        visibility = visibility,
        alwayslink = True,
        deps = deps + [
            Label("@com_nvidia_isaac_engine//engine/alice"),
            Label("@com_nvidia_isaac_engine//engine/core"),
            Label("//messages"),
        ],
        **kwargs
    )

def isaac_component_group(name, components = [], deps = [], visibility = [], **kwargs):
    """
    TODO
    """
    isaac_cc_library(
        name = name,
        srcs = [_snakeCaseToUpperCamelCase(x) + ".cpp" for x in components],
        hdrs = [_snakeCaseToUpperCamelCase(x) + ".hpp" for x in components],
        visibility = visibility,
        deps = deps + [
            Label("@com_nvidia_isaac_engine//engine/alice"),
            Label("@com_nvidia_isaac_engine//engine/core"),
            Label("//messages"),
        ],
        **kwargs
    )

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

def isaac_subgraph(name, subgraph, modules = [], data = [], **kwargs):
    """
    Defines an Isaac subgraph which can be used as building block in an Isaac application.
    An application can depend on a subgraph by adding the subgraph target to the `data` argument.
    See isaac_app on how to depend on modules.
    """
    module_sos = [_expand_module_dep(x) for x in modules]

    native.filegroup(
        name = name,
        data = module_sos + [subgraph] + data,
        **kwargs
    )

def isaac_graph(name, graph = None, modules = [], data = [], **kwargs):
    """
    Defines an Isaac subgraph which can be used as building block in an Isaac application.
    An application can depend on a subgraph by adding the subgraph target to the `data` argument.
    See isaac_app on how to depend on modules.
    """
    if graph == None:
        graph = name + ".json"

    module_sos = [_expand_module_dep(x) for x in modules]

    native.cc_test(
        name = "_" + name + "_" + graph + "_check_json",
        args = ["$(location %s)" % graph],
        deps = [Label("@com_nvidia_isaac_engine//engine/alice/tools:check_json")],
        data = [graph] + module_sos + data,
        **kwargs
    )

    native.filegroup(
        name = name,
        data = module_sos + [graph] + data,
        **kwargs
    )

def _isaac_app_runscript_impl(ctx):
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.out,
        substitutions = {
            "{APP_JSON_FILE}": ctx.file.app_json_file.path,
        },
    )

_isaac_app_runscript = rule(
    implementation = _isaac_app_runscript_impl,
    output_to_genfiles = True,
    attrs = {
        "script_name": attr.string(mandatory = True),
        "app_json_file": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
        "template": attr.label(
            default = Label("@com_nvidia_isaac_engine//engine/alice/tools:run.sh.tpl"),
            allow_single_file = True,
        ),
    },
    outputs = {"out": "run_%{script_name}"},
)

def isaac_app(name, modules = [], app_json_file = None, data = [], script = None, tags = [], excludes = [], remap_paths = {}, **kwargs):
    '''
    Defines a default Isaac application. The application depends on a couple of modules and is
    started via an app JSON file. `modules` gives an easy way to specify modules which are found
    in the //packages folder. `sos` can be used to specify additional modules via full labels.
    Specifying a module here has the effect that it will be built and put in the runfiles tree. At
    execution time all modules specified in the app JSON file need to be available. This target
    also defines a corresponding deployment package with prefix "-pkg".
    '''
    if app_json_file == None:
        app_json_file = name + ".app.json"

    # Convert a list of modules into data dependencies using the following rules:
    module_sos = [_expand_module_dep(x) for x in modules]

    if script == None:
        script = "_script_" + name
        _isaac_app_runscript(
            name = script,
            script_name = name,
            app_json_file = app_json_file,
        )

    native.sh_binary(
        tags = tags,
        name = name,
        srcs = [script],
        data = [Label("@com_nvidia_isaac_engine//engine/alice/tools:main")] + [app_json_file] + module_sos + data,
        **kwargs
    )

    native.cc_test(
        tags = tags,
        name = "_" + name + "_check_json",
        args = ["$(location %s)" % app_json_file],
        deps = [Label("@com_nvidia_isaac_engine//engine/alice/tools:check_json")],
        data = [app_json_file] + module_sos + data,
        **kwargs
    )

    isaac_pkg(
        excludes = excludes,
        tags = tags + ["manual", "pkg"],
        name = name + "-pkg",
        srcs = [name],
        remap_paths = remap_paths,
    )

def _isaac_pkg_impl(ctx):
    arg_compression = ""

    # Analyse desired compression
    if ctx.attr.extension:
        dotPos = ctx.attr.extension.find(".")
        if dotPos > 0:
            dotPos += 1
            suffix = ctx.attr.extension[dotPos:]
            if suffix == "gz":
                arg_compression = "--gzip"
            elif suffix == "bz2":
                arg_compression = "--bzip2"
            else:
                fail("Unsupported compression '%s'" % ctx.attr.extension)

    files = depset()

    # Collect all datafiles of all data
    for dep in ctx.attr.data:
        if hasattr(dep, "data_runfiles"):
            files = depset(transitive = [files, dep.data_runfiles.files])

    # Collect all runfiles of all dependencies
    for dep in ctx.attr.srcs:
        if hasattr(dep, "default_runfiles"):
            files = depset(transitive = [files, dep.default_runfiles.files])
    files = files.to_list()

    exc_files = depset()
    for dep in ctx.attr.excludes:
        if hasattr(dep, "default_runfiles"):
            exc_files = depset(transitive = [exc_files, dep.default_runfiles.files])
    exc_files = exc_files.to_list()

    for f in exc_files:
        if not f.is_source and f in files:
            files.remove(f)

    # Create a space-separate string with paths to all files
    file_list = " ".join([f.path for f in files])

    # Setup a rule to move files from bazel-out to the root folder
    bazel_out_rename = "--transform='flags=r;s|bazel-out/k8-opt/bin/||' " + \
                       "--transform='flags=r;s|bazel-out/k8-fastbuild/bin/||' " + \
                       "--transform='flags=r;s|bazel-out/aarch64-opt/bin/||' " + \
                       "--transform='flags=r;s|bazel-out/aarch64-fastbuild/bin/||' " + \
                       "--transform='flags=r;s|" + ctx.attr.strip_prefix + "||' "

    # Additional replacement rules
    for key, value in ctx.attr.remap_paths.items():
        bazel_out_rename += "--transform='flags=r;s|%s|%s|' " % (key, value)

    # Create the tar archive
    ctx.actions.run_shell(
        command = "tar --hard-dereference %s %s  -chf %s %s" %
                  (arg_compression, bazel_out_rename, ctx.outputs.out.path, file_list),
        inputs = files,
        outputs = [ctx.outputs.out],
        use_default_shell_env = True,
    )

# A rule which creates a tar package with the executable and all necessary runfiles compared to
# pkg_tar which needs manual dependency tracing.
_isaac_pkg = rule(
    implementation = _isaac_pkg_impl,
    executable = False,
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "data": attr.label_list(allow_files = True),
        "extension": attr.string(default = "tar"),
        "strip_prefix": attr.string(default = ""),
        "remap_paths": attr.string_dict(),
        "excludes": attr.label_list(allow_files = True),
    },
    outputs = {
        "out": "%{name}.%{extension}",
    },
)

def isaac_pkg(name, srcs, data = [], remap_paths = {}, excludes = [], **kwargs):
    '''
    Creates a package containing all targets specified in `srcs`. `isaac_pkg` tracks transitive
    dependencies automatically and thus will include all necessary runfiles. The package also
    includes a script "run" which can be used to conveniently run applications.
    '''
    remap_paths_copy = {"run/run": "run"}
    remap_paths_copy.update(remap_paths)
    _isaac_pkg(
        name = name,
        srcs = srcs + [Label("@com_nvidia_isaac_sdk//run:run_script")],
        data = data,
        remap_paths = remap_paths_copy,
        excludes = excludes,
        **kwargs
    )
