'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

def _capnpc_impl(ctx):
    '''
    A custom rule to compile C++ code from a capnp schema file. It calls the capnpc tool which
    is compiled from the cap'n'proto repository to generate source files for the given capnp file.
    '''

    # We can only handle capnp files
    if ctx.file.proto.extension != "capnp":
        fail("Invalid extension:", ctx.file.proto.extension)

    # Get the directory in which we are writing the auto generated C++ source
    outdir = ctx.outputs.outs[0].root.path

    # Run the capnpc tool to generate C++ source
    ctx.actions.run(
        execution_requirements = {"no-sandbox" : "1"},
        tools= [ctx.file.capnpc_cpp],
        inputs = [ctx.file.proto],
        outputs = ctx.outputs.outs,
        arguments = [
            "compile",
            "--src-prefix=.",
            "-o" + ctx.file.capnpc_cpp.path + ":" + outdir,
            ctx.file.proto.path],
        progress_message = "Compiling cap'n'proto...",
        executable = ctx.executable.capnpc_tool,
        use_default_shell_env = False)

    return struct(proto = struct(srcs = [ctx.file.proto]))

# A custom rule to compile C++ code from a capnp schema file
_capnpc = rule(
    implementation = _capnpc_impl,
    output_to_genfiles = True,
    attrs = {
        "proto": attr.label(mandatory=True, allow_single_file=True),
        "outs": attr.output_list(),
        "capnpc_tool": attr.label(executable=True, cfg="host", allow_single_file=True,
                                  default=Label("@capnproto//:capnpc_tool")),
        "capnpc_cpp": attr.label(executable=True, cfg="host", allow_single_file=True,
                                 default=Label("@capnproto//:capnpc-c++"))
    },
)

def cc_capnp_library(name, protos, deps=[]):
    '''
    A custom bazel rule to compile cap'n'protos into a cc_library. The created library will have the
    name specified in this rule. It contains a generated C++ header file which can be used to read
    and write protos.
    '''

    # Create a target for every proto file
    for proto in protos:
        _capnpc(
            name = "_gen_" + proto,
            proto = proto,
            outs = [proto + ".c++", proto + ".h"])

    # Create a cc_library which contains the generated code for all protos
    native.cc_library(
        name = name,
        srcs = [proto + ".c++" for proto in protos],
        hdrs = [proto + ".h" for proto in protos],
        data = protos,
        deps = ["@capnproto//:capnp_lite"] + deps,
        visibility = ["//visibility:public"],
    )

