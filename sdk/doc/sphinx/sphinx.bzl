"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

def _impl(ctx):
    # We currently generate both PDF and HTML. Originally I wanted to provide an attribute which can
    # be used to select the document output format. However using a declare_file with a filename
    # generated from the output name and the extension doesn't seem to work. The rule would not run
    # thinking that there is nothing to do. There is probably solution for that, but in the meantime
    # we just create both.
    for fmt in ["latexpdf", "html"]:
        # Get tool and output file based on desired format
        tool = None
        out = None
        if fmt == "latexpdf":
            tool = ctx.executable._sphinx_latexpdf_tool
            out = ctx.outputs.latexpdf
        elif fmt == "html":
            tool = ctx.executable._sphinx_html_tool
            out = ctx.outputs.html
        else:
            fail("Invalid format")

        # Create an action to create the doc from the source files and all it's transitive dependecies.
        ctx.actions.run(
            inputs = ctx.files.srcs + ctx.files.deps + ctx.files._sphinx_tool_deps,
            outputs = [out],
            arguments = [out.path, ctx.file.master_doc.path, ctx.attr.version],
            progress_message = "Building documentation with SPHINX...",
            executable = tool,
            use_default_shell_env = True,
        )

# A rule to create documentation from restructured text using Sphinx.
sphinx = rule(
    implementation = _impl,
    attrs = {
        # The name of the generated file. This is for example the name of the PDF file (without)
        # extension when using the latexpdf format.
        "output_name": attr.string(
            mandatory = True,
        ),
        # Name of the master document for Sphinx
        "master_doc": attr.label(
            mandatory = True,
            allow_single_file = [".rst"],
        ),
        # Version string to be used in the documentation
        "version": attr.string(mandatory = True),
        # Additional restructured text source files used to create the documentation.
        "srcs": attr.label_list(allow_files = [".rst", ".jpg", ".png"]),
        # Additional dependencies which will be traveresed transitively to discover all restructured
        # text source files required to create this document.
        "deps": attr.label_list(),
        # This is the tool used to create the documentation in the latex PDF format
        "_sphinx_latexpdf_tool": attr.label(
            executable = True,
            cfg = "host",
            allow_files = True,
            default = Label("//doc/sphinx:sphinx_latexpdf"),
        ),
        # This is the tool used to create the documentation in the HTML format
        "_sphinx_html_tool": attr.label(
            executable = True,
            cfg = "host",
            allow_files = True,
            default = Label("//doc/sphinx:sphinx_html"),
        ),
        # TODO For some reason data dependencies of the sphinx tool don't show up in the sandbox.
        # Thus we just add them here explicitely
        "_sphinx_tool_deps": attr.label(
            default = Label("//doc/sphinx:sphinx_files"),
        ),
    },
    # TODO For some reason when declare_file is used instead of this the rule doesn't activate.
    outputs = {
        "latexpdf": "%{output_name}.pdf",
        "html": "%{output_name}.tar.gz",
    },
)

SphinxFiles = provider("transitive_sources")

def get_transitive_srcs(srcs, deps):
    """Obtain the source files for a target and its transitive dependencies.
    Args:
      srcs: a list of source files
      deps: a list of targets that are direct dependencies
    Returns:
      a collection of the transitive sources
    """
    return depset(srcs, transitive = [dep[SphinxFiles].transitive_sources for dep in deps])

def _sphinx_dep_impl(ctx):
    trans_srcs = get_transitive_srcs(ctx.files.srcs, ctx.attr.deps)
    return [
        SphinxFiles(transitive_sources = trans_srcs),
        DefaultInfo(files = trans_srcs),
    ]

# This rule is used to collect source files and track their dependencies. These can be used in
# the deps section of a `sphinx` rule to provide the document generator with all required input
# files.
sphinx_dep = rule(
    implementation = _sphinx_dep_impl,
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "deps": attr.label_list(),
    },
)
