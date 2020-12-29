"""
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# A custom library which supports building stub shared library from existing shared library.
# nm is used to extract all defined functions and awk used to generate dummy functions for them.
def isaac_shared_library_stub(name, original_library_so, out_c = None, **kwargs):
    if not out_c:
        out_c = "_%s.c" % name
    gen_name = out_c.replace(".", "_")
    native.genrule(
        name = gen_name,
        srcs = [
            original_library_so,
        ],
        outs = [
            out_c,
        ],
        cmd = "$(NM) -D $(SRCS) | grep \"\\ T\\ \" | sed -e \"/\\ _fini$$/d\" -e \"/\\ _init$$/d\" | awk '{print $$3}' | sort | uniq | awk '{print \"void \"$$0\"(){}\"}' > $(@)",
    )
    native.cc_binary(
        name = name,
        linkshared = True,
        srcs = [
            out_c,
        ],
    )
