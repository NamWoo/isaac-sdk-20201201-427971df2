#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

# Create documentation by faking bazel into thinking that the component API has to be rebuilt:
# This is a temporary workaround until the create_component_api rule is improved to track the source
# code of all codelets which are put inside the documentation.
date +%s > doc/dummy.txt; bazel build doc; > doc/dummy.txt

# Let's open the PDF
evince bazel-bin/doc/isaac.pdf &

# Let's open th HTML
mkdir -p /tmp/isaac_doc
tar -xf bazel-bin/doc/isaac.tar.gz -C /tmp/isaac_doc
chromium-browser /tmp/isaac_doc/isaac/doc/index.html &
