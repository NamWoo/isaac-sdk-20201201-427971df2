#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################

# Some input files might be created by more complicated rules and placed in the genfiles folder.
# Let's move them to the current tree. We are operating on a safe environment here.
# TODO Will this create problems when we are not in a sandbox?
# FIXME This will only work for opt mode.
rsync -av bazel-out/k8-opt/bin/doc ./doc

SPHINXOPTS=
SOURCEDIR="."
BUILDDIR="_sphinx_build"

MASTER_DOC=${2/.rst/}

# The conf file needs to be at the root of the source folder
sed -e "s,MASTER_DOC,${MASTER_DOC},g; s,VERSION,$3,g" doc/sphinx/conf.py > conf.py

# Run the sphinx tool (needs to be installed previously)
sphinx-build -M latexpdf $SOURCEDIR $BUILDDIR $SPHINXOPTS

# Copy the generated PDF to the requested output location.
cp $BUILDDIR/latex/ISAAC.pdf $1
