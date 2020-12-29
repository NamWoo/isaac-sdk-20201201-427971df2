"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//doc/sphinx:sphinx.bzl", "sphinx", "sphinx_dep")

def isaac_doc_dep(**kwargs):
    """
    A documentation target with dependencies
    """
    sphinx_dep(**kwargs)

def isaac_doc(**kwargs):
    """
    A documentation to be built
    """
    sphinx(**kwargs)
