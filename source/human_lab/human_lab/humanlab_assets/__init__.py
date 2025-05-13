# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""Package containing asset and sensor configurations."""

import os
import toml

##
import os

# Conveniences to other module directories via relative paths
HUMANLAB_ASSETS_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
"""Path to the extension source directory."""

HUMANLAB_ASSETS_DIR = os.path.join(HUMANLAB_ASSETS_EXT_DIR, "humanlab_assets/assets")
"""Path to the extension data directory."""


# HUMANLAB_ASSETS_METADATA = toml.load(os.path.join(HUMANLAB_ASSETS_EXT_DIR, "config", "extension.toml"))
# """Extension metadata dictionary parsed from the extension.toml file."""

# # Configure the module-level variables
# __version__ = HUMANLAB_ASSETS_METADATA["package"]["version"]

from .robots import *
from .sensors import *
