# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Package containing task implementations for the extension."""

##
# Register Gym environments.
##

import os
import toml

from .manager_based import *


##
# Register Gym environments.
##

from .utils import import_packages

# The blacklist is used to prevent importing configs from sub-packages
# TODO(@ashwinvk): Remove pick_place from the blacklist once pinocchio from Isaac Sim is compatibility
_BLACKLIST_PKGS = ["utils", ".mdp", "pick_place"]
# Import all configs in this package
import_packages(__name__, _BLACKLIST_PKGS)
